//! Task scheduler and radio runtime backend.
//!
//! Modeled on esp-rtos's scheduler (TCB + context switch + ready queue +
//! blocking primitives), adapted for the WS63 app core (single-hart
//! `rv32imfc`). Key difference from esp32c3 (`rv32imc`): WS63 has the **F**
//! extension, so the context switch must also save/restore the callee-saved FP
//! registers `fs0..fs11` (the WiFi blob does floating-point RF math).
//!
//! Cooperative scheduling remains available. Priority scheduling additionally
//! permits an interrupt epilogue to switch immediately to a higher-priority task
//! made ready by the ISR and to time-slice equal-priority tasks.
//!
//! With the `embassy` feature, this crate also owns the process-wide
//! `embassy-time` driver. Embassy deadlines and RTOS sleep/time-slice deadlines
//! share the injected scheduler timer; applications must select
//! `embassy-time/tick-hz-1_000_000`. The injected millisecond clock gives the
//! driver 1 ms resolution while preserving the ecosystem-wide 1 MHz tick ABI.
//!
//! Layering: this crate depends only on `core`, `critical-section`, and the
//! runtime-neutral radio driver contract. The application injects allocation
//! and monotonic-time resources before any radio task can start.

#[cfg(test)]
use crate::BudgetSpec;
#[cfg(test)]
use crate::config::DYNAMIC_TASK_CAPACITY;
use crate::config::{
    ContractViolation, CooperativeConfig, CooperativeOnly, Diagnostics, Ported, PortedConfig,
    Resources, RuntimeHandle, SchedulerPort, StartError, TASK_SLOT_COUNT, TaskDiagnostic,
    TaskState,
};
#[cfg(target_arch = "riscv32")]
use crate::context::initialize_irq_frame;
use crate::context::{TaskContext, cooperative_context_switch_fallback};
use crate::scheduling::{BudgetExpiry, BudgetState};
use crate::{RunPolicy, TaskId};

mod sync;
#[cfg(test)]
use sync::release_mutex_locked;
use sync::{RtosMutex, Semaphore, enqueue_mutex_waiter, remove_mutex_waiter, remove_waiter};
mod time;
#[cfg(test)]
use time::{claim_timer_rearm_generation, earliest_deadline};
use time::{now_ms, rearm_timer};
pub use time::{on_software_interrupt, on_timer_interrupt, request_reschedule};

use core::cell::{Cell, RefCell, UnsafeCell};
use core::ffi::c_void;
use core::num::{NonZeroU32, NonZeroUsize};
#[cfg(feature = "embassy")]
use core::task::Waker;
use critical_section::Mutex;
#[cfg(feature = "embassy")]
use embassy_time_driver::Driver as EmbassyTimeDriver;
#[cfg(feature = "embassy")]
use embassy_time_queue_utils::Queue as EmbassyTimeQueue;
use hisi_rf_rtos_driver::{
    Error as DriverError, MutexHandle, Runtime, SemaphoreHandle, TaskConfig, WaitOutcome,
    WaitTimeout,
};

/// LiteOS-compatible priority levels: 0 is highest, 31 is lowest.
const PRIORITY_LEVELS: usize = 32;
/// Reserved scheduler slot for the always-eligible idle thread.
const IDLE_SLOT: usize = 1;
const IDLE_STACK_SIZE: usize = 2048;
/// Sentinel "no task" index for intrusive list links.
const NIL: usize = usize::MAX;
const TASK_SLOT_BITS: u32 = 8;
const TASK_SLOT_MASK: u32 = (1 << TASK_SLOT_BITS) - 1;
// TaskId's low byte is a versioned slot-index ABI. Increasing the table past
// this bound requires a new encoding, not truncation of the slot index.
const _: () = assert!(TASK_SLOT_COUNT <= TASK_SLOT_MASK as usize + 1);

/// Process-wide runtime state installed by one of the typed start functions.
#[derive(Clone, Copy)]
struct StartState {
    config: StartConfig,
    resources: Resources,
    port: Option<SchedulerPort>,
}

#[derive(Clone, Copy)]
struct StartConfig {
    minimum_stack_size: NonZeroUsize,
    radio_task_policy: RunPolicy,
    max_scheduler_lock_duration: Option<NonZeroU32>,
}

static START_STATE: Mutex<Cell<Option<StartState>>> = Mutex::new(Cell::new(None));

#[repr(C, align(16))]
struct IdleStack([u8; IDLE_STACK_SIZE]);

static mut IDLE_STACK: IdleStack = IdleStack([0; IDLE_STACK_SIZE]);

fn start_state() -> StartState {
    start_state_opt().expect("hisi-rtos must be started before radio runtime use")
}

fn start_state_opt() -> Option<StartState> {
    critical_section::with(|cs| START_STATE.borrow(cs).get())
}

fn allocate(size: usize) -> *mut u8 {
    // SAFETY: the application-provided allocator contract accepts arbitrary
    // non-zero task/control-block sizes and returns null on exhaustion.
    unsafe { (start_state().resources.allocate)(size) }
}

fn deallocate(pointer: *mut u8) {
    // SAFETY: every pointer passed here came from `allocate` and is released
    // exactly once after it is no longer in use.
    unsafe { (start_state().resources.deallocate)(pointer) }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Free,
    Ready,
    Running,
    Blocked,
    Sleeping,
    Throttled,
}

/// Task entry signature (matches the OSAL `osal_kthread_func`).
type TaskFn = extern "C" fn(*mut c_void) -> *mut c_void;

struct Tcb {
    ctx: TaskContext,
    saved_frame: usize,
    resume_generation: u32,
    state: State,
    stack: usize, // heap allocation addr to free on exit (0 for the main task)
    entry: Option<TaskFn>,
    arg: usize,   // task argument (*mut c_void stored as usize so Tcb is Send)
    next: usize,  // intrusive link: ready queue OR one wait queue
    wake_at: u64, // mask-ROM systick millisecond deadline
    waiting_sem: usize,
    waiting_mutex: usize,
    sem_granted: bool,
    base_priority: u8,
    priority: u8,
    inherited_waiters: [u8; PRIORITY_LEVELS],
    scheduler_lock_depth: u16,
    scheduler_lock_started_at: Option<u64>,
    identity_generation: u16,
    run_policy: RunPolicy,
    budget: BudgetState,
}
impl Tcb {
    const fn empty() -> Self {
        Tcb {
            ctx: TaskContext::zero(),
            saved_frame: 0,
            resume_generation: 0,
            state: State::Free,
            stack: 0,
            entry: None,
            arg: 0,
            next: NIL,
            wake_at: 0,
            waiting_sem: 0,
            waiting_mutex: 0,
            sem_granted: false,
            base_priority: (PRIORITY_LEVELS - 1) as u8,
            priority: (PRIORITY_LEVELS - 1) as u8,
            inherited_waiters: [0; PRIORITY_LEVELS],
            scheduler_lock_depth: 0,
            scheduler_lock_started_at: None,
            identity_generation: 0,
            run_policy: RunPolicy::Cooperative,
            budget: BudgetState::none(),
        }
    }
}

struct Sched {
    tasks: [Tcb; TASK_SLOT_COUNT],
    current: usize,
    ready_head: [usize; PRIORITY_LEVELS],
    ready_tail: [usize; PRIORITY_LEVELS],
    retired_stacks: [usize; TASK_SLOT_COUNT],
    retired_count: usize,
    slot_generations: [u16; TASK_SLOT_COUNT],
    time_slice_pending: bool,
    time_slice_deadline: u64,
    forced_next: usize,
    started: bool,
    diagnostics: Diagnostics,
}
impl Sched {
    const fn new() -> Self {
        const E: Tcb = Tcb::empty();
        Sched {
            tasks: [E; TASK_SLOT_COUNT],
            current: 0,
            ready_head: [NIL; PRIORITY_LEVELS],
            ready_tail: [NIL; PRIORITY_LEVELS],
            retired_stacks: [0; TASK_SLOT_COUNT],
            retired_count: 0,
            slot_generations: [0; TASK_SLOT_COUNT],
            time_slice_pending: false,
            time_slice_deadline: 0,
            forced_next: NIL,
            started: false,
            diagnostics: Diagnostics::EMPTY,
        }
    }

    fn diagnostics(&self) -> Diagnostics {
        let mut snapshot = self.diagnostics;
        snapshot.current_task = self.current;
        snapshot.current_lock_depth = self.tasks[self.current].scheduler_lock_depth;
        for task in &self.tasks {
            match task.state {
                State::Ready => snapshot.ready_tasks = snapshot.ready_tasks.saturating_add(1),
                State::Blocked => snapshot.blocked_tasks = snapshot.blocked_tasks.saturating_add(1),
                State::Sleeping => {
                    snapshot.sleeping_tasks = snapshot.sleeping_tasks.saturating_add(1)
                }
                State::Throttled => {
                    snapshot.throttled_tasks = snapshot.throttled_tasks.saturating_add(1)
                }
                State::Free | State::Running => {}
            }
        }
        snapshot.dynamic_used = self.tasks[(IDLE_SLOT + 1)..]
            .iter()
            .filter(|task| task.state != State::Free)
            .count() as u8;
        snapshot.dynamic_free = snapshot
            .dynamic_capacity
            .saturating_sub(snapshot.dynamic_used);
        snapshot
    }

    fn task_diagnostics(&self, output: &mut [TaskDiagnostic]) -> usize {
        let count = output.len().min(TASK_SLOT_COUNT);
        for (index, output) in output[..count].iter_mut().enumerate() {
            let task = &self.tasks[index];
            *output = TaskDiagnostic {
                task: index,
                generation: task.identity_generation,
                state: match task.state {
                    State::Free => TaskState::Free,
                    State::Ready => TaskState::Ready,
                    State::Running => TaskState::Running,
                    State::Blocked => TaskState::Blocked,
                    State::Sleeping => TaskState::Sleeping,
                    State::Throttled => TaskState::Throttled,
                },
                entry: task.entry.map_or(0, |entry| entry as usize),
                waiting_sem: task.waiting_sem,
                waiting_mutex: task.waiting_mutex,
                wake_at: task.wake_at,
                base_priority: task.base_priority,
                priority: task.priority,
                scheduler_lock_depth: task.scheduler_lock_depth,
                run_policy: task.run_policy,
                budget_remaining: task.budget.remaining(),
                budget_replenishes_at: task.budget.replenishes_at(),
            };
        }
        count
    }
    fn ready_priority(&self, task: usize) -> usize {
        self.tasks[task].priority as usize
    }
    fn ready_push(&mut self, i: usize) {
        let priority = self.ready_priority(i);
        self.tasks[i].next = NIL;
        if self.ready_tail[priority] == NIL {
            self.ready_head[priority] = i;
        } else {
            self.tasks[self.ready_tail[priority]].next = i;
        }
        self.ready_tail[priority] = i;
    }
    fn ready_pop(&mut self) -> usize {
        for priority in 0..PRIORITY_LEVELS {
            let i = self.ready_head[priority];
            if i != NIL {
                self.ready_head[priority] = self.tasks[i].next;
                if self.ready_head[priority] == NIL {
                    self.ready_tail[priority] = NIL;
                }
                self.tasks[i].next = NIL;
                return i;
            }
        }
        NIL
    }

    fn ready_pop_or_idle(&mut self) -> usize {
        let next = self.ready_pop();
        if next == NIL { IDLE_SLOT } else { next }
    }
    fn ready_remove(&mut self, task: usize) {
        let priority = self.ready_priority(task);
        let mut previous = NIL;
        let mut current = self.ready_head[priority];
        while current != NIL {
            if current == task {
                let next = self.tasks[current].next;
                if previous == NIL {
                    self.ready_head[priority] = next;
                } else {
                    self.tasks[previous].next = next;
                }
                if self.ready_tail[priority] == current {
                    self.ready_tail[priority] = previous;
                }
                self.tasks[current].next = NIL;
                return;
            }
            previous = current;
            current = self.tasks[current].next;
        }
    }

    fn set_effective_priority(&mut self, task: usize, priority: u8) {
        if self.tasks[task].priority == priority {
            return;
        }
        let ready = self.tasks[task].state == State::Ready;
        if ready {
            self.ready_remove(task);
        }
        self.tasks[task].priority = priority;
        if ready {
            self.ready_push(task);
        }
    }

    fn refresh_inherited_priority(&mut self, task: usize, depth: usize) {
        assert!(depth < TASK_SLOT_COUNT, "mutex inheritance cycle");
        let old = self.tasks[task].priority;
        let inherited = self.tasks[task]
            .inherited_waiters
            .iter()
            .position(|count| *count != 0)
            .map_or(self.tasks[task].base_priority, |priority| priority as u8);
        let new = self.tasks[task].base_priority.min(inherited);
        if old == new {
            return;
        }
        self.set_effective_priority(task, new);

        let waiting_mutex = self.tasks[task].waiting_mutex;
        if waiting_mutex != 0 {
            // SAFETY: a blocked task keeps its mutex alive; all mutation occurs
            // under this scheduler critical section.
            let state = unsafe { &mut *(*(waiting_mutex as *const RtosMutex)).inner.get() };
            remove_mutex_waiter(self, state, task);
            enqueue_mutex_waiter(self, state, task);
            if state.owner != NIL {
                self.replace_inheritance(state.owner, old, new, depth + 1);
            }
        }
    }

    fn replace_inheritance(&mut self, owner: usize, old: u8, new: u8, depth: usize) {
        if old == new {
            return;
        }
        let old_count = &mut self.tasks[owner].inherited_waiters[old as usize];
        *old_count = old_count.checked_sub(1).expect("missing inherited waiter");
        let new_count = &mut self.tasks[owner].inherited_waiters[new as usize];
        *new_count = new_count
            .checked_add(1)
            .expect("too many inherited waiters");
        self.refresh_inherited_priority(owner, depth);
    }

    fn add_inheritance(&mut self, owner: usize, priority: u8) {
        let count = &mut self.tasks[owner].inherited_waiters[priority as usize];
        *count = count.checked_add(1).expect("too many inherited waiters");
        self.diagnostics.priority_inheritances =
            self.diagnostics.priority_inheritances.saturating_add(1);
        self.refresh_inherited_priority(owner, 0);
    }

    fn remove_inheritance(&mut self, owner: usize, priority: u8) {
        let count = &mut self.tasks[owner].inherited_waiters[priority as usize];
        *count = count.checked_sub(1).expect("missing inherited waiter");
        self.refresh_inherited_priority(owner, 0);
    }
    fn take_yield_target(&mut self, current: usize) -> Option<usize> {
        let next = self.ready_pop();
        if next == NIL {
            return None;
        }
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some(next)
    }
    fn take_reschedule_target(&mut self, allow_equal_priority: bool) -> Option<(usize, usize)> {
        let current = self.current;
        if self.tasks[current].state != State::Running
            || self.tasks[current].scheduler_lock_depth != 0
            || !matches!(self.tasks[current].run_policy, RunPolicy::Preemptive { .. })
        {
            return None;
        }
        let current_priority = self.ready_priority(current);
        let end = if allow_equal_priority {
            current_priority.saturating_add(1)
        } else {
            current_priority
        };
        if !(0..end).any(|priority| self.ready_head[priority] != NIL) {
            return None;
        }
        let next = self.ready_pop();
        debug_assert!(next != NIL);
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some((current, next))
    }

    fn take_preemption_target(&mut self) -> Option<(usize, usize)> {
        self.take_reschedule_target(false)
    }

    #[cfg(test)]
    fn take_irq_epilogue_target(&mut self, interrupt_depth: u16) -> Option<(usize, usize)> {
        if !self.started || interrupt_depth != 0 {
            return None;
        }
        let time_slice = self.time_slice_pending;
        let current_priority = self.tasks[self.current].priority;
        let target = if self.tasks[self.current].state != State::Running {
            Some((self.current, self.ready_pop_or_idle()))
        } else {
            self.take_reschedule_target(time_slice)
        };
        if target.is_some() {
            self.diagnostics.irq_preemptions = self.diagnostics.irq_preemptions.saturating_add(1);
            if time_slice
                && target.is_some_and(|(_, next)| self.tasks[next].priority == current_priority)
            {
                self.diagnostics.time_slice_preemptions =
                    self.diagnostics.time_slice_preemptions.saturating_add(1);
            }
            self.time_slice_pending = false;
        }
        target
    }

    #[cfg(target_arch = "riscv32")]
    fn schedule_from_trap(&mut self, frame: usize, interrupt_depth: u16, now: u64) -> usize {
        if !self.started || interrupt_depth != 0 {
            return frame;
        }

        let current = self.current;
        let current_priority = self.tasks[current].priority;
        let time_slice = self.time_slice_pending;
        let target = if self.forced_next != NIL {
            let next = self.forced_next;
            self.forced_next = NIL;
            Some((current, next))
        } else if self.tasks[current].state != State::Running {
            Some((current, self.ready_pop_or_idle()))
        } else {
            self.take_reschedule_target(time_slice)
        };

        let Some((previous, next)) = target else {
            return frame;
        };

        self.account_switch(previous, next, now);

        if self.tasks[previous].state != State::Free {
            self.tasks[previous].saved_frame = frame;
        }

        let next_frame = self.tasks[next].saved_frame;
        assert!(next_frame != 0, "target task has no saved trap frame");
        self.tasks[next].saved_frame = 0;
        self.tasks[next].state = State::Running;
        self.tasks[next].resume_generation = self.tasks[next].resume_generation.wrapping_add(1);
        self.current = next;
        self.diagnostics.context_switches = self.diagnostics.context_switches.saturating_add(1);
        self.diagnostics.irq_preemptions = self.diagnostics.irq_preemptions.saturating_add(1);
        if time_slice && self.tasks[next].priority == current_priority {
            self.diagnostics.time_slice_preemptions =
                self.diagnostics.time_slice_preemptions.saturating_add(1);
        }
        self.time_slice_pending = false;
        next_frame
    }
    fn retire_stack(&mut self, stack: usize) {
        if stack != 0 {
            debug_assert!(self.retired_count < TASK_SLOT_COUNT);
            self.retired_stacks[self.retired_count] = stack;
            self.retired_count += 1;
        }
    }
    fn lock_current(&mut self, now: u64) -> Result<(), DriverError> {
        let task = &mut self.tasks[self.current];
        if task.scheduler_lock_depth == 0 {
            task.scheduler_lock_started_at = Some(now);
        }
        task.scheduler_lock_depth = task
            .scheduler_lock_depth
            .checked_add(1)
            .ok_or(DriverError::Runtime)?;
        self.diagnostics.scheduler_locks = self.diagnostics.scheduler_locks.saturating_add(1);
        Ok(())
    }
    fn unlock_current(&mut self) -> Result<(), DriverError> {
        let task = &mut self.tasks[self.current];
        if task.scheduler_lock_depth == 0 {
            return Err(DriverError::InvalidContext);
        }
        task.scheduler_lock_depth -= 1;
        if task.scheduler_lock_depth == 0 {
            task.scheduler_lock_started_at = None;
        }
        Ok(())
    }

    fn unlock_current_and_take_preemption(
        &mut self,
        now: u64,
    ) -> Result<Option<(usize, usize)>, DriverError> {
        self.unlock_current()?;
        if self.tasks[self.current].scheduler_lock_depth == 0
            && self.tasks[self.current].budget.lock_overrun_pending()
        {
            let current = self.current;
            self.tasks[current]
                .budget
                .throttle_after_lock_overrun(now)
                .expect("pending budget overrun has a budget policy");
            self.tasks[current].state = State::Throttled;
            self.diagnostics.budget_throttles = self.diagnostics.budget_throttles.saturating_add(1);
            let next = self.ready_pop_or_idle();
            return Ok(Some((current, next)));
        }
        let target = self.take_reschedule_target(self.time_slice_pending);
        if target.is_some() {
            self.time_slice_pending = false;
        }
        Ok(target)
    }
    fn wake_sleepers(&mut self, now: u64) {
        for i in 0..TASK_SLOT_COUNT {
            if self.tasks[i].state == State::Sleeping && now >= self.tasks[i].wake_at {
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
                self.diagnostics.sleeper_wakes = self.diagnostics.sleeper_wakes.saturating_add(1);
            } else if self.tasks[i].state == State::Blocked
                && self.tasks[i].waiting_sem != 0
                && self.tasks[i].wake_at != 0
                && now >= self.tasks[i].wake_at
            {
                let sem = self.tasks[i].waiting_sem as *const Semaphore;
                // SAFETY: a timed waiter keeps the semaphore alive for the
                // duration of the call, and all queue mutation is serialized by
                // the scheduler critical section.
                let sem_state = unsafe { &mut *(*sem).inner.get() };
                remove_waiter(self, sem_state, i);
                self.tasks[i].waiting_sem = 0;
                self.tasks[i].sem_granted = false;
                self.tasks[i].wake_at = 0;
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
                self.diagnostics.semaphore_timeouts =
                    self.diagnostics.semaphore_timeouts.saturating_add(1);
            } else if self.tasks[i].state == State::Blocked
                && self.tasks[i].waiting_mutex != 0
                && self.tasks[i].wake_at != 0
                && now >= self.tasks[i].wake_at
            {
                let mutex = self.tasks[i].waiting_mutex as *const RtosMutex;
                // SAFETY: the waiter keeps the mutex alive and the scheduler
                // critical section serializes its queue and owner state.
                let state = unsafe { &mut *(*mutex).inner.get() };
                remove_mutex_waiter(self, state, i);
                if state.owner != NIL {
                    self.remove_inheritance(state.owner, self.tasks[i].priority);
                }
                self.tasks[i].waiting_mutex = 0;
                self.tasks[i].sem_granted = false;
                self.tasks[i].wake_at = 0;
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
            }
        }
        self.replenish_budgets(now);
    }

    fn replenish_budgets(&mut self, now: u64) {
        for i in 0..TASK_SLOT_COUNT {
            if self.tasks[i].state == State::Throttled && self.tasks[i].budget.replenish_if_due(now)
            {
                self.tasks[i].state = State::Ready;
                self.ready_push(i);
                self.diagnostics.budget_replenishments =
                    self.diagnostics.budget_replenishments.saturating_add(1);
            }
        }
    }

    fn on_timer(
        &mut self,
        now: u64,
        max_scheduler_lock_duration: NonZeroU32,
    ) -> Option<ContractViolation> {
        self.wake_sleepers(now);
        let current = self.current;
        if self.tasks[current].state != State::Running {
            return None;
        }
        let locked = self.tasks[current].scheduler_lock_depth != 0;
        match self.tasks[current].budget.on_timer(now, locked) {
            BudgetExpiry::ThrottleNow => {
                self.tasks[current].state = State::Throttled;
                self.diagnostics.budget_exhaustions =
                    self.diagnostics.budget_exhaustions.saturating_add(1);
                self.diagnostics.budget_throttles =
                    self.diagnostics.budget_throttles.saturating_add(1);
            }
            BudgetExpiry::DeferredBySchedulerLock => {
                self.diagnostics.budget_exhaustions =
                    self.diagnostics.budget_exhaustions.saturating_add(1);
                self.diagnostics.budget_lock_overruns =
                    self.diagnostics.budget_lock_overruns.saturating_add(1);
            }
            BudgetExpiry::NotBudgeted | BudgetExpiry::StillAvailable => {}
        }
        let task = &self.tasks[current];
        let violation = task.scheduler_lock_started_at.and_then(|started_at| {
            let held_ms = now.saturating_sub(started_at);
            (held_ms >= u64::from(max_scheduler_lock_duration.get())).then_some(
                ContractViolation::SchedulerLockOverrun {
                    task_slot: current,
                    held_ms,
                    limit_ms: max_scheduler_lock_duration.get(),
                },
            )
        });
        if violation.is_some() {
            self.diagnostics.scheduler_lock_overruns =
                self.diagnostics.scheduler_lock_overruns.saturating_add(1);
        }
        violation
    }

    fn account_switch(&mut self, previous: usize, next: usize, now: u64) {
        self.tasks[previous].budget.on_switch_out(now);
        self.tasks[next].budget.on_dispatch(now);
        self.time_slice_deadline = 0;
    }
    fn alloc_dynamic_slot(&mut self) -> Result<usize, DriverError> {
        ((IDLE_SLOT + 1)..TASK_SLOT_COUNT)
            .find(|&i| self.tasks[i].state == State::Free)
            .ok_or(DriverError::NoTaskSlots)
    }

    fn set_run_policy(&mut self, slot: usize, policy: RunPolicy, now: u64) {
        let was_ready = self.tasks[slot].state == State::Ready;
        let was_throttled = self.tasks[slot].state == State::Throttled;
        if was_ready {
            self.ready_remove(slot);
        }
        let task = &mut self.tasks[slot];
        task.run_policy = policy;
        task.budget = BudgetState::for_policy(policy, now);
        if task.state == State::Running {
            task.budget.on_dispatch(now);
        } else if was_throttled {
            // The old budget no longer owns eligibility after a policy change.
            // A new Budgeted policy also starts with a fresh full budget.
            task.state = State::Ready;
        }
        if was_ready || was_throttled {
            self.ready_push(slot);
        }
    }

    fn current_switch_guard(&self) -> Result<usize, DriverError> {
        let current = self.current;
        if self.tasks[current].scheduler_lock_depth != 0 {
            return Err(DriverError::InvalidContext);
        }
        Ok(current)
    }

    fn earliest_wake_deadline(&self) -> Option<u64> {
        self.tasks
            .iter()
            .filter(|task| {
                matches!(task.state, State::Sleeping | State::Blocked) && task.wake_at != 0
            })
            .map(|task| task.wake_at)
            .min()
    }

    fn earliest_budget_deadline(&self) -> Option<u64> {
        let current_deadline = (self.tasks[self.current].state == State::Running)
            .then(|| self.tasks[self.current].budget.exhaustion_deadline())
            .flatten();
        let replenish_deadline = self
            .tasks
            .iter()
            .filter(|task| task.state == State::Throttled)
            .map(|task| task.budget.replenishes_at())
            .filter(|deadline| *deadline != 0)
            .min();
        current_deadline.into_iter().chain(replenish_deadline).min()
    }

    fn scheduler_lock_deadline(&self, max_duration: NonZeroU32) -> Option<u64> {
        let task = &self.tasks[self.current];
        (task.state == State::Running && task.scheduler_lock_depth != 0)
            .then_some(task.scheduler_lock_started_at)
            .flatten()
            .map(|started_at| started_at.saturating_add(u64::from(max_duration.get())))
    }

    fn next_time_slice_deadline(&mut self, now: u64) -> Option<u64> {
        let RunPolicy::Preemptive { time_slice } = self.tasks[self.current].run_policy else {
            self.time_slice_deadline = 0;
            return None;
        };
        if !self.has_equal_priority_ready(self.tasks[self.current].priority) {
            self.time_slice_deadline = 0;
            return None;
        }
        if self.time_slice_deadline == 0 {
            self.time_slice_deadline = now.saturating_add(time_slice.get() as u64);
        }
        Some(self.time_slice_deadline)
    }

    fn has_equal_priority_ready(&self, priority: u8) -> bool {
        self.ready_head[priority as usize] != NIL
    }
}

static SCHED: Mutex<RefCell<Sched>> = Mutex::new(RefCell::new(Sched::new()));
static INTERRUPT_DEPTH: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));

/// Marks entry into a target interrupt handler.
///
/// ISR-safe wakeups may make tasks ready, but task context switching is
/// deferred until after interrupt exit.
#[doc(hidden)]
pub fn interrupt_enter() {
    critical_section::with(|cs| {
        let depth = INTERRUPT_DEPTH.borrow(cs);
        depth.set(depth.get().saturating_add(1));
    });
}

/// Marks exit from a target interrupt handler.
#[doc(hidden)]
pub fn interrupt_exit() {
    critical_section::with(|cs| {
        let depth = INTERRUPT_DEPTH.borrow(cs);
        debug_assert!(depth.get() != 0);
        depth.set(depth.get().saturating_sub(1));
    });
}

/// Deferred scheduling point called by `hisi-riscv-rt` on its IRQ stack.
///
/// The symbol deliberately remains private Rust API: the runtime assembly owns
/// the ABI contract and provides a weak no-op when this crate is not linked.
#[cfg(target_arch = "riscv32")]
#[unsafe(no_mangle)]
extern "C" fn __hisi_irq_epilogue(encoded_frame: usize) -> usize {
    let now = now_ms();
    let next_frame = critical_section::with(|cs| {
        let depth = INTERRUPT_DEPTH.borrow(cs).get();
        SCHED
            .borrow_ref_mut(cs)
            .schedule_from_trap(encoded_frame, depth, now)
    });
    // The selected task may have a different budget/time-slice deadline. Arm
    // the shared timer only after scheduler state names that task as current.
    rearm_timer();
    next_frame
}

fn reclaim_retired_stacks() {
    let (stacks, count) = critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        let stacks = scheduler.retired_stacks;
        let count = scheduler.retired_count;
        scheduler.retired_stacks = [0; TASK_SLOT_COUNT];
        scheduler.retired_count = 0;
        (stacks, count)
    });
    for stack in &stacks[..count] {
        deallocate(*stack as *mut u8);
    }
}

const fn switch_delivery_is_valid(
    ported: bool,
    in_interrupt: bool,
    machine_interrupts_enabled: bool,
) -> bool {
    !ported || in_interrupt || machine_interrupts_enabled
}

#[cfg(target_arch = "riscv32")]
fn machine_interrupts_enabled() -> bool {
    let mstatus: usize;
    // SAFETY: reading mstatus has no memory side effects and does not alter the
    // interrupt state. MIE is bit 3 in the standard machine-status register.
    unsafe {
        core::arch::asm!("csrr {mstatus}, mstatus", mstatus = out(reg) mstatus, options(nomem, nostack));
    }
    mstatus & (1 << 3) != 0
}

#[cfg(not(target_arch = "riscv32"))]
fn machine_interrupts_enabled() -> bool {
    true
}

fn ensure_switch_delivery() -> Result<(), DriverError> {
    let ported = start_state().port.is_some();
    let in_interrupt = critical_section::with(|cs| INTERRUPT_DEPTH.borrow(cs).get() != 0);
    switch_delivery_is_valid(ported, in_interrupt, machine_interrupts_enabled())
        .then_some(())
        .ok_or(DriverError::InvalidContext)
}

/// First-run trampoline: a freshly restored task lands here (`ctx.mepc`),
/// runs its entry, then exits. Reads its own entry/arg from the current TCB.
extern "C" fn trampoline() -> ! {
    let (_slot, entry, arg, _stack) = critical_section::with(|cs| {
        let s = SCHED.borrow_ref(cs);
        let t = &s.tasks[s.current];
        (s.current, t.entry, t.arg, t.stack)
    });
    if let Some(f) = entry {
        f(arg as *mut c_void);
    }
    task_exit();
}

extern "C" fn idle_entry(_arg: *mut c_void) -> *mut c_void {
    loop {
        // Equal-priority user work must not be starved by the reserved idle
        // thread. Higher-priority wakeups are handled by the IRQ epilogue.
        let _ = yield_now();
        #[cfg(target_arch = "riscv32")]
        // SAFETY: the idle thread has no outstanding MMIO transaction or lock;
        // an enabled scheduler/target interrupt wakes the hart.
        unsafe {
            core::arch::asm!("wfi", options(nomem, nostack));
        }
        #[cfg(not(target_arch = "riscv32"))]
        core::hint::spin_loop();
    }
}

/// Initialize the scheduler, adopting the current execution as the main task
/// (slot 0). Idempotent.
fn init() {
    let now = now_ms();
    #[cfg(target_arch = "riscv32")]
    let trap_scheduling = start_state().port.is_some();
    #[cfg(target_arch = "riscv32")]
    let (initial_tp, initial_fcsr): (usize, u32) = unsafe {
        let (tp, fcsr);
        core::arch::asm!(
            "mv {tp}, tp",
            "frcsr {fcsr}",
            tp = out(reg) tp,
            fcsr = out(reg) fcsr,
            options(nomem, nostack),
        );
        (tp, fcsr)
    };
    // SAFETY: `IDLE_STACK` is reserved for the single scheduler instance. Its
    // address is taken once during initialization and no Rust reference is
    // created for the mutable static.
    let idle_stack = unsafe { core::ptr::addr_of_mut!(IDLE_STACK.0).cast::<u8>() as usize };
    let idle_top = (idle_stack + IDLE_STACK_SIZE) & !0xf;
    critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        if s.started {
            return;
        }
        s.tasks[0].state = State::Running;
        s.slot_generations[0] = 1;
        s.tasks[0].identity_generation = 1;
        s.tasks[0].run_policy = RunPolicy::Cooperative;
        s.tasks[0].budget = BudgetState::for_policy(RunPolicy::Cooperative, now);
        s.tasks[0].budget.on_dispatch(now);
        s.current = 0;

        let idle = &mut s.tasks[IDLE_SLOT];
        idle.ctx = TaskContext::zero();
        #[cfg(target_arch = "riscv32")]
        {
            idle.ctx.tp = initial_tp as u32;
            idle.ctx.mstatus = 0x7880;
            idle.ctx.fcsr = initial_fcsr;
        }
        let idle_trampoline: extern "C" fn() -> ! = trampoline;
        idle.ctx.mepc = idle_trampoline as usize as u32;
        idle.ctx.sp = idle_top as u32;
        #[cfg(target_arch = "riscv32")]
        if trap_scheduling {
            // SAFETY: the reserved static stack is exclusively owned by the
            // idle task and is never deallocated.
            idle.saved_frame = unsafe {
                initialize_irq_frame(idle_top, idle_trampoline as usize, initial_tp, initial_fcsr)
            };
        }
        idle.state = State::Ready;
        idle.stack = 0;
        idle.entry = Some(idle_entry);
        idle.arg = 0;
        idle.base_priority = (PRIORITY_LEVELS - 1) as u8;
        idle.priority = (PRIORITY_LEVELS - 1) as u8;
        idle.identity_generation = 1;
        idle.run_policy = RunPolicy::Cooperative;
        idle.budget = BudgetState::none();
        s.slot_generations[IDLE_SLOT] = 1;
        s.started = true;
    });
}

/// Spawn a task, distinguishing task-table exhaustion from allocator failure.
fn spawn(
    entry: TaskFn,
    arg: *mut c_void,
    stack_size: usize,
    priority: u8,
    run_policy: RunPolicy,
) -> Result<usize, DriverError> {
    init();
    reclaim_retired_stacks();
    let size = stack_size.max(start_state().config.minimum_stack_size.get());
    let stack = allocate(size);
    if stack.is_null() {
        return Err(DriverError::ResourceExhausted);
    }
    #[cfg(target_arch = "riscv32")]
    let trap_scheduling = start_state().port.is_some();
    // 16-byte aligned stack top.
    let top = (stack as usize + size) & !0xf;
    #[cfg(target_arch = "riscv32")]
    let (initial_tp, initial_fcsr): (usize, u32) = unsafe {
        let (tp, fcsr);
        core::arch::asm!(
            "mv {tp}, tp",
            "frcsr {fcsr}",
            tp = out(reg) tp,
            fcsr = out(reg) fcsr,
            options(nomem, nostack),
        );
        (tp, fcsr)
    };
    let now = now_ms();
    let slot = critical_section::with(|cs| -> Result<usize, DriverError> {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let i = s.alloc_dynamic_slot()?;
        s.slot_generations[i] = s.slot_generations[i].wrapping_add(1).max(1);
        let identity_generation = s.slot_generations[i];
        let t = &mut s.tasks[i];
        t.ctx = TaskContext::zero();
        #[cfg(target_arch = "riscv32")]
        {
            t.ctx.tp = initial_tp as u32;
            t.ctx.mstatus = 0x7880;
            t.ctx.fcsr = initial_fcsr;
        }
        // Cast through a fn pointer (not a direct fn-item->int cast).
        let tramp: extern "C" fn() -> ! = trampoline;
        t.ctx.mepc = tramp as usize as u32;
        t.ctx.sp = top as u32;
        #[cfg(target_arch = "riscv32")]
        if trap_scheduling {
            // SAFETY: `stack..top` is this task's freshly allocated stack and
            // no task can observe it until the TCB is queued below.
            t.saved_frame =
                unsafe { initialize_irq_frame(top, tramp as usize, initial_tp, initial_fcsr) };
        }
        t.state = State::Ready;
        t.stack = stack as usize;
        t.entry = Some(entry);
        t.arg = arg as usize;
        t.wake_at = 0;
        t.base_priority = priority;
        t.priority = priority;
        t.identity_generation = identity_generation;
        t.run_policy = run_policy;
        t.budget = BudgetState::for_policy(run_policy, now);
        s.ready_push(i);
        Ok(i)
    });
    if slot.is_err() {
        deallocate(stack);
    } else {
        rearm_timer();
    }
    slot
}

/// Switch away from `prev` to the next ready task, busy-idling (waking sleepers)
/// until one is runnable. `prev`'s state must already be set by the caller
/// (Ready+queued for yield, Blocked for a wait, Free for exit).
fn switch_to(prev: usize, next: usize) {
    if next == prev {
        critical_section::with(|cs| {
            SCHED.borrow_ref_mut(cs).tasks[next].state = State::Running;
        });
        rearm_timer();
        return;
    }
    if let Some(port) = start_state().port {
        assert!(
            machine_interrupts_enabled(),
            "ported context switch requested while machine interrupts are disabled"
        );
        let generation = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            assert_eq!(s.current, prev, "switch source is not the running task");
            assert_eq!(s.forced_next, NIL, "a trap switch is already pending");
            assert!(
                s.tasks[next].saved_frame != 0,
                "target task has no trap frame"
            );
            s.forced_next = next;
            s.tasks[prev].resume_generation
        });
        rearm_timer();
        (port.pend_reschedule)();
        loop {
            let resumed = critical_section::with(|cs| {
                let s = SCHED.borrow_ref(cs);
                s.current == prev && s.tasks[prev].resume_generation != generation
            });
            if resumed {
                return;
            }
            core::hint::spin_loop();
        }
    }
    let now = now_ms();
    let (op, np) = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.account_switch(prev, next, now);
        s.diagnostics.context_switches = s.diagnostics.context_switches.saturating_add(1);
        s.tasks[next].state = State::Running;
        s.current = next;
        (
            core::ptr::addr_of_mut!(s.tasks[prev].ctx),
            core::ptr::addr_of!(s.tasks[next].ctx),
        )
    });
    rearm_timer();
    // SAFETY: contexts live in the static SCHED (stable address); single-hart,
    // and the lock is released so the resumed task can re-enter the scheduler.
    unsafe { cooperative_context_switch_fallback(op, np) };
}

fn switch_away(prev: usize) {
    if start_state().port.is_some()
        && critical_section::with(|cs| {
            let s = SCHED.borrow_ref(cs);
            s.current == prev && s.tasks[prev].state == State::Running
        })
    {
        return;
    }
    let now = now_ms();
    let next = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.wake_sleepers(now);
        s.ready_pop_or_idle()
    });
    switch_to(prev, next);
}

/// Yield the CPU: requeue the current task and run the next ready one.
fn yield_now() -> Result<(), DriverError> {
    ensure_switch_delivery()?;
    let now = now_ms();
    let target = critical_section::with(|cs| -> Result<_, DriverError> {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let cur = s.current_switch_guard()?;
        s.diagnostics.yields = s.diagnostics.yields.saturating_add(1);
        s.wake_sleepers(now);
        // A cooperative yield promises progress to another ready task. Select
        // that task before requeueing the current one; otherwise a strict
        // priority queue would immediately select the yielding high-priority
        // task again and starve lower-priority initialization work.
        Ok(s.take_yield_target(cur).map(|next| (cur, next)))
    })?;
    if let Some((prev, next)) = target {
        switch_to(prev, next);
    }
    reclaim_retired_stacks();
    Ok(())
}

/// Sleep the current task for `ms` milliseconds (cooperative; wakes when a later
/// schedule sees the deadline pass).
fn sleep_ms(ms: u32) -> Result<(), DriverError> {
    if ms == 0 {
        return yield_now();
    }
    ensure_switch_delivery()?;
    let wake_at = now_ms().saturating_add(ms as u64);
    let prev = critical_section::with(|cs| -> Result<_, DriverError> {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let cur = s.current_switch_guard()?;
        s.diagnostics.sleeps = s.diagnostics.sleeps.saturating_add(1);
        s.tasks[cur].state = State::Sleeping;
        s.tasks[cur].wake_at = wake_at;
        Ok(cur)
    })?;
    rearm_timer();
    switch_away(prev);
    reclaim_retired_stacks();
    Ok(())
}

/// Current task slot index (its "pid"/"tid").
fn current_id() -> usize {
    critical_section::with(|cs| SCHED.borrow_ref(cs).current)
}

fn encode_task_id(slot: usize, generation: u16) -> Result<TaskId, DriverError> {
    if slot >= TASK_SLOT_COUNT || slot > TASK_SLOT_MASK as usize || generation == 0 {
        return Err(DriverError::InvalidHandle);
    }
    let slot = u32::try_from(slot).map_err(|_| DriverError::Runtime)?;
    Ok(TaskId::from_raw(
        (u32::from(generation) << TASK_SLOT_BITS) | slot,
    ))
}

fn decode_task_id(task: TaskId) -> Result<(usize, u16), DriverError> {
    let raw = task.into_raw();
    let slot = usize::try_from(raw & TASK_SLOT_MASK).map_err(|_| DriverError::InvalidHandle)?;
    let generation =
        u16::try_from(raw >> TASK_SLOT_BITS).map_err(|_| DriverError::InvalidHandle)?;
    if slot >= TASK_SLOT_COUNT || generation == 0 {
        return Err(DriverError::InvalidHandle);
    }
    Ok((slot, generation))
}

fn task_exit() -> ! {
    assert!(
        ensure_switch_delivery().is_ok(),
        "task returned while ported context-switch delivery was unavailable"
    );
    // Retire the stack before switching away. A resumed task drains the retired
    // list only after it is running on a different stack.
    let prev = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let cur = s.current;
        assert_eq!(
            s.tasks[cur].scheduler_lock_depth, 0,
            "task returned while holding the scheduler lock"
        );
        assert_ne!(cur, IDLE_SLOT, "idle task returned");
        let stack = s.tasks[cur].stack;
        s.tasks[cur] = Tcb::empty();
        s.retire_stack(stack);
        cur
    });
    switch_away(prev);
    unreachable!()
}

struct HisiRuntime;

static RUNTIME: HisiRuntime = HisiRuntime;

/// Starts the port-less cooperative-only scheduler profile.
///
/// This profile switches only when a task explicitly yields, blocks, sleeps,
/// exits, or otherwise returns control to the scheduler. IRQ wakeups do not
/// immediately preempt, and sleep deadlines are observed at the next scheduling
/// point. Budgeted and Preemptive policy APIs are intentionally absent from the
/// returned capability.
pub fn start_cooperative(
    config: CooperativeConfig,
    resources: Resources,
) -> Result<RuntimeHandle<CooperativeOnly>, StartError> {
    start_inner(
        StartConfig {
            minimum_stack_size: config.minimum_stack_size,
            radio_task_policy: RunPolicy::Cooperative,
            max_scheduler_lock_duration: None,
        },
        resources,
        None,
    )?;
    Ok(RuntimeHandle::new())
}

/// Starts the scheduler with a target timer and deferred-reschedule port.
pub fn start_with_port(
    config: PortedConfig,
    resources: Resources,
    port: SchedulerPort,
) -> Result<RuntimeHandle<Ported>, StartError> {
    start_inner(
        StartConfig {
            minimum_stack_size: config.minimum_stack_size,
            radio_task_policy: config.radio_task_policy,
            max_scheduler_lock_duration: Some(config.max_scheduler_lock_duration),
        },
        resources,
        Some(port),
    )?;
    Ok(RuntimeHandle::new())
}

fn start_inner(
    config: StartConfig,
    resources: Resources,
    port: Option<SchedulerPort>,
) -> Result<(), StartError> {
    let already_started = critical_section::with(|cs| {
        let state = START_STATE.borrow(cs);
        if state.get().is_some() {
            true
        } else {
            state.set(Some(StartState {
                config,
                resources,
                port,
            }));
            false
        }
    });
    if already_started {
        return Err(StartError::AlreadyStarted);
    }
    if let Err(error) = hisi_rf_rtos_driver::install(&RUNTIME) {
        critical_section::with(|cs| START_STATE.borrow(cs).set(None));
        return Err(StartError::Driver(error));
    }
    init();
    rearm_timer();
    Ok(())
}

/// Snapshot scheduler counters without changing task state or scheduling.
pub fn diagnostics() -> Diagnostics {
    critical_section::with(|cs| SCHED.borrow_ref(cs).diagnostics())
}

/// Copies scheduler slot state into `output` without changing scheduling.
pub fn task_diagnostics(output: &mut [TaskDiagnostic]) -> usize {
    critical_section::with(|cs| SCHED.borrow_ref(cs).task_diagnostics(output))
}

fn set_task_run_policy_inner(task: TaskId, policy: RunPolicy) -> Result<(), DriverError> {
    let (slot, generation) = decode_task_id(task)?;
    let now = now_ms();
    critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        if scheduler.tasks[slot].state == State::Free
            || scheduler.tasks[slot].identity_generation != generation
        {
            return Err(DriverError::InvalidHandle);
        }
        scheduler.set_run_policy(slot, policy, now);
        Ok(())
    })?;
    rearm_timer();
    Ok(())
}

impl RuntimeHandle<Ported> {
    /// Changes a live task's per-thread run policy.
    ///
    /// The opaque task identity includes a generation, so a handle retained
    /// after task exit cannot mutate a later task that reuses the same slot.
    pub fn set_task_run_policy(&self, task: TaskId, policy: RunPolicy) -> Result<(), DriverError> {
        set_task_run_policy_inner(task, policy)
    }
}

fn semaphore_from_handle(handle: SemaphoreHandle) -> &'static Semaphore {
    let pointer = handle.into_raw().get() as *const Semaphore;
    // SAFETY: this backend creates handles only from heap-allocated Semaphore
    // objects and the driver contract requires users to stop all access before
    // destroy.
    unsafe { &*pointer }
}

fn mutex_from_handle(handle: MutexHandle) -> &'static RtosMutex {
    let pointer = handle.into_raw().get() as *const RtosMutex;
    // SAFETY: this backend creates handles only from live RtosMutex allocations.
    unsafe { &*pointer }
}

impl Runtime for HisiRuntime {
    fn spawn(
        &self,
        entry: hisi_rf_rtos_driver::TaskEntry,
        arg: *mut c_void,
        config: TaskConfig,
    ) -> Result<TaskId, DriverError> {
        if config.priority as usize >= PRIORITY_LEVELS {
            return Err(DriverError::Runtime);
        }
        let slot = spawn(
            entry,
            arg,
            config.stack_size.get(),
            config.priority,
            start_state().config.radio_task_policy,
        )?;
        let generation =
            critical_section::with(|cs| SCHED.borrow_ref(cs).tasks[slot].identity_generation);
        encode_task_id(slot, generation)
    }

    fn yield_now(&self) -> Result<(), DriverError> {
        yield_now()
    }

    fn sleep_ms(&self, milliseconds: NonZeroU32) -> Result<(), DriverError> {
        sleep_ms(milliseconds.get())
    }

    fn current_task(&self) -> Result<TaskId, DriverError> {
        let slot = current_id();
        let generation =
            critical_section::with(|cs| SCHED.borrow_ref(cs).tasks[slot].identity_generation);
        encode_task_id(slot, generation)
    }

    fn set_task_priority(&self, task: TaskId, priority: u8) -> Result<(), DriverError> {
        if priority as usize >= PRIORITY_LEVELS {
            return Err(DriverError::Runtime);
        }
        let (slot, generation) = decode_task_id(task)?;
        critical_section::with(|cs| {
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let Some(tcb) = scheduler.tasks.get(slot) else {
                return Err(DriverError::InvalidHandle);
            };
            if tcb.state == State::Free || tcb.identity_generation != generation {
                return Err(DriverError::InvalidHandle);
            }
            scheduler.tasks[slot].base_priority = priority;
            scheduler.refresh_inherited_priority(slot, 0);
            Ok(())
        })
    }

    fn lock_scheduler(&self) -> Result<(), DriverError> {
        ensure_switch_delivery()?;
        let now = now_ms();
        let result = critical_section::with(|cs| SCHED.borrow_ref_mut(cs).lock_current(now));
        if result.is_ok() {
            rearm_timer();
        }
        result
    }

    fn unlock_scheduler(&self) -> Result<(), DriverError> {
        ensure_switch_delivery()?;
        let now = now_ms();
        let preemption = critical_section::with(|cs| {
            SCHED
                .borrow_ref_mut(cs)
                .unlock_current_and_take_preemption(now)
        })?;
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
        rearm_timer();
        Ok(())
    }

    fn interrupt_enter(&self) -> Result<(), DriverError> {
        crate::interrupt_enter();
        Ok(())
    }

    fn interrupt_exit(&self) -> Result<(), DriverError> {
        crate::interrupt_exit();
        Ok(())
    }

    fn semaphore_create(&self, initial: u32) -> Result<SemaphoreHandle, DriverError> {
        let count = i32::try_from(initial).map_err(|_| DriverError::Runtime)?;
        let pointer = allocate(core::mem::size_of::<Semaphore>()) as *mut Semaphore;
        let raw = NonZeroUsize::new(pointer as usize).ok_or(DriverError::ResourceExhausted)?;
        // SAFETY: the RF allocator guarantees size/alignment for Semaphore.
        unsafe { pointer.write(Semaphore::new(count)) };
        // SAFETY: `raw` identifies this live allocation until destroy.
        Ok(unsafe { SemaphoreHandle::from_raw(raw) })
    }

    fn semaphore_down(
        &self,
        semaphore: SemaphoreHandle,
        timeout: WaitTimeout,
    ) -> Result<WaitOutcome, DriverError> {
        let timeout_ms = match timeout {
            WaitTimeout::NoWait => 0,
            WaitTimeout::Milliseconds(value) => value.get(),
            WaitTimeout::Forever => u32::MAX,
        };
        Ok(
            if semaphore_from_handle(semaphore).down_timeout(timeout_ms)? {
                WaitOutcome::Acquired
            } else {
                WaitOutcome::TimedOut
            },
        )
    }

    fn semaphore_up(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        semaphore_from_handle(semaphore).up()
    }

    unsafe fn semaphore_destroy(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        deallocate(semaphore.into_raw().get() as *mut u8);
        Ok(())
    }

    fn mutex_create(&self) -> Result<MutexHandle, DriverError> {
        let pointer = allocate(core::mem::size_of::<RtosMutex>()) as *mut RtosMutex;
        let raw = NonZeroUsize::new(pointer as usize).ok_or(DriverError::ResourceExhausted)?;
        // SAFETY: the RF allocator guarantees size/alignment for RtosMutex.
        unsafe { pointer.write(RtosMutex::new()) };
        // SAFETY: raw identifies the live allocation until destroy.
        Ok(unsafe { MutexHandle::from_raw(raw) })
    }

    fn mutex_lock(
        &self,
        mutex: MutexHandle,
        timeout: WaitTimeout,
    ) -> Result<WaitOutcome, DriverError> {
        let timeout_ms = match timeout {
            WaitTimeout::NoWait => 0,
            WaitTimeout::Milliseconds(value) => value.get(),
            WaitTimeout::Forever => u32::MAX,
        };
        Ok(if mutex_from_handle(mutex).lock(timeout_ms)? {
            WaitOutcome::Acquired
        } else {
            WaitOutcome::TimedOut
        })
    }

    fn mutex_unlock(&self, mutex: MutexHandle) -> Result<(), DriverError> {
        mutex_from_handle(mutex).unlock()
    }

    unsafe fn mutex_destroy(&self, mutex: MutexHandle) -> Result<(), DriverError> {
        let busy = critical_section::with(|_| {
            // SAFETY: caller promises the handle is live during this check.
            let state = unsafe { &*mutex_from_handle(mutex).inner.get() };
            state.owner != NIL || state.wait_head != NIL
        });
        if busy {
            return Err(DriverError::InvalidContext);
        }
        deallocate(mutex.into_raw().get() as *mut u8);
        Ok(())
    }
}

#[cfg(test)]
mod tests;
