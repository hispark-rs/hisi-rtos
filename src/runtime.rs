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
#[cfg(target_arch = "riscv32")]
use crate::context::initialize_irq_frame;
use crate::context::{TaskContext, cooperative_context_switch_fallback};
use crate::scheduling::{BudgetExpiry, BudgetState};
use crate::{RunPolicy, TaskId};

use core::cell::{Cell, RefCell, UnsafeCell};
use core::ffi::c_void;
use core::marker::PhantomData;
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

/// The application thread adopted when the scheduler starts.
const ADOPTED_MAIN_TASKS: usize = 1;
/// Scheduler-owned idle threads that cannot be allocated by applications.
const INTERNAL_IDLE_TASKS: usize = 1;
/// Dynamic task slots available through the runtime contract.
pub const DYNAMIC_TASK_CAPACITY: usize = 15;
/// Total scheduler slots, including adopted and internal threads.
const TASK_SLOT_COUNT: usize = ADOPTED_MAIN_TASKS + INTERNAL_IDLE_TASKS + DYNAMIC_TASK_CAPACITY;
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

/// Platform services owned by the application rather than the radio stack.
#[derive(Clone, Copy)]
pub struct Resources {
    /// Allocates `size` bytes, returning null on exhaustion.
    pub allocate: unsafe fn(size: usize) -> *mut u8,
    /// Releases an allocation previously returned by [`Resources::allocate`].
    pub deallocate: unsafe fn(pointer: *mut u8),
    /// Returns a wrapping monotonic millisecond counter.
    pub monotonic_ms: fn() -> u64,
}

/// Target timer and deferred-reschedule operations injected by the application.
///
/// The callbacks must not call user code. `arm_timer` receives a non-zero
/// relative delay no greater than `max_timer_delay`; the RTOS chunks longer
/// deadlines. The WS63 implementation uses TIMER_INT0 and SOFT_INT0.
#[derive(Clone, Copy)]
pub struct SchedulerPort {
    /// Largest delay accepted by [`SchedulerPort::arm_timer`].
    pub max_timer_delay: NonZeroU32,
    /// Arms or replaces the one-shot scheduler timer.
    pub arm_timer: fn(NonZeroU32),
    /// Stops the scheduler timer when no deadline remains.
    pub disarm_timer: fn(),
    /// Pends the target's deferred-reschedule interrupt.
    pub pend_reschedule: fn(),
    /// Handles a scheduler contract violation without returning.
    ///
    /// The callback runs outside the scheduler critical section. A development
    /// port may report the violation before halting; a production port may
    /// record a crash summary and reset. It must never resume the violating
    /// task.
    pub contract_violation: fn(ContractViolation) -> !,
}

/// A fail-stop violation detected by the target-backed scheduler.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ContractViolation {
    /// A task held the scheduler lock beyond the configured bound.
    SchedulerLockOverrun {
        /// Scheduler slot that owns the lock.
        task_slot: usize,
        /// Observed lock hold time in milliseconds.
        held_ms: u64,
        /// Configured upper bound in milliseconds.
        limit_ms: u32,
    },
}

/// Scheduler configuration for the port-less cooperative-only profile.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct CooperativeConfig {
    /// Minimum stack allocation applied to every task request.
    pub minimum_stack_size: NonZeroUsize,
}

/// Scheduler configuration for a target with timer and reschedule ports.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct PortedConfig {
    /// Minimum stack allocation applied to every task request.
    pub minimum_stack_size: NonZeroUsize,
    /// Default per-thread policy for tasks created through
    /// `hisi-rf-rtos-driver`.
    ///
    /// This is explicit because every task created through that contract is a
    /// vendor/runtime worker. Native Rust and Embassy work remains cooperative
    /// unless the application assigns another policy.
    pub radio_task_policy: RunPolicy,
    /// Maximum time a task may continuously hold the scheduler lock.
    ///
    /// Interrupts remain enabled while the lock is held, so the shared timer
    /// detects expiry and invokes [`SchedulerPort::contract_violation`].
    pub max_scheduler_lock_duration: NonZeroU32,
}

/// Capability marker for the port-less cooperative-only runtime.
pub enum CooperativeOnly {}

/// Capability marker for a runtime with timer and deferred-reschedule ports.
pub enum Ported {}

/// Proof that the process-wide runtime started with capability `Mode`.
#[must_use = "retain the runtime handle to access mode-specific capabilities"]
pub struct RuntimeHandle<Mode> {
    _mode: PhantomData<fn() -> Mode>,
}

impl<Mode> RuntimeHandle<Mode> {
    const fn new() -> Self {
        Self { _mode: PhantomData }
    }
}

/// Read-only scheduler counters and task-state census for bring-up diagnostics.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Diagnostics {
    pub context_switches: u32,
    pub irq_preemptions: u32,
    pub timer_interrupts: u32,
    pub software_interrupts: u32,
    pub time_slice_preemptions: u32,
    pub priority_inheritances: u32,
    pub yields: u32,
    pub sleeps: u32,
    pub sleeper_wakes: u32,
    pub semaphore_blocks: u32,
    pub semaphore_wakes: u32,
    pub semaphore_timeouts: u32,
    pub scheduler_locks: u32,
    pub scheduler_lock_overruns: u32,
    pub budget_exhaustions: u32,
    pub budget_throttles: u32,
    pub budget_replenishments: u32,
    pub budget_lock_overruns: u32,
    pub current_task: usize,
    pub ready_tasks: u8,
    pub blocked_tasks: u8,
    pub sleeping_tasks: u8,
    pub throttled_tasks: u8,
    /// Scheduler-owned main and idle slots.
    pub internal_tasks: u8,
    /// Dynamic task slots promised by this runtime build.
    pub dynamic_capacity: u8,
    /// Dynamic slots currently occupied by live tasks.
    pub dynamic_used: u8,
    /// Dynamic slots currently available for task creation.
    pub dynamic_free: u8,
    pub current_lock_depth: u16,
}

/// State of one scheduler slot in a read-only bring-up snapshot.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum TaskState {
    #[default]
    Free,
    Ready,
    Running,
    Blocked,
    Sleeping,
    Throttled,
}

/// Read-only scheduler slot details for diagnosing blocked radio workers.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TaskDiagnostic {
    pub task: usize,
    pub generation: u16,
    pub state: TaskState,
    pub entry: usize,
    pub waiting_sem: usize,
    pub waiting_mutex: usize,
    pub wake_at: u64,
    pub base_priority: u8,
    /// Effective priority after inheritance (lower value is higher priority).
    pub priority: u8,
    pub scheduler_lock_depth: u16,
    pub run_policy: RunPolicy,
    pub budget_remaining: u32,
    pub budget_replenishes_at: u64,
}

impl Diagnostics {
    const EMPTY: Self = Self {
        context_switches: 0,
        irq_preemptions: 0,
        timer_interrupts: 0,
        software_interrupts: 0,
        time_slice_preemptions: 0,
        priority_inheritances: 0,
        yields: 0,
        sleeps: 0,
        sleeper_wakes: 0,
        semaphore_blocks: 0,
        semaphore_wakes: 0,
        semaphore_timeouts: 0,
        scheduler_locks: 0,
        scheduler_lock_overruns: 0,
        budget_exhaustions: 0,
        budget_throttles: 0,
        budget_replenishments: 0,
        budget_lock_overruns: 0,
        current_task: 0,
        ready_tasks: 0,
        blocked_tasks: 0,
        sleeping_tasks: 0,
        throttled_tasks: 0,
        internal_tasks: (ADOPTED_MAIN_TASKS + INTERNAL_IDLE_TASKS) as u8,
        dynamic_capacity: DYNAMIC_TASK_CAPACITY as u8,
        dynamic_used: 0,
        dynamic_free: DYNAMIC_TASK_CAPACITY as u8,
        current_lock_depth: 0,
    };
}

impl Default for CooperativeConfig {
    fn default() -> Self {
        Self {
            // The WS63 vendor archive was built with a 24 KiB LiteOS default.
            minimum_stack_size: NonZeroUsize::new(24 * 1024).unwrap(),
        }
    }
}

impl Default for PortedConfig {
    fn default() -> Self {
        Self {
            minimum_stack_size: CooperativeConfig::default().minimum_stack_size,
            radio_task_policy: RunPolicy::Cooperative,
            max_scheduler_lock_duration: NonZeroU32::new(100).unwrap(),
        }
    }
}

/// Failure to start the process-wide runtime.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StartError {
    /// This RTOS instance was already started.
    AlreadyStarted,
    /// Another runtime already owns the radio driver contract.
    Driver(DriverError),
}

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
static TIMER_REARM_GENERATION: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

#[repr(C, align(16))]
struct IdleStack([u8; IDLE_STACK_SIZE]);

static mut IDLE_STACK: IdleStack = IdleStack([0; IDLE_STACK_SIZE]);

#[cfg(feature = "embassy")]
static EMBASSY_TIME_QUEUE: Mutex<RefCell<EmbassyTimeQueue>> =
    Mutex::new(RefCell::new(EmbassyTimeQueue::new()));

#[cfg(feature = "embassy")]
const EMBASSY_TICKS_PER_MILLISECOND: u64 = embassy_time_driver::TICK_HZ / 1_000;

#[cfg(feature = "embassy")]
const _: () = assert!(embassy_time_driver::TICK_HZ.is_multiple_of(1_000));

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

fn now_ms() -> u64 {
    (start_state().resources.monotonic_ms)()
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

fn earliest_deadline(
    wake_deadline: Option<u64>,
    slice_deadline: Option<u64>,
    budget_deadline: Option<u64>,
    scheduler_lock_deadline: Option<u64>,
    embassy_deadline: Option<u64>,
) -> Option<u64> {
    wake_deadline
        .into_iter()
        .chain(slice_deadline)
        .chain(budget_deadline)
        .chain(scheduler_lock_deadline)
        .chain(embassy_deadline)
        .min()
}

#[cfg(feature = "embassy")]
fn embassy_now_ticks() -> u64 {
    start_state_opt()
        .map(|state| (state.resources.monotonic_ms)().saturating_mul(EMBASSY_TICKS_PER_MILLISECOND))
        .unwrap_or(0)
}

#[cfg(feature = "embassy")]
fn embassy_next_expiration_locked(
    cs: critical_section::CriticalSection<'_>,
    now_ms: u64,
) -> Option<u64> {
    let now_ticks = now_ms.saturating_mul(EMBASSY_TICKS_PER_MILLISECOND);
    let deadline_ticks = EMBASSY_TIME_QUEUE
        .borrow(cs)
        .borrow_mut()
        .next_expiration(now_ticks);
    if deadline_ticks == u64::MAX {
        return None;
    }
    let remaining_ticks = deadline_ticks.saturating_sub(now_ticks);
    let delay_ms = remaining_ticks.saturating_add(EMBASSY_TICKS_PER_MILLISECOND - 1)
        / EMBASSY_TICKS_PER_MILLISECOND;
    Some(now_ms.saturating_add(delay_ms.max(1)))
}

#[cfg(not(feature = "embassy"))]
fn embassy_next_expiration_locked(
    _cs: critical_section::CriticalSection<'_>,
    _now: u64,
) -> Option<u64> {
    None
}

fn claim_timer_rearm_generation(cell: &Cell<u64>) -> u64 {
    let generation = cell.get().wrapping_add(1);
    cell.set(generation);
    generation
}

fn rearm_timer() {
    let state = start_state();
    let Some(port) = state.port else {
        return;
    };
    loop {
        let now = (state.resources.monotonic_ms)();
        let (generation, deadline) = critical_section::with(|cs| {
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let lock_limit = state
                .config
                .max_scheduler_lock_duration
                .expect("ported runtime requires a scheduler-lock bound");
            let deadline = earliest_deadline(
                scheduler.earliest_wake_deadline(),
                scheduler.next_time_slice_deadline(now),
                scheduler.earliest_budget_deadline(),
                scheduler.scheduler_lock_deadline(lock_limit),
                embassy_next_expiration_locked(cs, now),
            );
            let generation = claim_timer_rearm_generation(TIMER_REARM_GENERATION.borrow(cs));
            (generation, deadline)
        });

        if let Some(deadline) = deadline {
            let remaining = deadline.saturating_sub(now).max(1);
            let delay = remaining.min(port.max_timer_delay.get() as u64) as u32;
            (port.arm_timer)(NonZeroU32::new(delay).unwrap());
        } else {
            (port.disarm_timer)();
        }

        let still_current =
            critical_section::with(|cs| TIMER_REARM_GENERATION.borrow(cs).get() == generation);
        if still_current {
            return;
        }
    }
}

/// Handles expiry of the injected scheduler timer.
///
/// The target handler must acknowledge its hardware source, call
/// [`interrupt_enter`], invoke this function, then call [`interrupt_exit`]. The
/// runtime epilogue performs any resulting context switch on the interrupted
/// task's stack.
pub fn on_timer_interrupt() {
    let now = now_ms();
    let state = start_state();
    let lock_limit = state
        .config
        .max_scheduler_lock_duration
        .expect("timer interrupt requires a ported runtime");
    let violation = critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        scheduler.diagnostics.timer_interrupts =
            scheduler.diagnostics.timer_interrupts.saturating_add(1);
        let violation = scheduler.on_timer(now, lock_limit);
        if scheduler.time_slice_deadline != 0 && now >= scheduler.time_slice_deadline {
            scheduler.time_slice_pending = true;
            scheduler.time_slice_deadline = 0;
        }
        violation
    });
    if let Some(violation) = violation {
        let port = state
            .port
            .expect("timer interrupt requires a scheduler port");
        (port.contract_violation)(violation);
    }
    rearm_timer();
}

/// Records delivery of the injected deferred-reschedule interrupt.
///
/// The target handler clears its hardware source and brackets this call with
/// [`interrupt_enter`] / [`interrupt_exit`]. Scheduling remains in the runtime
/// IRQ epilogue.
pub fn on_software_interrupt() {
    critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        scheduler.diagnostics.software_interrupts =
            scheduler.diagnostics.software_interrupts.saturating_add(1);
    });
}

/// Requests an immediate deferred scheduling point through the target port.
pub fn request_reschedule() {
    if let Some(port) = start_state().port {
        rearm_timer();
        (port.pend_reschedule)();
    }
}

#[cfg(feature = "embassy")]
struct HisiEmbassyTimeDriver;

#[cfg(feature = "embassy")]
impl EmbassyTimeDriver for HisiEmbassyTimeDriver {
    fn now(&self) -> u64 {
        embassy_now_ticks()
    }

    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let changed = critical_section::with(|cs| {
            EMBASSY_TIME_QUEUE
                .borrow(cs)
                .borrow_mut()
                .schedule_wake(at, waker)
        });
        if changed && start_state_opt().and_then(|state| state.port).is_some() {
            rearm_timer();
        }
    }
}

#[cfg(feature = "embassy")]
embassy_time_driver::time_driver_impl!(
    static EMBASSY_TIME_DRIVER: HisiEmbassyTimeDriver = HisiEmbassyTimeDriver
);

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

// ── Counting semaphore (blocks via the scheduler) ───────────────────────────

/// A counting semaphore. Tasks block in [`Semaphore::down`] when the count is 0
/// and are woken by [`Semaphore::up`]. Backs `osal_sem_*` / `osal_wait_*` /
/// `osal_mutex_*`.
///
/// `&self` methods + interior mutability so it can be a `static` or heap object
/// shared across tasks; all state is touched only inside the scheduler critical
/// section (single-hart exclusive). Waiters are queued on the per-task `next`
/// link (a task is on at most one queue — ready OR one wait queue — at a time).
struct Semaphore {
    inner: UnsafeCell<SemState>,
}
struct SemState {
    count: i32,
    wait_head: usize,
    wait_tail: usize,
}

fn enqueue_waiter(sched: &mut Sched, state: &mut SemState, task: usize) {
    sched.tasks[task].next = NIL;
    if state.wait_tail == NIL {
        state.wait_head = task;
    } else {
        sched.tasks[state.wait_tail].next = task;
    }
    state.wait_tail = task;
}

fn remove_waiter(sched: &mut Sched, state: &mut SemState, task: usize) {
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL {
        if current == task {
            let next = sched.tasks[current].next;
            if previous == NIL {
                state.wait_head = next;
            } else {
                sched.tasks[previous].next = next;
            }
            if state.wait_tail == current {
                state.wait_tail = previous;
            }
            sched.tasks[current].next = NIL;
            return;
        }
        previous = current;
        current = sched.tasks[current].next;
    }
}
// SAFETY: `inner` is only accessed inside `critical_section::with` on a single
// hart, which serialises every access.
unsafe impl Sync for Semaphore {}

impl Semaphore {
    /// Create a semaphore with initial `count`.
    const fn new(count: i32) -> Self {
        Semaphore {
            inner: UnsafeCell::new(SemState {
                count,
                wait_head: NIL,
                wait_tail: NIL,
            }),
        }
    }

    /// Acquire (P). Consumes a count if available, else blocks until [`up`] hands
    /// one off. Direct-handoff semantics: being woken == being granted, so there
    /// is no re-check loop (the only thing that unblocks a waiter is `up`).
    ///
    /// [`up`]: Semaphore::up
    fn down(&self) -> Result<(), DriverError> {
        let switch_delivery_available = ensure_switch_delivery().is_ok();
        let block = critical_section::with(|cs| -> Result<_, DriverError> {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section (single hart).
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                Ok(false)
            } else {
                if !switch_delivery_available {
                    return Err(DriverError::InvalidContext);
                }
                let cur = s.current_switch_guard()?;
                s.diagnostics.semaphore_blocks = s.diagnostics.semaphore_blocks.saturating_add(1);
                s.tasks[cur].state = State::Blocked;
                s.tasks[cur].wake_at = 0;
                s.tasks[cur].waiting_sem = self as *const Self as usize;
                s.tasks[cur].sem_granted = false;
                enqueue_waiter(s, st, cur);
                Ok(true)
            }
        })?;
        if block {
            // Parked on this sem's wait queue; `up` will move us back to Ready
            // (== the grant). When we resume here, we already hold the count.
            switch_away(current_id());
            critical_section::with(|cs| {
                let s = &mut *SCHED.borrow_ref_mut(cs);
                s.tasks[s.current].sem_granted = false;
            });
        }
        Ok(())
    }

    /// Acquire with a timeout (ms). Returns `true` if a count was obtained,
    /// `false` if the deadline passed first. `u32::MAX` (wait-forever) blocks
    /// like [`down`](Semaphore::down).
    ///
    /// The waiter is linked into the semaphore queue so [`up`](Self::up) can
    /// hand the grant directly to it. The scheduler removes it from that queue
    /// if the mask-ROM systick deadline wins first.
    fn down_timeout(&self, timeout_ms: u32) -> Result<bool, DriverError> {
        if timeout_ms == u32::MAX {
            self.down()?;
            return Ok(true);
        }
        let switch_delivery_available = ensure_switch_delivery().is_ok();
        let deadline = now_ms().saturating_add(timeout_ms as u64);
        let current = critical_section::with(|cs| -> Result<_, DriverError> {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                return Ok(None);
            }
            if timeout_ms == 0 {
                return Ok(Some(NIL));
            }
            if !switch_delivery_available {
                return Err(DriverError::InvalidContext);
            }
            let cur = s.current_switch_guard()?;
            s.diagnostics.semaphore_blocks = s.diagnostics.semaphore_blocks.saturating_add(1);
            s.tasks[cur].state = State::Blocked;
            s.tasks[cur].wake_at = deadline;
            s.tasks[cur].waiting_sem = self as *const Self as usize;
            s.tasks[cur].sem_granted = false;
            enqueue_waiter(s, st, cur);
            Ok(Some(cur))
        })?;
        if matches!(current, Some(slot) if slot != NIL) {
            rearm_timer();
        }
        match current {
            None => Ok(true),
            Some(NIL) => Ok(false),
            Some(current) => {
                switch_away(current);
                Ok(critical_section::with(|cs| {
                    let s = &mut *SCHED.borrow_ref_mut(cs);
                    let granted = s.tasks[s.current].sem_granted;
                    s.tasks[s.current].sem_granted = false;
                    granted
                }))
            }
        }
    }

    /// Release (V). Wakes one waiter if any, else increments the count.
    fn up(&self) -> Result<(), DriverError> {
        let machine_interrupts_enabled = machine_interrupts_enabled();
        let mut defer_reschedule = false;
        let preemption = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            let w = st.wait_head;
            if w != NIL {
                st.wait_head = s.tasks[w].next;
                if st.wait_head == NIL {
                    st.wait_tail = NIL;
                }
                s.tasks[w].next = NIL;
                s.tasks[w].wake_at = 0;
                s.tasks[w].waiting_sem = 0;
                s.tasks[w].sem_granted = true;
                s.tasks[w].state = State::Ready;
                s.ready_push(w);
                s.diagnostics.semaphore_wakes = s.diagnostics.semaphore_wakes.saturating_add(1);
            } else {
                st.count += 1;
            }
            let interrupt_depth = INTERRUPT_DEPTH.borrow(cs).get();
            if interrupt_depth == 0 && machine_interrupts_enabled {
                s.take_preemption_target()
            } else {
                defer_reschedule = interrupt_depth == 0 && !machine_interrupts_enabled;
                None
            }
        });
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
        if defer_reschedule {
            request_reschedule();
        }
        rearm_timer();
        Ok(())
    }
}

// Recursive mutex with priority-ordered waiters and priority inheritance.
struct RtosMutex {
    inner: UnsafeCell<MutexState>,
}

struct MutexState {
    owner: usize,
    depth: u32,
    wait_head: usize,
    wait_tail: usize,
}

// SAFETY: all state is accessed under the single-hart scheduler critical section.
unsafe impl Sync for RtosMutex {}

impl RtosMutex {
    const fn new() -> Self {
        Self {
            inner: UnsafeCell::new(MutexState {
                owner: NIL,
                depth: 0,
                wait_head: NIL,
                wait_tail: NIL,
            }),
        }
    }

    fn lock(&self, timeout_ms: u32) -> Result<bool, DriverError> {
        let switch_delivery_available = ensure_switch_delivery().is_ok();
        let deadline = now_ms().saturating_add(timeout_ms as u64);
        let current = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            let current = s.current;
            // SAFETY: exclusive under the scheduler critical section.
            let state = unsafe { &mut *self.inner.get() };
            if state.owner == current {
                state.depth = state.depth.checked_add(1).ok_or(DriverError::Runtime)?;
                return Ok(None);
            }
            if state.owner == NIL {
                state.owner = current;
                state.depth = 1;
                return Ok(None);
            }
            if timeout_ms == 0 {
                return Ok(Some(NIL));
            }
            if !switch_delivery_available {
                return Err(DriverError::InvalidContext);
            }
            s.current_switch_guard()?;
            if mutex_chain_contains(s, state.owner, current) {
                return Err(DriverError::InvalidContext);
            }

            let owner = state.owner;
            s.tasks[current].state = State::Blocked;
            s.tasks[current].wake_at = if timeout_ms == u32::MAX { 0 } else { deadline };
            s.tasks[current].waiting_mutex = self as *const Self as usize;
            s.tasks[current].sem_granted = false;
            enqueue_mutex_waiter(s, state, current);
            s.add_inheritance(owner, s.tasks[current].priority);
            Ok(Some(current))
        })?;

        if matches!(current, Some(slot) if slot != NIL) {
            rearm_timer();
        }
        match current {
            None => Ok(true),
            Some(NIL) => Ok(false),
            Some(current) => {
                switch_away(current);
                Ok(critical_section::with(|cs| {
                    let s = &mut *SCHED.borrow_ref_mut(cs);
                    let granted = s.tasks[s.current].sem_granted;
                    s.tasks[s.current].sem_granted = false;
                    granted
                }))
            }
        }
    }

    fn unlock(&self) -> Result<(), DriverError> {
        let machine_interrupts_enabled = machine_interrupts_enabled();
        let mut defer_reschedule = false;
        let preemption = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            let current = s.current;
            // SAFETY: exclusive under the scheduler critical section.
            let state = unsafe { &mut *self.inner.get() };
            if state.owner != current || state.depth == 0 {
                return Err(DriverError::InvalidContext);
            }
            state.depth -= 1;
            if state.depth != 0 {
                return Ok(None);
            }
            release_mutex_locked(s, state, current);

            let interrupt_depth = INTERRUPT_DEPTH.borrow(cs).get();
            Ok(if interrupt_depth == 0 && machine_interrupts_enabled {
                s.take_preemption_target()
            } else {
                defer_reschedule = interrupt_depth == 0 && !machine_interrupts_enabled;
                None
            })
        })?;
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
        if defer_reschedule {
            request_reschedule();
        }
        rearm_timer();
        Ok(())
    }
}

fn release_mutex_locked(sched: &mut Sched, state: &mut MutexState, owner: usize) {
    let next = pop_mutex_waiter(sched, state);
    if next == NIL {
        state.owner = NIL;
        return;
    }

    // Every waiter donated to the old owner. Remove those contributions before
    // changing ownership, including the waiter receiving the direct handoff.
    sched.remove_inheritance(owner, sched.tasks[next].priority);
    let mut waiter = state.wait_head;
    while waiter != NIL {
        sched.remove_inheritance(owner, sched.tasks[waiter].priority);
        waiter = sched.tasks[waiter].next;
    }

    state.owner = next;
    state.depth = 1;
    sched.tasks[next].waiting_mutex = 0;
    sched.tasks[next].wake_at = 0;
    sched.tasks[next].sem_granted = true;
    sched.tasks[next].state = State::Ready;
    sched.ready_push(next);

    // Remaining waiters now donate to the new owner.
    waiter = state.wait_head;
    while waiter != NIL {
        sched.add_inheritance(next, sched.tasks[waiter].priority);
        waiter = sched.tasks[waiter].next;
    }
}

fn enqueue_mutex_waiter(sched: &mut Sched, state: &mut MutexState, task: usize) {
    let priority = sched.tasks[task].priority;
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL && sched.tasks[current].priority <= priority {
        previous = current;
        current = sched.tasks[current].next;
    }
    sched.tasks[task].next = current;
    if previous == NIL {
        state.wait_head = task;
    } else {
        sched.tasks[previous].next = task;
    }
    if current == NIL {
        state.wait_tail = task;
    }
}

fn remove_mutex_waiter(sched: &mut Sched, state: &mut MutexState, task: usize) {
    let mut previous = NIL;
    let mut current = state.wait_head;
    while current != NIL {
        if current == task {
            let next = sched.tasks[current].next;
            if previous == NIL {
                state.wait_head = next;
            } else {
                sched.tasks[previous].next = next;
            }
            if state.wait_tail == current {
                state.wait_tail = previous;
            }
            sched.tasks[current].next = NIL;
            return;
        }
        previous = current;
        current = sched.tasks[current].next;
    }
}

fn pop_mutex_waiter(sched: &mut Sched, state: &mut MutexState) -> usize {
    let task = state.wait_head;
    if task != NIL {
        state.wait_head = sched.tasks[task].next;
        if state.wait_head == NIL {
            state.wait_tail = NIL;
        }
        sched.tasks[task].next = NIL;
    }
    task
}

fn mutex_chain_contains(sched: &Sched, mut owner: usize, sought: usize) -> bool {
    for _ in 0..TASK_SLOT_COUNT {
        if owner == sought {
            return true;
        }
        let waiting = sched.tasks[owner].waiting_mutex;
        if waiting == 0 {
            return false;
        }
        // SAFETY: a blocked task keeps the mutex alive under the scheduler lock.
        let state = unsafe { &*(*(waiting as *const RtosMutex)).inner.get() };
        if state.owner == NIL {
            return false;
        }
        owner = state.owner;
    }
    true
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
mod tests {
    use super::*;

    fn ready_task(scheduler: &mut Sched, slot: usize, priority: u8) {
        scheduler.tasks[slot].state = State::Ready;
        scheduler.tasks[slot].priority = priority;
        scheduler.tasks[slot].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(1).unwrap(),
        };
        scheduler.ready_push(slot);
    }

    #[test]
    fn dynamic_allocation_reserves_main_and_idle_slots() {
        let mut scheduler = Sched::new();

        for dynamic in 0..DYNAMIC_TASK_CAPACITY {
            let slot = IDLE_SLOT + 1 + dynamic;
            assert_eq!(scheduler.alloc_dynamic_slot(), Ok(slot));
            scheduler.tasks[slot].state = State::Ready;
        }
        assert_eq!(
            scheduler.alloc_dynamic_slot(),
            Err(DriverError::NoTaskSlots)
        );
        assert_eq!(scheduler.tasks[IDLE_SLOT].state, State::Free);

        let diagnostics = scheduler.diagnostics();
        assert_eq!(diagnostics.internal_tasks, 2);
        assert_eq!(diagnostics.dynamic_capacity, 15);
        assert_eq!(diagnostics.dynamic_used, 15);
        assert_eq!(diagnostics.dynamic_free, 0);
    }

    #[test]
    fn idle_is_selected_only_when_the_ready_queues_are_empty() {
        let mut scheduler = Sched::new();

        assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT);
        ready_task(&mut scheduler, IDLE_SLOT + 1, (PRIORITY_LEVELS - 1) as u8);
        assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT + 1);
        assert_eq!(scheduler.ready_pop_or_idle(), IDLE_SLOT);
    }

    #[test]
    fn scheduler_lock_rejects_switching_or_blocking_entry_points() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;

        assert_eq!(scheduler.current_switch_guard(), Ok(0));
        scheduler.lock_current(0).unwrap();
        assert_eq!(
            scheduler.current_switch_guard(),
            Err(DriverError::InvalidContext)
        );
        scheduler.unlock_current().unwrap();
        assert_eq!(scheduler.current_switch_guard(), Ok(0));
    }

    #[test]
    fn ported_thread_switch_requires_mie_but_irq_epilogue_does_not() {
        assert!(switch_delivery_is_valid(false, false, false));
        assert!(switch_delivery_is_valid(true, false, true));
        assert!(switch_delivery_is_valid(true, true, false));
        assert!(!switch_delivery_is_valid(true, false, false));
    }

    #[test]
    fn ready_queue_prefers_lower_priority_number_and_keeps_fifo() {
        let mut scheduler = Sched::new();
        ready_task(&mut scheduler, 1, 8);
        ready_task(&mut scheduler, 2, 4);
        ready_task(&mut scheduler, 3, 4);

        assert_eq!(scheduler.ready_pop(), 2);
        assert_eq!(scheduler.ready_pop(), 3);
        assert_eq!(scheduler.ready_pop(), 1);
        assert_eq!(scheduler.ready_pop(), NIL);
    }

    #[test]
    fn ready_task_can_move_between_priority_queues() {
        let mut scheduler = Sched::new();
        ready_task(&mut scheduler, 1, 8);
        ready_task(&mut scheduler, 2, 4);

        scheduler.ready_remove(1);
        scheduler.tasks[1].priority = 2;
        scheduler.ready_push(1);

        assert_eq!(scheduler.ready_pop(), 1);
        assert_eq!(scheduler.ready_pop(), 2);
    }

    #[test]
    fn cooperative_yield_hands_off_before_requeueing_higher_priority_task() {
        let mut scheduler = Sched::new();
        scheduler.current = 1;
        scheduler.tasks[1].state = State::Running;
        scheduler.tasks[1].priority = 2;
        ready_task(&mut scheduler, 2, 8);

        assert_eq!(scheduler.take_yield_target(1), Some(2));
        assert_eq!(scheduler.ready_pop(), 1);
    }

    #[test]
    fn preemptive_ready_queue_uses_priority_then_fifo() {
        let mut scheduler = Sched::new();
        ready_task(&mut scheduler, 1, 8);
        ready_task(&mut scheduler, 2, 4);
        ready_task(&mut scheduler, 3, 2);

        assert_eq!(scheduler.ready_pop(), 3);
        assert_eq!(scheduler.ready_pop(), 2);
        assert_eq!(scheduler.ready_pop(), 1);
    }

    #[test]
    fn all_run_policies_use_effective_priority_then_fifo() {
        let spec =
            BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
        let mut scheduler = Sched::new();
        scheduler.tasks[1].state = State::Ready;
        scheduler.tasks[1].priority = 20;
        scheduler.tasks[1].run_policy = RunPolicy::Cooperative;
        scheduler.ready_push(1);
        scheduler.tasks[2].state = State::Ready;
        scheduler.tasks[2].priority = 2;
        scheduler.tasks[2].run_policy = RunPolicy::Budgeted(spec);
        scheduler.ready_push(2);
        scheduler.tasks[3].state = State::Ready;
        scheduler.tasks[3].priority = 2;
        scheduler.tasks[3].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(1).unwrap(),
        };
        scheduler.ready_push(3);

        assert_eq!(scheduler.ready_pop(), 2);
        assert_eq!(scheduler.ready_pop(), 3);
        assert_eq!(scheduler.ready_pop(), 1);
    }

    #[test]
    fn policy_change_releases_a_throttled_task() {
        let spec =
            BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
        let mut scheduler = Sched::new();
        scheduler.tasks[2].state = State::Throttled;
        scheduler.tasks[2].run_policy = RunPolicy::Budgeted(spec);
        scheduler.tasks[2].budget = BudgetState::for_policy(RunPolicy::Budgeted(spec), 10);

        scheduler.set_run_policy(2, RunPolicy::Cooperative, 12);

        assert_eq!(scheduler.tasks[2].state, State::Ready);
        assert_eq!(scheduler.tasks[2].run_policy, RunPolicy::Cooperative);
        assert_eq!(scheduler.ready_pop(), 2);
    }

    #[test]
    fn exited_stacks_are_retired_for_later_reclamation() {
        let mut scheduler = Sched::new();
        scheduler.retire_stack(0x1000);
        scheduler.retire_stack(0x2000);

        assert_eq!(scheduler.retired_count, 2);
        assert_eq!(&scheduler.retired_stacks[..2], &[0x1000, 0x2000]);
    }

    #[test]
    fn scheduler_lock_is_nested_and_rejects_unbalanced_unlock() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.lock_current(10).unwrap();
        scheduler.lock_current(11).unwrap();
        assert_eq!(scheduler.tasks[0].scheduler_lock_depth, 2);
        scheduler.unlock_current().unwrap();
        scheduler.unlock_current().unwrap();
        assert_eq!(scheduler.unlock_current(), Err(DriverError::InvalidContext));
    }

    #[test]
    fn outermost_scheduler_unlock_releases_pending_higher_priority_task() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 10;
        scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(1).unwrap(),
        };
        ready_task(&mut scheduler, 1, 4);
        scheduler.lock_current(0).unwrap();
        scheduler.lock_current(0).unwrap();

        assert_eq!(
            scheduler.unlock_current_and_take_preemption(0).unwrap(),
            None
        );
        assert_eq!(
            scheduler.unlock_current_and_take_preemption(0).unwrap(),
            Some((0, 1))
        );
        assert!(matches!(scheduler.tasks[0].state, State::Ready));
    }

    #[test]
    fn irq_epilogue_preempts_only_after_outermost_interrupt_exit() {
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 10;
        scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(1).unwrap(),
        };
        ready_task(&mut scheduler, 1, 4);

        assert_eq!(scheduler.take_irq_epilogue_target(1), None);
        assert_eq!(scheduler.diagnostics.irq_preemptions, 0);
        assert_eq!(scheduler.take_irq_epilogue_target(0), Some((0, 1)));
        assert_eq!(scheduler.diagnostics.irq_preemptions, 1);
    }

    #[test]
    fn cooperative_task_is_not_preempted_by_irq_but_can_yield() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 10;
        ready_task(&mut scheduler, 1, 4);

        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        scheduler.started = true;
        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        assert_eq!(scheduler.take_yield_target(0), Some(1));
        assert_eq!(scheduler.diagnostics.irq_preemptions, 0);
    }

    #[test]
    fn expired_time_slice_round_robins_equal_priority_tasks() {
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 4;
        scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(5).unwrap(),
        };
        ready_task(&mut scheduler, 1, 4);

        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        scheduler.time_slice_pending = true;
        assert_eq!(scheduler.take_irq_epilogue_target(0), Some((0, 1)));
        assert_eq!(scheduler.diagnostics.time_slice_preemptions, 1);
        assert!(!scheduler.time_slice_pending);
    }

    #[test]
    fn scheduler_lock_preserves_expired_time_slice_until_unlock() {
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 4;
        scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(5).unwrap(),
        };
        ready_task(&mut scheduler, 1, 4);
        scheduler.time_slice_pending = true;
        scheduler.lock_current(100).unwrap();

        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        assert!(scheduler.time_slice_pending);
        assert_eq!(
            scheduler.unlock_current_and_take_preemption(0).unwrap(),
            Some((0, 1))
        );
        assert!(!scheduler.time_slice_pending);
    }

    #[test]
    fn budget_exhaustion_removes_task_until_replenishment() {
        let spec =
            BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.current = 0;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 2;
        scheduler.tasks[0].run_policy = RunPolicy::Budgeted(spec);
        scheduler.tasks[0].budget = BudgetState::for_policy(RunPolicy::Budgeted(spec), 100);
        scheduler.tasks[0].budget.on_dispatch(100);
        ready_task(&mut scheduler, 1, 20);

        assert_eq!(scheduler.on_timer(105, NonZeroU32::new(100).unwrap()), None);
        assert_eq!(scheduler.tasks[0].state, State::Throttled);
        assert_eq!(scheduler.diagnostics.budget_exhaustions, 1);
        assert_eq!(scheduler.take_irq_epilogue_target(0), Some((0, 1)));

        scheduler.replenish_budgets(119);
        assert_eq!(scheduler.tasks[0].state, State::Throttled);
        scheduler.replenish_budgets(120);
        assert_eq!(scheduler.tasks[0].state, State::Ready);
        assert_eq!(scheduler.diagnostics.budget_replenishments, 1);
    }

    #[test]
    fn scheduler_lock_defers_but_cannot_cancel_budget_throttle() {
        let spec =
            BudgetSpec::try_new(NonZeroU32::new(5).unwrap(), NonZeroU32::new(20).unwrap()).unwrap();
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 2;
        scheduler.tasks[0].run_policy = RunPolicy::Budgeted(spec);
        scheduler.tasks[0].budget = BudgetState::for_policy(RunPolicy::Budgeted(spec), 100);
        scheduler.tasks[0].budget.on_dispatch(100);
        ready_task(&mut scheduler, 1, 20);
        scheduler.lock_current(100).unwrap();

        assert_eq!(scheduler.on_timer(105, NonZeroU32::new(100).unwrap()), None);
        assert_eq!(scheduler.tasks[0].state, State::Running);
        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        assert_eq!(scheduler.diagnostics.budget_lock_overruns, 1);

        assert_eq!(
            scheduler.unlock_current_and_take_preemption(106).unwrap(),
            Some((0, 1))
        );
        assert_eq!(scheduler.tasks[0].state, State::Throttled);
        assert_eq!(scheduler.tasks[0].budget.replenishes_at(), 120);
    }

    #[test]
    fn scheduler_lock_limit_is_a_timer_deadline_and_fail_stop_violation() {
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.lock_current(100).unwrap();
        let limit = NonZeroU32::new(10).unwrap();

        assert_eq!(scheduler.scheduler_lock_deadline(limit), Some(110));
        assert_eq!(scheduler.on_timer(109, limit), None);
        assert_eq!(
            scheduler.on_timer(110, limit),
            Some(ContractViolation::SchedulerLockOverrun {
                task_slot: 0,
                held_ms: 10,
                limit_ms: 10,
            })
        );
        assert_eq!(scheduler.diagnostics.scheduler_lock_overruns, 1);
    }

    #[test]
    fn task_identity_generation_rejects_stale_slot_handle() {
        let stale = encode_task_id(3, 7).unwrap();
        assert_eq!(decode_task_id(stale), Ok((3, 7)));
        let replacement = encode_task_id(3, 8).unwrap();
        assert_ne!(stale, replacement);
        let last_slot = TASK_SLOT_COUNT - 1;
        assert_eq!(
            decode_task_id(encode_task_id(last_slot, 1).unwrap()),
            Ok((last_slot, 1))
        );
        assert_eq!(
            encode_task_id(TASK_SLOT_COUNT, 1),
            Err(DriverError::InvalidHandle)
        );
        assert_eq!(encode_task_id(0, 0), Err(DriverError::InvalidHandle));
    }

    #[test]
    fn earliest_deadline_ignores_forever_waiters() {
        let mut scheduler = Sched::new();
        scheduler.tasks[1].state = State::Blocked;
        scheduler.tasks[1].wake_at = 0;
        scheduler.tasks[2].state = State::Sleeping;
        scheduler.tasks[2].wake_at = 42;
        scheduler.tasks[3].state = State::Blocked;
        scheduler.tasks[3].wake_at = 17;

        assert_eq!(scheduler.earliest_wake_deadline(), Some(17));
    }

    #[test]
    fn shared_timer_uses_earliest_rtos_slice_or_embassy_deadline() {
        assert_eq!(
            earliest_deadline(Some(30), Some(20), Some(15), Some(12), Some(10)),
            Some(10)
        );
        assert_eq!(
            earliest_deadline(Some(30), Some(20), None, Some(18), None),
            Some(18)
        );
        assert_eq!(
            earliest_deadline(Some(30), None, Some(25), None, Some(40)),
            Some(25)
        );
        assert_eq!(
            earliest_deadline(None, None, None, None, Some(40)),
            Some(40)
        );
        assert_eq!(earliest_deadline(None, None, None, None, None), None);
    }

    #[test]
    fn stale_timer_programming_ticket_requires_retry() {
        let generation = Cell::new(0);
        let older = claim_timer_rearm_generation(&generation);
        let newer = claim_timer_rearm_generation(&generation);

        assert_ne!(older, newer);
        assert_ne!(generation.get(), older);
        assert_eq!(generation.get(), newer);

        let retry = claim_timer_rearm_generation(&generation);
        assert_eq!(generation.get(), retry);
    }

    #[test]
    fn unrelated_deadline_rearm_does_not_postpone_time_slice() {
        let mut scheduler = Sched::new();
        ready_task(&mut scheduler, 1, 4);
        scheduler.tasks[0].run_policy = RunPolicy::Preemptive {
            time_slice: NonZeroU32::new(5).unwrap(),
        };
        scheduler.tasks[0].priority = 4;

        assert_eq!(scheduler.next_time_slice_deadline(10), Some(15));
        assert_eq!(scheduler.next_time_slice_deadline(12), Some(15));

        scheduler.time_slice_deadline = 0;
        assert_eq!(scheduler.next_time_slice_deadline(15), Some(20));
        scheduler.ready_pop();
        assert_eq!(scheduler.next_time_slice_deadline(16), None);
    }

    #[test]
    fn forever_semaphore_wait_is_not_treated_as_an_expired_deadline() {
        let semaphore = Semaphore::new(0);
        let mut scheduler = Sched::new();
        scheduler.tasks[1].state = State::Blocked;
        scheduler.tasks[1].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
        scheduler.tasks[1].wake_at = 0;
        unsafe {
            (*semaphore.inner.get()).wait_head = 1;
            (*semaphore.inner.get()).wait_tail = 1;
        }

        scheduler.wake_sleepers(1_000);

        assert!(matches!(scheduler.tasks[1].state, State::Blocked));
        assert_eq!(unsafe { (*semaphore.inner.get()).wait_head }, 1);
        assert_eq!(scheduler.diagnostics.semaphore_timeouts, 0);
    }

    #[test]
    fn timed_semaphore_wait_wakes_only_after_its_deadline() {
        let semaphore = Semaphore::new(0);
        let mut scheduler = Sched::new();
        scheduler.tasks[1].state = State::Blocked;
        scheduler.tasks[1].waiting_sem = core::ptr::addr_of!(semaphore) as usize;
        scheduler.tasks[1].wake_at = 10;
        unsafe {
            (*semaphore.inner.get()).wait_head = 1;
            (*semaphore.inner.get()).wait_tail = 1;
        }

        scheduler.wake_sleepers(9);
        assert!(matches!(scheduler.tasks[1].state, State::Blocked));
        scheduler.wake_sleepers(10);

        assert!(matches!(scheduler.tasks[1].state, State::Ready));
        assert_eq!(scheduler.ready_pop(), 1);
        assert_eq!(scheduler.diagnostics.semaphore_timeouts, 1);
    }

    #[test]
    fn duplicate_mutex_waiters_keep_owner_inherited_until_both_leave() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].base_priority = 20;
        scheduler.tasks[0].priority = 20;

        scheduler.add_inheritance(0, 2);
        scheduler.add_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 2);
        scheduler.remove_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 2);
        scheduler.remove_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 20);
    }

    #[test]
    fn chained_mutex_inheritance_propagates_effective_priority() {
        let mut scheduler = Sched::new();
        let upstream = RtosMutex::new();
        let downstream = RtosMutex::new();

        scheduler.tasks[0].state = State::Blocked;
        scheduler.tasks[0].base_priority = 20;
        scheduler.tasks[0].priority = 20;
        scheduler.tasks[0].waiting_mutex = core::ptr::addr_of!(upstream) as usize;
        scheduler.tasks[1].state = State::Running;
        scheduler.tasks[1].base_priority = 30;
        scheduler.tasks[1].priority = 30;
        unsafe {
            (*upstream.inner.get()).owner = 1;
            (*upstream.inner.get()).wait_head = 0;
            (*upstream.inner.get()).wait_tail = 0;
            (*downstream.inner.get()).owner = 0;
        }
        scheduler.add_inheritance(1, 20);

        scheduler.add_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 2);
        assert_eq!(scheduler.tasks[1].priority, 2);

        scheduler.remove_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 20);
        assert_eq!(scheduler.tasks[1].priority, 20);
    }

    #[test]
    fn timed_out_mutex_waiter_restores_owner_priority() {
        let mut scheduler = Sched::new();
        let mutex = RtosMutex::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].base_priority = 20;
        scheduler.tasks[0].priority = 20;
        scheduler.tasks[1].state = State::Blocked;
        scheduler.tasks[1].base_priority = 2;
        scheduler.tasks[1].priority = 2;
        scheduler.tasks[1].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
        scheduler.tasks[1].wake_at = 10;
        unsafe {
            (*mutex.inner.get()).owner = 0;
            (*mutex.inner.get()).depth = 1;
            (*mutex.inner.get()).wait_head = 1;
            (*mutex.inner.get()).wait_tail = 1;
        }
        scheduler.add_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 2);

        scheduler.wake_sleepers(10);
        assert_eq!(scheduler.tasks[0].priority, 20);
        assert_eq!(scheduler.tasks[1].state, State::Ready);
        assert_eq!(scheduler.tasks[1].waiting_mutex, 0);
        assert_eq!(unsafe { (*mutex.inner.get()).wait_head }, NIL);
    }

    #[test]
    fn mutex_handoff_transfers_remaining_inheritance_to_new_owner() {
        let mut scheduler = Sched::new();
        let mutex = RtosMutex::new();

        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].base_priority = 20;
        scheduler.tasks[0].priority = 20;
        scheduler.tasks[1].state = State::Blocked;
        scheduler.tasks[1].base_priority = 2;
        scheduler.tasks[1].priority = 2;
        scheduler.tasks[1].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
        scheduler.tasks[2].state = State::Blocked;
        scheduler.tasks[2].base_priority = 5;
        scheduler.tasks[2].priority = 5;
        scheduler.tasks[2].waiting_mutex = core::ptr::addr_of!(mutex) as usize;
        unsafe {
            let state = &mut *mutex.inner.get();
            state.owner = 0;
            state.depth = 0;
            enqueue_mutex_waiter(&mut scheduler, state, 1);
            enqueue_mutex_waiter(&mut scheduler, state, 2);
        }
        scheduler.add_inheritance(0, 2);
        scheduler.add_inheritance(0, 5);

        unsafe { release_mutex_locked(&mut scheduler, &mut *mutex.inner.get(), 0) };

        let state = unsafe { &*mutex.inner.get() };
        assert_eq!(state.owner, 1);
        assert_eq!(state.wait_head, 2);
        assert_eq!(scheduler.tasks[0].priority, 20);
        assert_eq!(scheduler.tasks[1].priority, 2);
        assert_eq!(scheduler.tasks[1].inherited_waiters[5], 1);
        assert_eq!(scheduler.tasks[1].state, State::Ready);
        assert!(scheduler.tasks[1].sem_granted);
        assert_eq!(scheduler.tasks[1].waiting_mutex, 0);
    }

    #[test]
    fn base_priority_change_preserves_and_then_restores_inheritance() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].base_priority = 20;
        scheduler.tasks[0].priority = 20;

        scheduler.add_inheritance(0, 2);
        scheduler.tasks[0].base_priority = 10;
        scheduler.refresh_inherited_priority(0, 0);
        assert_eq!(scheduler.tasks[0].priority, 2);

        scheduler.remove_inheritance(0, 2);
        assert_eq!(scheduler.tasks[0].priority, 10);
    }
}
