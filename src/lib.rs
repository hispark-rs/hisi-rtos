//! Cooperative task scheduler and radio runtime backend.
//!
//! Modeled on esp-rtos's scheduler (TCB + context switch + ready queue +
//! blocking primitives), adapted for the WS63 app core (single-hart
//! `rv32imfc`). Key difference from esp32c3 (`rv32imc`): WS63 has the **F**
//! extension, so the context switch must also save/restore the callee-saved FP
//! registers `fs0..fs11` (the WiFi blob does floating-point RF math).
//!
//! This is a **cooperative** scheduler: a task runs until it calls
//! [`yield_now`], blocks on a [`Semaphore`], or [`sleep_ms`]s. That matches the
//! vendor WiFi worker-thread model (it waits on semaphores / sleeps). Preemptive
//! time-slicing (a timer ISR driving the switch) is a follow-on; the cooperative
//! core is what the blob's `osal_kthread_*` / `osal_sem_*` / `osal_wait_*` /
//! `osal_msleep` need.
//!
//! Layering: this crate depends only on `core`, `critical-section`, and the
//! runtime-neutral radio driver contract. The application injects allocation
//! and monotonic-time resources before any radio task can start.

#![no_std]

use core::cell::{Cell, RefCell, UnsafeCell};
use core::ffi::c_void;
use core::num::{NonZeroU32, NonZeroUsize};
use critical_section::Mutex;
use hisi_rf_rtos_driver::{
    Error as DriverError, Runtime, SemaphoreHandle, TaskConfig, TaskId, WaitOutcome, WaitTimeout,
};

/// Max concurrent tasks (slot table; the WiFi stack uses only a few).
const MAX_TASKS: usize = 16;
/// LiteOS-compatible priority levels: 0 is highest, 31 is lowest.
const PRIORITY_LEVELS: usize = 32;
/// Sentinel "no task" index for intrusive list links.
const NIL: usize = usize::MAX;

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

/// Scheduler configuration fixed before the first task starts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    /// Minimum stack allocation applied to every task request.
    pub minimum_stack_size: NonZeroUsize,
    /// Ready-queue selection policy.
    pub scheduling: SchedulingPolicy,
}

/// Scheduling policy selected before the runtime starts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SchedulingPolicy {
    /// Explicit yield/block points use creation-order FIFO scheduling.
    Cooperative,
    /// Explicit scheduling points select the lowest numeric task priority.
    /// This does not by itself enable timer-driven preemption.
    Priority,
}

/// Read-only scheduler counters and task-state census for bring-up diagnostics.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct Diagnostics {
    pub context_switches: u32,
    pub yields: u32,
    pub sleeps: u32,
    pub sleeper_wakes: u32,
    pub semaphore_blocks: u32,
    pub semaphore_wakes: u32,
    pub semaphore_timeouts: u32,
    pub scheduler_locks: u32,
    pub current_task: usize,
    pub ready_tasks: u8,
    pub blocked_tasks: u8,
    pub sleeping_tasks: u8,
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
}

/// Read-only scheduler slot details for diagnosing blocked radio workers.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub struct TaskDiagnostic {
    pub task: usize,
    pub state: TaskState,
    pub entry: usize,
    pub waiting_sem: usize,
    pub wake_at: u64,
    pub priority: u8,
    pub scheduler_lock_depth: u16,
}

impl Diagnostics {
    const EMPTY: Self = Self {
        context_switches: 0,
        yields: 0,
        sleeps: 0,
        sleeper_wakes: 0,
        semaphore_blocks: 0,
        semaphore_wakes: 0,
        semaphore_timeouts: 0,
        scheduler_locks: 0,
        current_task: 0,
        ready_tasks: 0,
        blocked_tasks: 0,
        sleeping_tasks: 0,
        current_lock_depth: 0,
    };
}

impl Default for Config {
    fn default() -> Self {
        Self {
            // The WS63 vendor archive was built with a 24 KiB LiteOS default.
            minimum_stack_size: NonZeroUsize::new(24 * 1024).unwrap(),
            scheduling: SchedulingPolicy::Cooperative,
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
    config: Config,
    resources: Resources,
}

static START_STATE: Mutex<Cell<Option<StartState>>> = Mutex::new(Cell::new(None));

fn start_state() -> StartState {
    critical_section::with(|cs| START_STATE.borrow(cs).get())
        .expect("hisi-rtos must be started before radio runtime use")
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

// ── Saved CPU context (offsets MUST match `context_switch` asm) ──────────────
#[repr(C)]
#[derive(Clone, Copy)]
struct Ctx {
    ra: usize,      // 0
    sp: usize,      // 4
    s: [usize; 12], // 8..56  (s0..s11)
    fs: [u32; 12],  // 56..104 (fs0..fs11, FLEN=32)
    tp: usize,      // 104
    mstatus: u32,   // 108
    fcsr: u32,      // 112
}
impl Ctx {
    const fn zero() -> Self {
        Ctx {
            ra: 0,
            sp: 0,
            s: [0; 12],
            fs: [0; 12],
            tp: 0,
            mstatus: 0,
            fcsr: 0,
        }
    }
}

/// Cooperative context switch: save callee-saved regs of the current task to
/// `*old`, restore `*new`, return into the new task. Caller-saved regs are
/// spilled by the compiler around this normal call, so only callee-saved
/// (ra, sp, s0-s11, fs0-fs11) need saving.
#[cfg(target_arch = "riscv32")]
#[unsafe(naked)]
unsafe extern "C" fn context_switch(old: *mut Ctx, new: *const Ctx) {
    core::arch::naked_asm!(
        // Enable the F extension for the fsw/flw below (rv32imfc has it, but the
        // inline-asm assembler context defaults to a baseline without F).
        ".option arch, +f",
        // save current -> *old (a0)
        "sw  ra,  0(a0)",
        "sw  sp,  4(a0)",
        "sw  s0,  8(a0)",
        "sw  s1, 12(a0)",
        "sw  s2, 16(a0)",
        "sw  s3, 20(a0)",
        "sw  s4, 24(a0)",
        "sw  s5, 28(a0)",
        "sw  s6, 32(a0)",
        "sw  s7, 36(a0)",
        "sw  s8, 40(a0)",
        "sw  s9, 44(a0)",
        "sw  s10,48(a0)",
        "sw  s11,52(a0)",
        "fsw fs0, 56(a0)",
        "fsw fs1, 60(a0)",
        "fsw fs2, 64(a0)",
        "fsw fs3, 68(a0)",
        "fsw fs4, 72(a0)",
        "fsw fs5, 76(a0)",
        "fsw fs6, 80(a0)",
        "fsw fs7, 84(a0)",
        "fsw fs8, 88(a0)",
        "fsw fs9, 92(a0)",
        "fsw fs10,96(a0)",
        "fsw fs11,100(a0)",
        "sw  tp, 104(a0)",
        "csrr t0, mstatus",
        "sw  t0, 108(a0)",
        "frcsr t0",
        "sw  t0, 112(a0)",
        // restore *new (a1) -> current
        "lw  ra,  0(a1)",
        "lw  sp,  4(a1)",
        "lw  s0,  8(a1)",
        "lw  s1, 12(a1)",
        "lw  s2, 16(a1)",
        "lw  s3, 20(a1)",
        "lw  s4, 24(a1)",
        "lw  s5, 28(a1)",
        "lw  s6, 32(a1)",
        "lw  s7, 36(a1)",
        "lw  s8, 40(a1)",
        "lw  s9, 44(a1)",
        "lw  s10,48(a1)",
        "lw  s11,52(a1)",
        "flw fs0, 56(a1)",
        "flw fs1, 60(a1)",
        "flw fs2, 64(a1)",
        "flw fs3, 68(a1)",
        "flw fs4, 72(a1)",
        "flw fs5, 76(a1)",
        "flw fs6, 80(a1)",
        "flw fs7, 84(a1)",
        "flw fs8, 88(a1)",
        "flw fs9, 92(a1)",
        "flw fs10,96(a1)",
        "flw fs11,100(a1)",
        "lw  tp, 104(a1)",
        "lw  t0, 112(a1)",
        "fscsr t0",
        // Restore mstatus last: setting MIE before the remaining context is
        // live would allow an interrupt to observe a half-restored task.
        "lw  t0, 108(a1)",
        "csrw mstatus, t0",
        "ret",
    )
}

#[cfg(not(target_arch = "riscv32"))]
unsafe extern "C" fn context_switch(_old: *mut Ctx, _new: *const Ctx) {
    unreachable!("WS63 context switching is only available on riscv32");
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum State {
    Free,
    Ready,
    Running,
    Blocked,
    Sleeping,
}

/// Task entry signature (matches the OSAL `osal_kthread_func`).
type TaskFn = extern "C" fn(*mut c_void) -> *mut c_void;

struct Tcb {
    ctx: Ctx,
    state: State,
    stack: usize, // heap allocation addr to free on exit (0 for the main task)
    entry: Option<TaskFn>,
    arg: usize,   // task argument (*mut c_void stored as usize so Tcb is Send)
    next: usize,  // intrusive link: ready queue OR one wait queue
    wake_at: u64, // mask-ROM systick millisecond deadline
    waiting_sem: usize,
    sem_granted: bool,
    priority: u8,
    scheduler_lock_depth: u16,
}
impl Tcb {
    const fn empty() -> Self {
        Tcb {
            ctx: Ctx::zero(),
            state: State::Free,
            stack: 0,
            entry: None,
            arg: 0,
            next: NIL,
            wake_at: 0,
            waiting_sem: 0,
            sem_granted: false,
            priority: (PRIORITY_LEVELS - 1) as u8,
            scheduler_lock_depth: 0,
        }
    }
}

struct Sched {
    tasks: [Tcb; MAX_TASKS],
    current: usize,
    ready_head: [usize; PRIORITY_LEVELS],
    ready_tail: [usize; PRIORITY_LEVELS],
    retired_stacks: [usize; MAX_TASKS],
    retired_count: usize,
    priority_scheduling: bool,
    started: bool,
    diagnostics: Diagnostics,
}
impl Sched {
    const fn new() -> Self {
        const E: Tcb = Tcb::empty();
        Sched {
            tasks: [E; MAX_TASKS],
            current: 0,
            ready_head: [NIL; PRIORITY_LEVELS],
            ready_tail: [NIL; PRIORITY_LEVELS],
            retired_stacks: [0; MAX_TASKS],
            retired_count: 0,
            priority_scheduling: false,
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
                State::Free | State::Running => {}
            }
        }
        snapshot
    }

    fn task_diagnostics(&self, output: &mut [TaskDiagnostic]) -> usize {
        let count = output.len().min(MAX_TASKS);
        for (index, output) in output[..count].iter_mut().enumerate() {
            let task = &self.tasks[index];
            *output = TaskDiagnostic {
                task: index,
                state: match task.state {
                    State::Free => TaskState::Free,
                    State::Ready => TaskState::Ready,
                    State::Running => TaskState::Running,
                    State::Blocked => TaskState::Blocked,
                    State::Sleeping => TaskState::Sleeping,
                },
                entry: task.entry.map_or(0, |entry| entry as usize),
                waiting_sem: task.waiting_sem,
                wake_at: task.wake_at,
                priority: task.priority,
                scheduler_lock_depth: task.scheduler_lock_depth,
            };
        }
        count
    }
    fn ready_push(&mut self, i: usize) {
        let priority = if self.priority_scheduling {
            self.tasks[i].priority as usize
        } else {
            0
        };
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
    fn ready_remove(&mut self, task: usize) {
        let priority = if self.priority_scheduling {
            self.tasks[task].priority as usize
        } else {
            0
        };
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
    fn take_yield_target(&mut self, current: usize) -> Option<usize> {
        let next = self.ready_pop();
        if next == NIL {
            return None;
        }
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some(next)
    }
    fn take_preemption_target(&mut self) -> Option<(usize, usize)> {
        if !self.priority_scheduling {
            return None;
        }
        let current = self.current;
        if self.tasks[current].state != State::Running
            || self.tasks[current].scheduler_lock_depth != 0
        {
            return None;
        }
        let current_priority = self.tasks[current].priority as usize;
        if !(0..current_priority).any(|priority| self.ready_head[priority] != NIL) {
            return None;
        }
        let next = self.ready_pop();
        debug_assert!(next != NIL);
        self.tasks[current].state = State::Ready;
        self.ready_push(current);
        Some((current, next))
    }
    fn retire_stack(&mut self, stack: usize) {
        if stack != 0 {
            debug_assert!(self.retired_count < MAX_TASKS);
            self.retired_stacks[self.retired_count] = stack;
            self.retired_count += 1;
        }
    }
    fn lock_current(&mut self) -> Result<(), DriverError> {
        let task = &mut self.tasks[self.current];
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
        Ok(())
    }

    fn unlock_current_and_take_preemption(
        &mut self,
    ) -> Result<Option<(usize, usize)>, DriverError> {
        self.unlock_current()?;
        Ok(self.take_preemption_target())
    }
    fn wake_sleepers(&mut self, now: u64) {
        for i in 0..MAX_TASKS {
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
            }
        }
    }
    fn alloc_slot(&mut self) -> Option<usize> {
        (0..MAX_TASKS).find(|&i| self.tasks[i].state == State::Free)
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

fn reclaim_retired_stacks() {
    let (stacks, count) = critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        let stacks = scheduler.retired_stacks;
        let count = scheduler.retired_count;
        scheduler.retired_stacks = [0; MAX_TASKS];
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

/// First-run trampoline: a freshly switched-to task lands here (its `ctx.ra`),
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

/// Initialize the scheduler, adopting the current execution as the main task
/// (slot 0). Idempotent.
fn init() {
    let priority_scheduling = matches!(start_state().config.scheduling, SchedulingPolicy::Priority);
    critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        if s.started {
            return;
        }
        s.tasks[0].state = State::Running;
        s.current = 0;
        s.priority_scheduling = priority_scheduling;
        s.started = true;
    });
}

/// Spawn a task. Returns its slot index, or `None` if the table/stack is full.
fn spawn(entry: TaskFn, arg: *mut c_void, stack_size: usize, priority: u8) -> Option<usize> {
    init();
    reclaim_retired_stacks();
    let size = stack_size.max(start_state().config.minimum_stack_size.get());
    let stack = allocate(size);
    if stack.is_null() {
        return None;
    }
    // 16-byte aligned stack top.
    let top = (stack as usize + size) & !0xf;
    let slot = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let i = s.alloc_slot()?;
        let t = &mut s.tasks[i];
        t.ctx = Ctx::zero();
        #[cfg(target_arch = "riscv32")]
        unsafe {
            core::arch::asm!(
                "mv {tp}, tp",
                "csrr {mstatus}, mstatus",
                "frcsr {fcsr}",
                tp = out(reg) t.ctx.tp,
                mstatus = out(reg) t.ctx.mstatus,
                fcsr = out(reg) t.ctx.fcsr,
                options(nomem, nostack),
            );
        }
        // Cast through a fn pointer (not a direct fn-item->int cast).
        let tramp: extern "C" fn() -> ! = trampoline;
        t.ctx.ra = tramp as usize;
        t.ctx.sp = top;
        t.state = State::Ready;
        t.stack = stack as usize;
        t.entry = Some(entry);
        t.arg = arg as usize;
        t.wake_at = 0;
        t.priority = priority;
        s.ready_push(i);
        Some(i)
    });
    if slot.is_none() {
        deallocate(stack);
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
        return;
    }
    let (op, np) = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.diagnostics.context_switches = s.diagnostics.context_switches.saturating_add(1);
        s.tasks[next].state = State::Running;
        s.current = next;
        (
            core::ptr::addr_of_mut!(s.tasks[prev].ctx),
            core::ptr::addr_of!(s.tasks[next].ctx),
        )
    });
    // SAFETY: contexts live in the static SCHED (stable address); single-hart,
    // and the lock is released so the resumed task can re-enter the scheduler.
    unsafe { context_switch(op, np) };
}

fn switch_away(prev: usize) {
    loop {
        let now = now_ms();
        let next = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            s.wake_sleepers(now);
            s.ready_pop()
        });
        if next == NIL {
            core::hint::spin_loop();
            continue;
        }
        switch_to(prev, next);
        return;
    }
}

/// Yield the CPU: requeue the current task and run the next ready one.
fn yield_now() {
    let now = now_ms();
    let target = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.diagnostics.yields = s.diagnostics.yields.saturating_add(1);
        s.wake_sleepers(now);
        let cur = s.current;
        // A cooperative yield promises progress to another ready task. Select
        // that task before requeueing the current one; otherwise a strict
        // priority queue would immediately select the yielding high-priority
        // task again and starve lower-priority initialization work.
        s.take_yield_target(cur).map(|next| (cur, next))
    });
    if let Some((prev, next)) = target {
        switch_to(prev, next);
    }
    reclaim_retired_stacks();
}

/// Sleep the current task for `ms` milliseconds (cooperative; wakes when a later
/// schedule sees the deadline pass).
fn sleep_ms(ms: u32) {
    if ms == 0 {
        yield_now();
        return;
    }
    let wake_at = now_ms().saturating_add(ms as u64);
    let prev = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        s.diagnostics.sleeps = s.diagnostics.sleeps.saturating_add(1);
        let cur = s.current;
        s.tasks[cur].state = State::Sleeping;
        s.tasks[cur].wake_at = wake_at;
        cur
    });
    switch_away(prev);
    reclaim_retired_stacks();
}

/// Current task slot index (its "pid"/"tid").
fn current_id() -> usize {
    critical_section::with(|cs| SCHED.borrow_ref(cs).current)
}

fn task_exit() -> ! {
    // Retire the stack before switching away. A resumed task drains the retired
    // list only after it is running on a different stack.
    let prev = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let cur = s.current;
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
    fn down(&self) {
        let block = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section (single hart).
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                false
            } else {
                let cur = s.current;
                s.diagnostics.semaphore_blocks = s.diagnostics.semaphore_blocks.saturating_add(1);
                s.tasks[cur].state = State::Blocked;
                s.tasks[cur].wake_at = 0;
                s.tasks[cur].waiting_sem = self as *const Self as usize;
                s.tasks[cur].sem_granted = false;
                enqueue_waiter(s, st, cur);
                true
            }
        });
        if block {
            // Parked on this sem's wait queue; `up` will move us back to Ready
            // (== the grant). When we resume here, we already hold the count.
            switch_away(current_id());
            critical_section::with(|cs| {
                let s = &mut *SCHED.borrow_ref_mut(cs);
                s.tasks[s.current].sem_granted = false;
            });
        }
    }

    /// Acquire with a timeout (ms). Returns `true` if a count was obtained,
    /// `false` if the deadline passed first. `u32::MAX` (wait-forever) blocks
    /// like [`down`](Semaphore::down).
    ///
    /// The waiter is linked into the semaphore queue so [`up`](Self::up) can
    /// hand the grant directly to it. The scheduler removes it from that queue
    /// if the mask-ROM systick deadline wins first.
    fn down_timeout(&self, timeout_ms: u32) -> bool {
        if timeout_ms == u32::MAX {
            self.down();
            return true;
        }
        let deadline = now_ms().saturating_add(timeout_ms as u64);
        let current = critical_section::with(|cs| {
            let s = &mut *SCHED.borrow_ref_mut(cs);
            // SAFETY: exclusive under the critical section.
            let st = unsafe { &mut *self.inner.get() };
            if st.count > 0 {
                st.count -= 1;
                return None;
            }
            if timeout_ms == 0 {
                return Some(NIL);
            }
            let cur = s.current;
            s.diagnostics.semaphore_blocks = s.diagnostics.semaphore_blocks.saturating_add(1);
            s.tasks[cur].state = State::Blocked;
            s.tasks[cur].wake_at = deadline;
            s.tasks[cur].waiting_sem = self as *const Self as usize;
            s.tasks[cur].sem_granted = false;
            enqueue_waiter(s, st, cur);
            Some(cur)
        });
        match current {
            None => true,
            Some(NIL) => false,
            Some(current) => {
                switch_away(current);
                critical_section::with(|cs| {
                    let s = &mut *SCHED.borrow_ref_mut(cs);
                    let granted = s.tasks[s.current].sem_granted;
                    s.tasks[s.current].sem_granted = false;
                    granted
                })
            }
        }
    }

    /// Release (V). Wakes one waiter if any, else increments the count.
    fn up(&self) {
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
            if INTERRUPT_DEPTH.borrow(cs).get() == 0 {
                s.take_preemption_target()
            } else {
                None
            }
        });
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
    }
}

struct HisiRuntime;

static RUNTIME: HisiRuntime = HisiRuntime;

/// Starts the scheduler and installs it as the firmware's sole radio runtime.
pub fn start(config: Config, resources: Resources) -> Result<(), StartError> {
    let already_started = critical_section::with(|cs| {
        let state = START_STATE.borrow(cs);
        if state.get().is_some() {
            true
        } else {
            state.set(Some(StartState { config, resources }));
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

fn semaphore_from_handle(handle: SemaphoreHandle) -> &'static Semaphore {
    let pointer = handle.into_raw().get() as *const Semaphore;
    // SAFETY: this backend creates handles only from heap-allocated Semaphore
    // objects and the driver contract requires users to stop all access before
    // destroy.
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
        let slot = spawn(entry, arg, config.stack_size.get(), config.priority)
            .ok_or(DriverError::ResourceExhausted)?;
        let raw = u32::try_from(slot).map_err(|_| DriverError::Runtime)?;
        Ok(TaskId::from_raw(raw))
    }

    fn yield_now(&self) -> Result<(), DriverError> {
        yield_now();
        Ok(())
    }

    fn sleep_ms(&self, milliseconds: NonZeroU32) -> Result<(), DriverError> {
        sleep_ms(milliseconds.get());
        Ok(())
    }

    fn current_task(&self) -> Result<TaskId, DriverError> {
        let raw = u32::try_from(current_id()).map_err(|_| DriverError::Runtime)?;
        Ok(TaskId::from_raw(raw))
    }

    fn set_task_priority(&self, task: TaskId, priority: u8) -> Result<(), DriverError> {
        if priority as usize >= PRIORITY_LEVELS {
            return Err(DriverError::Runtime);
        }
        let slot = usize::try_from(task.into_raw()).map_err(|_| DriverError::InvalidHandle)?;
        critical_section::with(|cs| {
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let Some(tcb) = scheduler.tasks.get(slot) else {
                return Err(DriverError::InvalidHandle);
            };
            if tcb.state == State::Free {
                return Err(DriverError::InvalidHandle);
            }
            if tcb.state == State::Ready {
                scheduler.ready_remove(slot);
                scheduler.tasks[slot].priority = priority;
                scheduler.ready_push(slot);
            } else {
                scheduler.tasks[slot].priority = priority;
            }
            Ok(())
        })
    }

    fn lock_scheduler(&self) -> Result<(), DriverError> {
        critical_section::with(|cs| SCHED.borrow_ref_mut(cs).lock_current())
    }

    fn unlock_scheduler(&self) -> Result<(), DriverError> {
        let preemption = critical_section::with(|cs| {
            SCHED
                .borrow_ref_mut(cs)
                .unlock_current_and_take_preemption()
        })?;
        if let Some((current, next)) = preemption {
            switch_to(current, next);
        }
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
            if semaphore_from_handle(semaphore).down_timeout(timeout_ms) {
                WaitOutcome::Acquired
            } else {
                WaitOutcome::TimedOut
            },
        )
    }

    fn semaphore_up(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        semaphore_from_handle(semaphore).up();
        Ok(())
    }

    unsafe fn semaphore_destroy(&self, semaphore: SemaphoreHandle) -> Result<(), DriverError> {
        deallocate(semaphore.into_raw().get() as *mut u8);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ready_task(scheduler: &mut Sched, slot: usize, priority: u8) {
        scheduler.tasks[slot].state = State::Ready;
        scheduler.tasks[slot].priority = priority;
        scheduler.ready_push(slot);
    }

    #[test]
    fn ready_queue_prefers_lower_priority_number_and_keeps_fifo() {
        let mut scheduler = Sched::new();
        scheduler.priority_scheduling = true;
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
        scheduler.priority_scheduling = true;
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
    fn cooperative_policy_keeps_fifo_across_task_priorities() {
        let mut scheduler = Sched::new();
        ready_task(&mut scheduler, 1, 8);
        ready_task(&mut scheduler, 2, 4);
        ready_task(&mut scheduler, 3, 2);

        assert_eq!(scheduler.ready_pop(), 1);
        assert_eq!(scheduler.ready_pop(), 2);
        assert_eq!(scheduler.ready_pop(), 3);
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
        scheduler.lock_current().unwrap();
        scheduler.lock_current().unwrap();
        assert_eq!(scheduler.tasks[0].scheduler_lock_depth, 2);
        scheduler.unlock_current().unwrap();
        scheduler.unlock_current().unwrap();
        assert_eq!(scheduler.unlock_current(), Err(DriverError::InvalidContext));
    }

    #[test]
    fn outermost_scheduler_unlock_releases_pending_higher_priority_task() {
        let mut scheduler = Sched::new();
        scheduler.priority_scheduling = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 10;
        ready_task(&mut scheduler, 1, 4);
        scheduler.lock_current().unwrap();
        scheduler.lock_current().unwrap();

        assert_eq!(
            scheduler.unlock_current_and_take_preemption().unwrap(),
            None
        );
        assert_eq!(
            scheduler.unlock_current_and_take_preemption().unwrap(),
            Some((0, 1))
        );
        assert!(matches!(scheduler.tasks[0].state, State::Ready));
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
}
