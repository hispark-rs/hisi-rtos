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

#![no_std]

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
    Error as DriverError, MutexHandle, Runtime, SemaphoreHandle, TaskConfig, TaskId, WaitOutcome,
    WaitTimeout,
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
}

/// Scheduler configuration fixed before the first task starts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Config {
    /// Minimum stack allocation applied to every task request.
    pub minimum_stack_size: NonZeroUsize,
    /// Ready-queue selection policy.
    pub scheduling: SchedulingPolicy,
    /// Optional round-robin time slice for equal-priority ready tasks.
    ///
    /// Requires [`start_with_port`]; ignored by the compatibility [`start`]
    /// entry point when no timer port is installed.
    pub time_slice: Option<NonZeroU32>,
}

/// Scheduling policy selected before the runtime starts.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum SchedulingPolicy {
    /// Explicit yield/block points use creation-order FIFO scheduling.
    Cooperative,
    /// Explicit scheduling points select the lowest numeric task priority.
    /// Timer-driven preemption additionally requires a configured time slice
    /// and [`start_with_port`].
    Priority,
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
    pub waiting_mutex: usize,
    pub wake_at: u64,
    pub base_priority: u8,
    /// Effective priority after inheritance (lower value is higher priority).
    pub priority: u8,
    pub scheduler_lock_depth: u16,
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
            time_slice: None,
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
    port: Option<SchedulerPort>,
}

static START_STATE: Mutex<Cell<Option<StartState>>> = Mutex::new(Cell::new(None));

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

// Unified 272-byte task context. The field order matches the WS63 LiteOS
// `TaskContext` ABI and the trap frames emitted by hisi-riscv-rt.
#[repr(C, align(16))]
#[derive(Clone, Copy)]
struct TaskContext {
    mstatus: u32,          // 0
    mepc: u32,             // 4
    tp: u32,               // 8
    sp: u32,               // 12
    s: [u32; 12],          // 16..64: s11..s0
    caller_gpr: [u32; 16], // 64..128: t6..t3,a7..a0,t2..t0,ra
    f: [u32; 32],          // 128..256: fs11..fs0,ft11..ft0
    fcsr: u32,             // 256
    reserved: [u32; 3],    // 260..272
}
impl TaskContext {
    const fn zero() -> Self {
        TaskContext {
            mstatus: 0,
            mepc: 0,
            tp: 0,
            sp: 0,
            s: [0; 12],
            caller_gpr: [0; 16],
            f: [0; 32],
            fcsr: 0,
            reserved: [0; 3],
        }
    }
}

const _: () = {
    assert!(core::mem::size_of::<TaskContext>() == 272);
    assert!(core::mem::offset_of!(TaskContext, mstatus) == 0);
    assert!(core::mem::offset_of!(TaskContext, mepc) == 4);
    assert!(core::mem::offset_of!(TaskContext, s) == 16);
    assert!(core::mem::offset_of!(TaskContext, caller_gpr) == 64);
    assert!(core::mem::offset_of!(TaskContext, f) == 128);
    assert!(core::mem::offset_of!(TaskContext, fcsr) == 256);
};

#[cfg(target_arch = "riscv32")]
const TASK_CONTEXT_WORDS: usize = 68;

#[cfg(target_arch = "riscv32")]
unsafe fn initialize_irq_frame(top: usize, entry: usize, tp: usize, fcsr: u32) -> usize {
    let frame = (top - TASK_CONTEXT_WORDS * core::mem::size_of::<u32>()) as *mut u32;
    // SAFETY: the caller owns the allocated task stack through `Tcb::stack`,
    // and `top` leaves at least the configured minimum stack size below it.
    unsafe {
        frame.write_bytes(0, TASK_CONTEXT_WORDS);
        frame.add(0).write(0x7880);
        frame.add(1).write(entry as u32);
        frame.add(2).write(tp as u32);
        frame.add(3).write(top as u32);
        frame.add(64).write(fcsr);
    }
    frame as usize
}

/// Cooperative context switch: save callee-saved regs of the current task to
/// `*old`, restore `*new`, return into the new task. Caller-saved regs are
/// spilled by the compiler around this normal call, so only callee-saved
/// (ra, sp, s0-s11, fs0-fs11) need saving.
#[cfg(target_arch = "riscv32")]
#[unsafe(naked)]
unsafe extern "C" fn context_switch(old: *mut TaskContext, new: *const TaskContext) {
    core::arch::naked_asm!(
        ".option arch, +f",
        // Disable MIE and encode its prior value as MPIE for the eventual mret.
        "li t1, 8",
        "csrrc t0, mstatus, t1",
        "andi t2, t0, 8",
        "slli t2, t2, 4",
        "li t1, -137",
        "and t0, t0, t1",
        "or t0, t0, t2",
        "li t1, 0x1800",
        "or t0, t0, t1",
        "sw t0, 0(a0)",
        "sw ra, 4(a0)",
        "sw tp, 8(a0)",
        "sw sp, 12(a0)",
        "sw s11, 16(a0)",
        "sw s10, 20(a0)",
        "sw s9, 24(a0)",
        "sw s8, 28(a0)",
        "sw s7, 32(a0)",
        "sw s6, 36(a0)",
        "sw s5, 40(a0)",
        "sw s4, 44(a0)",
        "sw s3, 48(a0)",
        "sw s2, 52(a0)",
        "sw s1, 56(a0)",
        "sw s0, 60(a0)",
        "fsw fs11, 128(a0)",
        "fsw fs10, 132(a0)",
        "fsw fs9, 136(a0)",
        "fsw fs8, 140(a0)",
        "fsw fs7, 144(a0)",
        "fsw fs6, 148(a0)",
        "fsw fs5, 152(a0)",
        "fsw fs4, 156(a0)",
        "fsw fs3, 160(a0)",
        "fsw fs2, 164(a0)",
        "fsw fs1, 168(a0)",
        "fsw fs0, 172(a0)",
        "frcsr t0",
        "sw t0, 256(a0)",
        // Restore the same complete ABI used by interrupt and fresh frames.
        "mv t0, a1",
        "lw t1, 0(t0)",
        "csrw mstatus, t1",
        "lw t1, 4(t0)",
        "csrw mepc, t1",
        "lw t1, 256(t0)",
        "fscsr t1",
        "flw fs11, 128(t0)",
        "flw fs10, 132(t0)",
        "flw fs9, 136(t0)",
        "flw fs8, 140(t0)",
        "flw fs7, 144(t0)",
        "flw fs6, 148(t0)",
        "flw fs5, 152(t0)",
        "flw fs4, 156(t0)",
        "flw fs3, 160(t0)",
        "flw fs2, 164(t0)",
        "flw fs1, 168(t0)",
        "flw fs0, 172(t0)",
        "flw ft11, 176(t0)",
        "flw ft10, 180(t0)",
        "flw ft9, 184(t0)",
        "flw ft8, 188(t0)",
        "flw fa7, 192(t0)",
        "flw fa6, 196(t0)",
        "flw fa5, 200(t0)",
        "flw fa4, 204(t0)",
        "flw fa3, 208(t0)",
        "flw fa2, 212(t0)",
        "flw fa1, 216(t0)",
        "flw fa0, 220(t0)",
        "flw ft7, 224(t0)",
        "flw ft6, 228(t0)",
        "flw ft5, 232(t0)",
        "flw ft4, 236(t0)",
        "flw ft3, 240(t0)",
        "flw ft2, 244(t0)",
        "flw ft1, 248(t0)",
        "flw ft0, 252(t0)",
        "lw tp, 8(t0)",
        "lw s11, 16(t0)",
        "lw s10, 20(t0)",
        "lw s9, 24(t0)",
        "lw s8, 28(t0)",
        "lw s7, 32(t0)",
        "lw s6, 36(t0)",
        "lw s5, 40(t0)",
        "lw s4, 44(t0)",
        "lw s3, 48(t0)",
        "lw s2, 52(t0)",
        "lw s1, 56(t0)",
        "lw s0, 60(t0)",
        "lw t6, 64(t0)",
        "lw t5, 68(t0)",
        "lw t4, 72(t0)",
        "lw t3, 76(t0)",
        "lw a7, 80(t0)",
        "lw a6, 84(t0)",
        "lw a5, 88(t0)",
        "lw a4, 92(t0)",
        "lw a3, 96(t0)",
        "lw a2, 100(t0)",
        "lw a1, 104(t0)",
        "lw a0, 108(t0)",
        "lw t2, 112(t0)",
        "lw t1, 116(t0)",
        "lw ra, 124(t0)",
        "lw sp, 12(t0)",
        "lw t0, 120(t0)",
        "mret",
    )
}

#[cfg(not(target_arch = "riscv32"))]
unsafe extern "C" fn context_switch(_old: *mut TaskContext, _new: *const TaskContext) {
    unreachable!("WS63 context switching is only available on riscv32");
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
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
            tasks: [E; MAX_TASKS],
            current: 0,
            ready_head: [NIL; PRIORITY_LEVELS],
            ready_tail: [NIL; PRIORITY_LEVELS],
            retired_stacks: [0; MAX_TASKS],
            retired_count: 0,
            priority_scheduling: false,
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
                waiting_mutex: task.waiting_mutex,
                wake_at: task.wake_at,
                base_priority: task.base_priority,
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
        assert!(depth < MAX_TASKS, "mutex inheritance cycle");
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
        let target = self.take_reschedule_target(time_slice);
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
    fn schedule_from_trap(&mut self, frame: usize, interrupt_depth: u16) -> usize {
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
            let next = self.ready_pop();
            (next != NIL).then_some((current, next))
        } else {
            self.take_reschedule_target(time_slice)
        };

        let Some((previous, next)) = target else {
            return frame;
        };

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
        let target = self.take_reschedule_target(self.time_slice_pending);
        if target.is_some() {
            self.time_slice_pending = false;
        }
        Ok(target)
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
    }
    fn alloc_slot(&mut self) -> Option<usize> {
        (0..MAX_TASKS).find(|&i| self.tasks[i].state == State::Free)
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

    fn next_time_slice_deadline(&mut self, now: u64, slice: Option<NonZeroU32>) -> Option<u64> {
        if slice.is_none() || !self.has_ready_task() {
            self.time_slice_deadline = 0;
            return None;
        }
        if self.time_slice_deadline == 0 {
            self.time_slice_deadline = now.saturating_add(slice.unwrap().get() as u64);
        }
        Some(self.time_slice_deadline)
    }

    fn has_ready_task(&self) -> bool {
        self.ready_head.iter().any(|head| *head != NIL)
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
    critical_section::with(|cs| {
        let depth = INTERRUPT_DEPTH.borrow(cs).get();
        SCHED
            .borrow_ref_mut(cs)
            .schedule_from_trap(encoded_frame, depth)
    })
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

fn earliest_deadline(
    wake_deadline: Option<u64>,
    slice_deadline: Option<u64>,
    embassy_deadline: Option<u64>,
) -> Option<u64> {
    wake_deadline
        .into_iter()
        .chain(slice_deadline)
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
fn embassy_next_expiration(now_ms: u64) -> Option<u64> {
    let now_ticks = now_ms.saturating_mul(EMBASSY_TICKS_PER_MILLISECOND);
    let deadline_ticks = critical_section::with(|cs| {
        EMBASSY_TIME_QUEUE
            .borrow(cs)
            .borrow_mut()
            .next_expiration(now_ticks)
    });
    if deadline_ticks == u64::MAX {
        return None;
    }
    let remaining_ticks = deadline_ticks.saturating_sub(now_ticks);
    let delay_ms = remaining_ticks.saturating_add(EMBASSY_TICKS_PER_MILLISECOND - 1)
        / EMBASSY_TICKS_PER_MILLISECOND;
    Some(now_ms.saturating_add(delay_ms.max(1)))
}

#[cfg(not(feature = "embassy"))]
fn embassy_next_expiration(_now: u64) -> Option<u64> {
    None
}

fn rearm_timer() {
    let state = start_state();
    let Some(port) = state.port else {
        return;
    };
    let now = (state.resources.monotonic_ms)();
    let (wake_deadline, slice_deadline) = critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        (
            scheduler.earliest_wake_deadline(),
            scheduler.next_time_slice_deadline(now, state.config.time_slice),
        )
    });
    let deadline = earliest_deadline(wake_deadline, slice_deadline, embassy_next_expiration(now));
    let Some(deadline) = deadline else {
        (port.disarm_timer)();
        return;
    };
    let remaining = deadline.saturating_sub(now).max(1);
    let delay = remaining.min(port.max_timer_delay.get() as u64) as u32;
    (port.arm_timer)(NonZeroU32::new(delay).unwrap());
}

/// Handles expiry of the injected scheduler timer.
///
/// The target handler must acknowledge its hardware source, call
/// [`interrupt_enter`], invoke this function, then call [`interrupt_exit`]. The
/// runtime epilogue performs any resulting context switch on the interrupted
/// task's stack.
pub fn on_timer_interrupt() {
    let now = now_ms();
    critical_section::with(|cs| {
        let scheduler = &mut *SCHED.borrow_ref_mut(cs);
        scheduler.diagnostics.timer_interrupts =
            scheduler.diagnostics.timer_interrupts.saturating_add(1);
        scheduler.wake_sleepers(now);
        if scheduler.time_slice_deadline != 0 && now >= scheduler.time_slice_deadline {
            scheduler.time_slice_pending = true;
            scheduler.time_slice_deadline = 0;
        }
    });
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
    let slot = critical_section::with(|cs| {
        let s = &mut *SCHED.borrow_ref_mut(cs);
        let i = s.alloc_slot()?;
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
        s.ready_push(i);
        Some(i)
    });
    if slot.is_none() {
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
    rearm_timer();
    // SAFETY: contexts live in the static SCHED (stable address); single-hart,
    // and the lock is released so the resumed task can re-enter the scheduler.
    unsafe { context_switch(op, np) };
}

fn switch_away(prev: usize) {
    loop {
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
    rearm_timer();
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
        if matches!(current, Some(slot) if slot != NIL) {
            rearm_timer();
        }
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
        rearm_timer();
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

            Ok(if INTERRUPT_DEPTH.borrow(cs).get() == 0 {
                s.take_preemption_target()
            } else {
                None
            })
        })?;
        if let Some((current, next)) = preemption {
            switch_to(current, next);
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
    for _ in 0..MAX_TASKS {
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

/// Starts the scheduler and installs it as the firmware's sole radio runtime.
pub fn start(config: Config, resources: Resources) -> Result<(), StartError> {
    start_inner(config, resources, None)
}

/// Starts the scheduler with a target timer and deferred-reschedule port.
pub fn start_with_port(
    config: Config,
    resources: Resources,
    port: SchedulerPort,
) -> Result<(), StartError> {
    start_inner(config, resources, Some(port))
}

fn start_inner(
    config: Config,
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
            scheduler.tasks[slot].base_priority = priority;
            scheduler.refresh_inherited_priority(slot, 0);
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
    fn irq_epilogue_preempts_only_after_outermost_interrupt_exit() {
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.priority_scheduling = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 10;
        ready_task(&mut scheduler, 1, 4);

        assert_eq!(scheduler.take_irq_epilogue_target(1), None);
        assert_eq!(scheduler.diagnostics.irq_preemptions, 0);
        assert_eq!(scheduler.take_irq_epilogue_target(0), Some((0, 1)));
        assert_eq!(scheduler.diagnostics.irq_preemptions, 1);
    }

    #[test]
    fn irq_epilogue_does_not_preempt_cooperative_or_stopped_scheduler() {
        let mut scheduler = Sched::new();
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 10;
        ready_task(&mut scheduler, 1, 4);

        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        scheduler.started = true;
        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        assert_eq!(scheduler.diagnostics.irq_preemptions, 0);
    }

    #[test]
    fn expired_time_slice_round_robins_equal_priority_tasks() {
        let mut scheduler = Sched::new();
        scheduler.started = true;
        scheduler.priority_scheduling = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 4;
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
        scheduler.priority_scheduling = true;
        scheduler.tasks[0].state = State::Running;
        scheduler.tasks[0].priority = 4;
        ready_task(&mut scheduler, 1, 4);
        scheduler.time_slice_pending = true;
        scheduler.lock_current().unwrap();

        assert_eq!(scheduler.take_irq_epilogue_target(0), None);
        assert!(scheduler.time_slice_pending);
        assert_eq!(
            scheduler.unlock_current_and_take_preemption().unwrap(),
            Some((0, 1))
        );
        assert!(!scheduler.time_slice_pending);
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
        assert_eq!(earliest_deadline(Some(30), Some(20), Some(10)), Some(10));
        assert_eq!(earliest_deadline(Some(30), Some(20), None), Some(20));
        assert_eq!(earliest_deadline(Some(30), None, Some(40)), Some(30));
        assert_eq!(earliest_deadline(None, None, Some(40)), Some(40));
        assert_eq!(earliest_deadline(None, None, None), None);
    }

    #[test]
    fn unrelated_deadline_rearm_does_not_postpone_time_slice() {
        let mut scheduler = Sched::new();
        ready_task(&mut scheduler, 1, 4);
        let slice = NonZeroU32::new(5);

        assert_eq!(scheduler.next_time_slice_deadline(10, slice), Some(15));
        assert_eq!(scheduler.next_time_slice_deadline(12, slice), Some(15));

        scheduler.time_slice_deadline = 0;
        assert_eq!(scheduler.next_time_slice_deadline(15, slice), Some(20));
        scheduler.ready_pop();
        assert_eq!(scheduler.next_time_slice_deadline(16, slice), None);
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
        scheduler.priority_scheduling = true;
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
        scheduler.priority_scheduling = true;
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
        scheduler.priority_scheduling = true;
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
        scheduler.priority_scheduling = true;
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
        scheduler.priority_scheduling = true;
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
