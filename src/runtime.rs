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

mod scheduler;
use scheduler::{Sched, State, TaskFn, Tcb};
mod reservation;
use reservation::ReservationTable;
mod sync;
use sync::{
    MutexState, RtosMutex, SemState, Semaphore, cancel_wait_locked, enqueue_mutex_waiter,
    enqueue_waiter, remove_mutex_waiter, remove_waiter,
};
#[cfg(test)]
use sync::{release_mutex_locked, release_semaphore_locked};
mod time;
#[cfg(test)]
use time::{claim_timer_rearm_generation, earliest_deadline};
use time::{now_ms, rearm_timer};
pub use time::{on_software_interrupt, on_timer_interrupt, request_reschedule};
mod driver;
mod resource;
use resource::{RESOURCE_HANDLE_CAPACITY, ResourceKind, ResourceTable};

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
    Error as DriverError, MutexHandle, Runtime, RuntimeContract, RuntimeExecutionProfile,
    SemaphoreHandle, TaskAdmissionError, TaskCapacity, TaskConfig, TaskPriority, TaskReservation,
    WaitCancellationOutcome, WaitOutcome, WaitTimeout,
};

/// Contract-v1 priority levels: 0 is highest, 31 is lowest.
const PRIORITY_LEVELS: usize = hisi_rf_rtos_driver::TASK_PRIORITY_LEVELS as usize;
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

static SCHED: Mutex<RefCell<Sched>> = Mutex::new(RefCell::new(Sched::new()));
static INTERRUPT_DEPTH: Mutex<Cell<u16>> = Mutex::new(Cell::new(0));
static RESOURCE_HANDLES: Mutex<RefCell<ResourceTable<RESOURCE_HANDLE_CAPACITY>>> =
    Mutex::new(RefCell::new(ResourceTable::new()));

/// Marks entry into a target interrupt handler.
///
/// ISR-safe wakeups may make tasks ready, but task context switching is
/// deferred until after interrupt exit.
#[doc(hidden)]
pub fn interrupt_enter() {
    let now = start_state_opt().map(|state| (state.resources.monotonic_ms)());
    critical_section::with(|cs| {
        let depth = INTERRUPT_DEPTH.borrow(cs);
        if depth.get() == 0
            && let Some(now) = now
        {
            SCHED.borrow_ref_mut(cs).interrupt_enter(now);
        }
        depth.set(depth.get().saturating_add(1));
    });
}

/// Marks exit from a target interrupt handler.
#[doc(hidden)]
pub fn interrupt_exit() {
    let now = start_state_opt().map(|state| (state.resources.monotonic_ms)());
    critical_section::with(|cs| {
        let depth = INTERRUPT_DEPTH.borrow(cs);
        debug_assert!(depth.get() != 0);
        if depth.get() == 1
            && let Some(now) = now
        {
            SCHED.borrow_ref_mut(cs).interrupt_exit(now);
        }
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
        s.tasks[0].metrics.on_dispatch(now);
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
        idle.metrics.on_ready(now);
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
    reservation: Option<&TaskReservation>,
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
        let i = match reservation {
            Some(reservation) => s.alloc_reserved_dynamic_slot(reservation)?,
            None => s.alloc_dynamic_slot()?,
        };
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
        t.stack = stack as usize;
        t.entry = Some(entry);
        t.arg = arg as usize;
        t.wake_at = 0;
        t.base_priority = priority;
        t.priority = priority;
        t.identity_generation = identity_generation;
        t.run_policy = run_policy;
        t.budget = BudgetState::for_policy(run_policy, now);
        s.make_ready(i, now);
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
            if s.recover_completed_switch_request(prev, next) {
                return None;
            }
            assert_eq!(s.forced_next, NIL, "a trap switch is already pending");
            assert!(
                s.tasks[next].saved_frame != 0,
                "target task has no trap frame"
            );
            s.forced_next = next;
            Some(s.tasks[prev].resume_generation)
        });
        let Some(generation) = generation else {
            rearm_timer();
            return;
        };
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
        Ok(s.take_yield_target(cur, now).map(|next| (cur, next)))
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
    let now = now_ms();
    critical_section::with(|cs| SCHED.borrow_ref(cs).task_diagnostics(output, now))
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

#[cfg(test)]
mod conformance;
#[cfg(test)]
mod tests;
