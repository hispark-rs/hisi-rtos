use crate::RunPolicy;
use core::marker::PhantomData;
use core::num::{NonZeroU32, NonZeroUsize};
use hisi_rf_rtos_driver::Error as DriverError;

/// The application thread adopted when the scheduler starts.
pub(crate) const ADOPTED_MAIN_TASKS: usize = 1;
/// Scheduler-owned idle threads that cannot be allocated by applications.
pub(crate) const INTERNAL_IDLE_TASKS: usize = 1;
/// Dynamic task slots available through the runtime contract.
pub const DYNAMIC_TASK_CAPACITY: usize = 15;
/// Total scheduler slots, including adopted and internal threads.
pub(crate) const TASK_SLOT_COUNT: usize =
    ADOPTED_MAIN_TASKS + INTERNAL_IDLE_TASKS + DYNAMIC_TASK_CAPACITY;

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
    pub(crate) const fn new() -> Self {
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
    /// Cumulative dispatch-to-switch wall time charged to this task.
    ///
    /// This includes time spent servicing interrupts while the task was the
    /// interrupted context; subtract [`Self::irq_time_ms`] for the corresponding
    /// thread-mode estimate.
    pub cpu_time_ms: u64,
    /// Cumulative outermost interrupt-handler time attributed to this task.
    pub irq_time_ms: u64,
    /// Number of times this task became the running task.
    pub dispatches: u32,
    /// Number of this task's Budgeted quota exhaustion events.
    pub budget_exhaustions: u32,
    /// Longest uninterrupted dispatch-to-switch interval observed.
    pub max_continuous_run_ms: u64,
    /// Longest interval from becoming ready to being dispatched.
    pub max_ready_latency_ms: u64,
    /// Number of outermost scheduler-lock acquisitions by this task.
    pub scheduler_lock_entries: u32,
    /// Longest outermost scheduler-lock hold interval observed.
    pub max_scheduler_lock_ms: u64,
    /// Number of outermost interrupt entries attributed to this task.
    pub irq_entries: u32,
    /// Longest outermost interrupt-handler interval attributed to this task.
    pub max_irq_span_ms: u64,
}

impl Diagnostics {
    pub(crate) const EMPTY: Self = Self {
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
