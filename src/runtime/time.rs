use super::*;

static TIMER_REARM_GENERATION: Mutex<Cell<u64>> = Mutex::new(Cell::new(0));

#[cfg(feature = "embassy")]
static EMBASSY_TIME_QUEUE: Mutex<RefCell<EmbassyTimeQueue>> =
    Mutex::new(RefCell::new(EmbassyTimeQueue::new()));

#[cfg(feature = "embassy")]
const EMBASSY_TICKS_PER_MILLISECOND: u64 = embassy_time_driver::TICK_HZ / 1_000;

#[cfg(feature = "embassy")]
const _: () = assert!(embassy_time_driver::TICK_HZ.is_multiple_of(1_000));

pub(super) fn now_ms() -> u64 {
    (start_state().resources.monotonic_ms)()
}

pub(super) fn earliest_deadline(
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

pub(super) fn claim_timer_rearm_generation(cell: &Cell<u64>) -> u64 {
    let generation = cell.get().wrapping_add(1);
    cell.set(generation);
    generation
}

pub(super) fn rearm_timer() {
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
