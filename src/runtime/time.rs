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
pub(crate) fn embassy_now_ticks() -> u64 {
    start_state_opt()
        .map(|state| (state.resources.monotonic_ms)().saturating_mul(EMBASSY_TICKS_PER_MILLISECOND))
        .unwrap_or(0)
}

#[cfg(feature = "embassy")]
fn embassy_next_expiration(now_ms: u64) -> Option<u64> {
    let now_ticks = now_ms.saturating_mul(EMBASSY_TICKS_PER_MILLISECOND);
    let (expired, next) = critical_section::with(|cs| {
        let mut queue = EMBASSY_TIME_QUEUE.borrow(cs).borrow_mut();
        let expired = queue.drain(now_ticks, false);
        (expired, queue.next())
    });
    // A callback may reenter schedule_wake. No queue/scheduler borrow is held.
    for waker in expired.into_iter().flatten() {
        waker.wake();
    }
    next.map(|at| embassy_deadline_ms(at, now_ms))
}

#[cfg(feature = "embassy")]
fn embassy_deadline_ms(at: u64, now_ms: u64) -> u64 {
    let now_ticks = now_ms.saturating_mul(EMBASSY_TICKS_PER_MILLISECOND);
    let remaining = at.saturating_sub(now_ticks);
    // Quotient/remainder avoids overflow near the end of the 64-bit epoch.
    let delay = remaining / EMBASSY_TICKS_PER_MILLISECOND
        + u64::from(!remaining.is_multiple_of(EMBASSY_TICKS_PER_MILLISECOND));
    now_ms.saturating_add(delay.max(1))
}

#[cfg(not(feature = "embassy"))]
fn embassy_next_expiration(_now: u64) -> Option<u64> {
    None
}

pub(super) fn claim_timer_rearm_generation(cell: &Cell<u64>) -> u64 {
    let generation = cell.get().wrapping_add(1);
    cell.set(generation);
    generation
}

fn timer_rearm_ticket_is_current(cell: &Cell<u64>, ticket: u64) -> bool {
    cell.get() == ticket
}

pub(super) fn rearm_timer() {
    let state = start_state();
    let Some(port) = state.port else {
        return;
    };
    loop {
        let now = (state.resources.monotonic_ms)();
        // Claim before draining Embassy wakes: callbacks may publish a newer
        // timer plan. A stale outer write must fail the generation check.
        let generation = critical_section::with(|cs| {
            claim_timer_rearm_generation(TIMER_REARM_GENERATION.borrow(cs))
        });
        let embassy_deadline = embassy_next_expiration(now);
        let deadline = critical_section::with(|cs| {
            let scheduler = &mut *SCHED.borrow_ref_mut(cs);
            let lock_limit = state
                .config
                .max_scheduler_lock_duration
                .expect("ported runtime requires a scheduler-lock bound");
            earliest_deadline(
                scheduler.earliest_wake_deadline(),
                scheduler.next_dispatch_deadline(now),
                scheduler.earliest_budget_deadline(),
                scheduler.scheduler_lock_deadline(lock_limit),
                embassy_deadline,
            )
        });

        if let Some(deadline) = deadline {
            let remaining = deadline.saturating_sub(now).max(1);
            let delay = remaining.min(port.max_timer_delay.get() as u64) as u32;
            (port.arm_timer)(NonZeroU32::new(delay).unwrap());
        } else {
            (port.disarm_timer)();
        }

        let still_current = critical_section::with(|cs| {
            timer_rearm_ticket_is_current(TIMER_REARM_GENERATION.borrow(cs), generation)
        });
        if still_current {
            return;
        }
    }
}

#[cfg(kani)]
mod proofs {
    use super::*;

    #[kani::proof]
    fn stale_timer_rearm_ticket_cannot_validate_after_new_claim() {
        let initial = kani::any::<u64>();
        kani::assume(initial != u64::MAX);
        let generation = Cell::new(initial);
        let stale = claim_timer_rearm_generation(&generation);
        let current = claim_timer_rearm_generation(&generation);

        assert!(!timer_rearm_ticket_is_current(&generation, stale));
        assert!(timer_rearm_ticket_is_current(&generation, current));
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
pub(crate) fn embassy_schedule_wake(at: u64, waker: &Waker) {
    let cloned = waker.clone();
    let (old, evicted) = critical_section::with(|cs| {
        EMBASSY_TIME_QUEUE
            .borrow(cs)
            .borrow_mut()
            .insert(at, cloned)
    });
    drop(old);
    if let Some(evicted) = evicted {
        evicted.wake();
    }
    if start_state_opt().and_then(|state| state.port).is_some() {
        rearm_timer();
    }
}

#[cfg(all(test, feature = "embassy"))]
mod embassy_tests {
    use super::*;
    use std::sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    };
    use std::task::Wake;

    #[test]
    fn shared_queue_callback_can_register_again_without_borrow_or_lost_wake() {
        struct Count(AtomicUsize);
        impl Wake for Count {
            fn wake(self: Arc<Self>) {
                self.0.fetch_add(1, Ordering::SeqCst);
            }
        }
        struct Reenter(Waker);
        impl Wake for Reenter {
            fn wake(self: Arc<Self>) {
                embassy_schedule_wake(2_001, &self.0);
            }
        }
        // This is the only test touching the global shared queue. No scheduler
        // or MMIO is installed: test the real queue/callback boundary directly.
        let count = Arc::new(Count(AtomicUsize::new(0)));
        let waker = Waker::from(Arc::new(Reenter(Waker::from(count.clone()))));
        embassy_schedule_wake(1, &waker);
        assert_eq!(embassy_next_expiration(0), Some(1));
        assert_eq!(embassy_pending_wakers(), 1);
        embassy_next_expiration(1); // the callback registers during queue drain
        assert_eq!(embassy_pending_wakers(), 1);
        assert_eq!(embassy_next_expiration(2), Some(3));
        assert_eq!(count.0.load(Ordering::SeqCst), 0);
        assert_eq!(embassy_next_expiration(3), None);
        assert_eq!(count.0.load(Ordering::SeqCst), 1);
        assert_eq!(embassy_next_expiration(3), None); // duplicate notification
        assert_eq!(embassy_pending_wakers(), 0);
    }
    #[test]
    fn sub_millisecond_wait_rounds_up_without_claiming_microsecond_resolution() {
        assert_eq!(crate::embassy::RESOLUTION_US, 1_000);
        assert_eq!(embassy_deadline_ms(1, 0), 1);
        assert_eq!(embassy_deadline_ms(1000, 0), 1);
        assert_eq!(embassy_deadline_ms(1001, 0), 2);
        assert_eq!(embassy_deadline_ms(1001, 1), 2);
        assert_eq!(embassy_deadline_ms(u64::MAX, 0), u64::MAX / 1000 + 1);
    }
}

#[cfg(feature = "embassy")]
pub(crate) fn embassy_pending_wakers() -> usize {
    critical_section::with(|cs| EMBASSY_TIME_QUEUE.borrow(cs).borrow().len())
}
