use super::*;
use std::{
    sync::{
        Arc,
        atomic::{AtomicUsize, Ordering},
    },
    task::Wake,
};
struct Count(AtomicUsize);
impl Wake for Count {
    fn wake(self: Arc<Self>) {
        self.0.fetch_add(1, Ordering::SeqCst);
    }
}
fn counter() -> (Arc<Count>, Waker) {
    let c = Arc::new(Count(AtomicUsize::new(0)));
    let w = Waker::from(c.clone());
    (c, w)
}
#[test]
fn alarm_segments_keep_absolute_deadline_without_overflow() {
    assert_eq!(alarm_counts(0, 1, false), 1);
    assert_eq!(alarm_counts(10, 9, false), COUNTS_PER_TICK.get());
    assert_eq!(alarm_counts(u64::MAX, 0, false), u32::MAX);
    assert_eq!(alarm_counts(u64::MAX, 0, true), RETRY_COUNTS);
    assert_eq!(alarm_counts(10, 9, true), COUNTS_PER_TICK.get());
    for delta in [1, 2, 1000, 134217727, 134217728, u64::MAX] {
        let count = u64::from(alarm_counts(delta, 0, false));
        assert!(count <= u32::MAX as u64);
        if delta <= u32::MAX as u64 / u64::from(COUNTS_PER_TICK.get()) {
            assert_eq!(count, delta * u64::from(COUNTS_PER_TICK.get()));
        } else {
            assert_eq!(count, u32::MAX as u64);
        }
    }
}
#[test]
fn clock_faults_hold_epoch_and_recovery_never_rewinds() {
    let mut state = State::<Hardware>::new();
    state.accept_ticks(u64::from(COUNTS_PER_TICK.get()) * 1000);
    assert_eq!(state.last, 1000);
    state.accept(Err(nb::Error::WouldBlock));
    assert_eq!(state.health, ClockStatus::SnapshotPending);
    assert_eq!(state.last, 1000);
    state.accept(Err(nb::Error::Other(ClockError::Disabled)));
    assert_eq!(state.health, ClockStatus::Disabled);
    assert_eq!(state.last, 1000);
    state.accept_ticks(0);
    assert_eq!(state.health, ClockStatus::Backwards);
    assert_eq!(state.last, 1000);
    state.accept_ticks(u64::from(COUNTS_PER_TICK.get()) * 2000);
    assert_eq!(state.health, ClockStatus::Ready);
    assert_eq!(state.last, 2000);
    state.accept_ticks(u64::MAX);
    assert_eq!(state.last, u64::MAX / u64::from(COUNTS_PER_TICK.get()));
}
#[test]
fn uninitialized_now_never_accesses_mmio_and_past_schedule_wakes() {
    let driver = TimeDriver::<Hardware> {
        state: Mutex::new(RefCell::new(State::new())),
    };
    assert_eq!(driver.now(), 0);
    let (c, w) = counter();
    driver.schedule_wake(0, &w);
    assert_eq!(c.0.load(Ordering::SeqCst), 1);
    driver.schedule_wake(100, &w);
    assert_eq!(driver.now(), 0);
    assert_eq!(driver.with(|s| s.queue.len()), 1);
}
// These tests inject events at actual scheduler/backend boundaries; they do not
// model MMIO latency, clock synchronization or multi-core ownership.
use std::{boxed::Box, sync::Mutex as StdMutex};

struct Model {
    sample: nb::Result<u64, ClockError>,
    armed: Option<u32>,
    arm_hook: Option<Box<dyn FnOnce() + Send>>,
    polls: usize,
}
#[derive(Clone)]
struct Mock(Arc<StdMutex<Model>>);
impl Backend for Mock {
    fn mask_interrupt() {}
    fn unmask_interrupt() {}
    fn sample(&mut self) -> nb::Result<u64, ClockError> {
        let mut m = self.0.lock().unwrap();
        m.polls += 1;
        m.sample
    }
    fn stop(&mut self) {
        self.0.lock().unwrap().armed = None;
    }
    fn arm(&mut self, counts: u32) {
        let hook = {
            let mut m = self.0.lock().unwrap();
            m.armed = Some(counts);
            m.arm_hook.take()
        };
        if let Some(hook) = hook {
            hook();
        }
    }
}
fn model() -> (Arc<TimeDriver<Mock>>, Mock) {
    let mock = Mock(Arc::new(StdMutex::new(Model {
        sample: Ok(0),
        armed: None,
        arm_hook: None,
        polls: 0,
    })));
    let driver = Arc::new(TimeDriver {
        state: Mutex::new(RefCell::new(State::new())),
    });
    assert!(driver.install(mock.clone(), |_| Ok(0)).is_ok());
    (driver, mock)
}
fn advance(mock: &Mock, ticks: u64) {
    mock.0.lock().unwrap().sample = Ok(ticks * u64::from(COUNTS_PER_TICK.get()));
}

#[test]
fn actual_service_ignores_empty_duplicate_irqs_and_never_wakes_early() {
    let (driver, mock) = model();
    let (count, waker) = counter();
    driver.service(true);
    assert_eq!(mock.0.lock().unwrap().armed, None);
    driver.schedule_wake(10, &waker);
    advance(&mock, 9);
    driver.service(true);
    driver.service(true);
    assert_eq!(count.0.load(Ordering::SeqCst), 0);
    advance(&mock, 10);
    driver.service(true);
    driver.service(true);
    assert_eq!(count.0.load(Ordering::SeqCst), 1);
    assert_eq!(mock.0.lock().unwrap().armed, None);
}

#[test]
fn registration_during_alarm_programming_is_repolled_not_lost() {
    let (driver, mock) = model();
    let (early, early_waker) = counter();
    let (later, later_waker) = counter();
    let reenter = driver.clone();
    mock.0.lock().unwrap().arm_hook = Some(Box::new(move || {
        reenter.schedule_wake(5, &early_waker);
        reenter.service(true); // IRQ arriving while hardware is borrowed
    }));
    driver.schedule_wake(100, &later_waker);
    assert_eq!(early.0.load(Ordering::SeqCst), 1);
    assert_eq!(later.0.load(Ordering::SeqCst), 1);
    assert_eq!(driver.with(|s| s.queue.len()), 0);
    // Early wakes are invitations to poll, never successful Timer completion.
    assert_eq!(driver.now(), 0);
}

#[test]
fn deadline_crossed_during_programming_is_serviced_on_next_irq() {
    let (driver, mock) = model();
    let (count, waker) = counter();
    let m = mock.clone();
    mock.0.lock().unwrap().arm_hook = Some(Box::new(move || advance(&m, 20)));
    driver.schedule_wake(10, &waker);
    assert_eq!(count.0.load(Ordering::SeqCst), 0);
    driver.service(true);
    assert_eq!(count.0.load(Ordering::SeqCst), 1);
}

#[test]
fn snapshot_stall_retries_and_clock_wrap_is_an_epoch_fault() {
    let (driver, mock) = model();
    advance(&mock, 100);
    assert_eq!(driver.now(), 100);
    let (count, waker) = counter();
    mock.0.lock().unwrap().sample = Err(nb::Error::WouldBlock);
    driver.schedule_wake(1_000_000, &waker);
    assert_eq!(mock.0.lock().unwrap().armed, Some(RETRY_COUNTS));
    assert_eq!(count.0.load(Ordering::SeqCst), 0);
    mock.0.lock().unwrap().sample = Ok(u64::MAX);
    driver.service(true);
    let last = driver.now();
    advance(&mock, 0); // a full counter wrap cannot be distinguished from reset
    assert_eq!(driver.now(), last);
    assert_eq!(driver.with(|s| s.health), ClockStatus::Backwards);
}

#[test]
fn failed_install_returns_resource_and_duplicate_never_runs_validation() {
    let (_, mock) = model();
    let driver = TimeDriver {
        state: Mutex::new(RefCell::new(State::new())),
    };
    let (_, returned) = driver
        .install(mock, |_| Err(InitError::TimerRunning))
        .err()
        .unwrap();
    assert!(!driver.with(|s| s.initialized || s.installing));
    assert!(driver.install(returned.clone(), |_| Ok(0)).is_ok());
    let result = driver.install(returned, |_| panic!("duplicate must not touch hardware"));
    assert!(matches!(result, Err((InitError::AlreadyInitialized, _))));
}

#[test]
fn cancellation_retains_bounded_wake_and_callback_can_schedule_again() {
    struct Reenter(Arc<TimeDriver<Mock>>, Waker);
    impl Wake for Reenter {
        fn wake(self: Arc<Self>) {
            self.0.schedule_wake(20, &self.1);
        }
    }
    let (driver, mock) = model();
    let (second, second_waker) = counter();
    let waker = Waker::from(Arc::new(Reenter(driver.clone(), second_waker)));
    driver.schedule_wake(10, &waker);
    drop(waker); // cancelling a wait does not retract a registered wake
    advance(&mock, 10);
    driver.service(true); // callback schedules another deadline after borrow release
    assert_eq!(driver.with(|s| s.queue.len()), 1);
    advance(&mock, 20);
    driver.service(true);
    assert_eq!(second.0.load(Ordering::SeqCst), 1);
}
