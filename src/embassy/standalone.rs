use super::backend::{Backend, ClockError, Hardware};
use core::{cell::RefCell, num::NonZeroU32, task::Waker};
use critical_section::Mutex;
use embassy_time_driver::Driver;
use hisi_hal::{tcxo::TcxoDriver, timer::TimerDriver};

/// Maximum number of distinct retained task wakers.
pub use super::QUEUE_CAPACITY;
use super::queue::Queue;
const COUNTS_PER_TICK: NonZeroU32 = super::backend::COUNTS_PER_TICK;
const RETRY_COUNTS: u32 = super::backend::RETRY_COUNTS;
const _: () = assert!(embassy_time_driver::TICK_HZ == 1_000_000);

/// Current time-source observation. No fault is silently represented as timestamp0.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockStatus {
    /// No resources installed; now is zero and no hardware is touched.
    Uninitialized,
    /// Last snapshot advanced or preserved the timestamp.
    Ready,
    /// A nonblocking snapshot has not completed; now returns its previous value.
    SnapshotPending,
    /// Snapshot could not be started because TCXO was disabled externally.
    Disabled,
    /// Snapshot moved backwards; now holds its previous value. Epoch contract was broken.
    Backwards,
}
/// Read-only diagnostic state; no ownership or counter reset is implied.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub struct Status {
    /// Driver resources have been installed.
    pub initialized: bool,
    /// Latest snapshot observation.
    pub clock: ClockStatus,
    /// Last accepted microsecond timestamp.
    pub ticks: u64,
    /// Number of distinct task wakers currently retained.
    pub pending_wakers: usize,
}
/// Refused time-driver installation; both resources are returned to the caller.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum InitError {
    /// Installation already completed or is in progress.
    AlreadyInitialized,
    /// Either TIMER channel is running.
    TimerRunning,
    /// Either TIMER local interrupt is enabled or a pending source exists.
    TimerInterruptActive,
    /// TCXO was disabled/busy or its initial bounded snapshot did not complete.
    Clock(ClockError),
}

struct State<B> {
    resources: Option<B>,
    initialized: bool,
    installing: bool,
    busy: bool,
    dirty: bool,
    last: u64,
    health: ClockStatus,
    queue: Queue,
}
impl<B> State<B> {
    const fn new() -> Self {
        Self {
            resources: None,
            initialized: false,
            installing: false,
            busy: false,
            dirty: false,
            last: 0,
            health: ClockStatus::Uninitialized,
            queue: Queue::new(),
        }
    }
    fn accept_ticks(&mut self, raw: u64) {
        let ticks = super::math::counter_ticks(raw, COUNTS_PER_TICK);
        if ticks < self.last {
            self.health = ClockStatus::Backwards;
        } else {
            self.last = ticks;
            self.health = ClockStatus::Ready;
        }
    }
    fn accept(&mut self, sample: nb::Result<u64, ClockError>) {
        match sample {
            Ok(value) => {
                self.accept_ticks(value);
            }
            Err(nb::Error::WouldBlock) => self.health = ClockStatus::SnapshotPending,
            Err(nb::Error::Other(_)) => self.health = ClockStatus::Disabled,
        }
    }
}
struct TimeDriver<B> {
    state: Mutex<RefCell<State<B>>>,
}
fn alarm_counts(at: u64, now: u64, retry: bool) -> u32 {
    let count = super::math::alarm_counts(at, now, COUNTS_PER_TICK);
    if retry {
        count.min(RETRY_COUNTS)
    } else {
        count
    }
}
fn wake_all(batch: [Option<Waker>; QUEUE_CAPACITY]) {
    for waker in batch.into_iter().flatten() {
        waker.wake();
    }
}
impl<B: Backend> TimeDriver<B> {
    fn install(
        &self,
        mut resources: B,
        check: impl FnOnce(&mut B) -> Result<u64, InitError>,
    ) -> Result<(), (InitError, B)> {
        let reserved = self.with(|s| {
            if s.initialized || s.installing {
                false
            } else {
                s.installing = true;
                true
            }
        });
        if !reserved {
            return Err((InitError::AlreadyInitialized, resources));
        }
        let check = check(&mut resources);
        match check {
            Err(error) => {
                self.with(|s| s.installing = false);
                Err((error, resources))
            }
            Ok(value) => {
                self.with(|s| {
                    s.accept(Ok(value));
                    s.resources = Some(resources);
                    s.initialized = true;
                    s.installing = false;
                });
                self.service(false);
                Ok(())
            }
        }
    }

    fn with<R>(&self, f: impl FnOnce(&mut State<B>) -> R) -> R {
        critical_section::with(|cs| f(&mut self.state.borrow(cs).borrow_mut()))
    }
    fn service(&self, from_irq: bool) -> u64 {
        let resources = self.with(|s| {
            if !s.initialized {
                return None;
            }
            if s.busy {
                if from_irq {
                    s.dirty = true;
                    B::mask_interrupt();
                }
                return None;
            }
            s.busy = true;
            s.resources.take()
        });
        let Some(mut resources) = resources else {
            return self.with(|s| s.last);
        };
        B::mask_interrupt();
        // No polling loop and no MMIO within the driver metadata critical section.
        let sample = resources.sample();
        let (expired, next, now, retry) = self.with(|s| {
            s.accept(sample);
            let expired = s.queue.drain(s.last, false);
            let next = s.queue.next();
            s.dirty = false;
            (expired, next, s.last, s.health != ClockStatus::Ready)
        });
        resources.stop();
        if let Some(at) = next {
            resources.arm(alarm_counts(at, now, retry));
        }
        let changed = self.with(|s| {
            // Another ISR may have scheduled an earlier deadline while registers
            // were being programmed. Wake those queued tasks for a fresh poll,
            // rather than losing them behind the stale alarm. The queue is bounded.
            let changed = if s.dirty {
                s.queue.drain(0, true)
            } else {
                [const { None }; QUEUE_CAPACITY]
            };
            s.dirty = false;
            s.resources = Some(resources);
            s.busy = false;
            // Only local-controller bookkeeping is performed in this section.
            if next.is_some() {
                B::unmask_interrupt();
            }
            changed
        });
        wake_all(expired);
        wake_all(changed);
        self.with(|s| s.last)
    }
}
impl<B: Backend + Send + 'static> Driver for TimeDriver<B> {
    fn now(&self) -> u64 {
        self.service(false)
    }
    fn schedule_wake(&self, at: u64, waker: &Waker) {
        let cloned = waker.clone();
        let (old, evicted, immediate) = self.with(|s| {
            if at <= s.last {
                return (None, None, Some(cloned));
            }
            let (old, evicted) = s.queue.insert(at, cloned);
            s.dirty = true;
            (old, evicted, None)
        });
        self.service(false);
        drop(old);
        if let Some(waker) = evicted {
            waker.wake();
        }
        if let Some(waker) = immediate {
            waker.wake();
        }
    }
}
static DRIVER: TimeDriver<Hardware> = TimeDriver {
    state: Mutex::new(RefCell::new(State::new())),
};
pub(super) fn now() -> u64 {
    DRIVER.now()
}
pub(super) fn schedule_wake(at: u64, waker: &Waker) {
    DRIVER.schedule_wake(at, waker);
}

/// Permanently install the supplied TIMER block and TCXO snapshot window.
/// Both resources are returned on error. Validation may issue a bounded TCXO
/// snapshot; timeout can leave that request pending. No clock enabling/reset,
/// queue reset, pad changes or cross-core negotiation is performed.
///
/// This reserves the entire TIMER block (WS53 alarm channel 1, otherwise 0).
/// It cannot simultaneously back HAL delays, DMA wait polling or RTOS ticks.
/// The statically registered provider has no deinitialization or resource return
/// after success. Before initialization, time is zero and no MMIO is accessed.
///
/// # Safety
/// Platform must establish exclusive ownership, quiescing other cores, DMA,
/// NMI and previous users of both blocks; all TIMER channels must be stopped.
/// Preserve the configured uninterrupted epoch: 32 MHz WS53/BS2X, 24 MHz WS63.
/// These are configuration assumptions, not measurements. No clock changes,
/// counter reset or sleep stopping either clock are supported after handoff.
/// All calls and ISR delivery must run on one application hart. Exclusively
/// route [`super::ALARM_IRQ`] to [`on_alarm_interrupt`] without reentrant IRQ
/// delivery. Set trap routing/threshold before init and global MIE afterwards.
/// Wakers must be ISR-safe. Select exactly one global Embassy time provider.
#[allow(clippy::result_large_err)]
pub unsafe fn init(
    timer: TimerDriver<'static>,
    clock: TcxoDriver<'static>,
) -> Result<(), (InitError, TimerDriver<'static>, TcxoDriver<'static>)> {
    DRIVER
        .install(Hardware::new(timer, clock), Hardware::check)
        .map_err(|(error, resources)| {
            let (timer, clock) = resources.into_parts();
            (error, timer, clock)
        })
}
/// Service the selected alarm interrupt. Does not spin on the TCXO or invoke a waker with state borrowed.
/// Call only from the installed ISR (or while its local interrupt is disabled).
/// Spurious invocations recheck deadlines; they never complete time early.
pub fn on_alarm_interrupt() {
    DRIVER.service(true);
}
/// Observe initialization, monotonic cache and bounded queue occupancy without MMIO.
pub fn status() -> Status {
    DRIVER.with(|s| Status {
        initialized: s.initialized,
        clock: s.health,
        ticks: s.last,
        pending_wakers: s.queue.len(),
    })
}

#[cfg(all(test, not(target_arch = "riscv32")))]
#[path = "tests.rs"]
mod tests;
