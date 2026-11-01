//! Concrete HAL adaptation only: no queue, waker or deadline policy.
use super::standalone::InitError;
use core::num::NonZeroU32;
use hisi_hal::{
    interrupt,
    tcxo::TcxoDriver,
    timer::{TimerChannel, TimerDriver, TimerMode},
};

#[cfg(feature = "chip-ws53")]
pub use hisi_hal::tcxo::TcxoError as ClockError;
/// Failure to sample the inherited legacy TCXO epoch.
#[cfg(not(feature = "chip-ws53"))]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum ClockError {
    /// Counter disabled; no refresh was issued.
    Disabled,
    /// Bounded initialization refresh did not complete.
    SnapshotTimeout,
}

#[cfg(feature = "chip-ws53")]
const CHANNEL: TimerChannel = TimerChannel::Channel1;
#[cfg(not(feature = "chip-ws53"))]
const CHANNEL: TimerChannel = TimerChannel::Channel0;
#[cfg(feature = "chip-ws53")]
const IRQ: interrupt::Interrupt = interrupt::Interrupt::TIMER_1;
#[cfg(not(feature = "chip-ws53"))]
const IRQ: interrupt::Interrupt = hisi_hal::soc::chip::ALARM_INTERRUPT;
pub const ALARM_IRQ: u32 = IRQ as u32;
const CLOCK_HZ: u32 = hisi_hal::soc::chip::TIMER_CLOCK_HZ;
pub const COUNTS_PER_TICK: NonZeroU32 = NonZeroU32::new(CLOCK_HZ / 1_000_000).unwrap();
pub const RETRY_COUNTS: u32 = CLOCK_HZ / 1_000;
const _: () = assert!(CLOCK_HZ.is_multiple_of(1_000_000));
#[cfg(not(feature = "chip-ws53"))]
const _: () = assert!(CLOCK_HZ == hisi_hal::soc::chip::TCXO_HZ);

/// Internal seam for testing the scheduler against an event-injecting backend.
pub(super) trait Backend {
    fn mask_interrupt();
    fn unmask_interrupt();
    fn sample(&mut self) -> nb::Result<u64, ClockError>;
    fn stop(&mut self);
    fn arm(&mut self, counts: u32);
}

pub(super) struct Hardware {
    timer: TimerDriver<'static>,
    clock: TcxoDriver<'static>,
}
impl Hardware {
    pub fn new(timer: TimerDriver<'static>, clock: TcxoDriver<'static>) -> Self {
        Self { timer, clock }
    }
    pub fn into_parts(self) -> (TimerDriver<'static>, TcxoDriver<'static>) {
        (self.timer, self.clock)
    }
    pub fn check(&mut self) -> Result<u64, InitError> {
        #[cfg(feature = "chip-ws53")]
        {
            for ch in [TimerChannel::Channel0, TimerChannel::Channel1] {
                if self.timer.is_enabled(ch) {
                    return Err(InitError::TimerRunning);
                }
                if interrupt::is_enabled(ch.interrupt())
                    || interrupt::is_pending(ch.interrupt())
                    || self.timer.interrupt_pending(ch)
                {
                    return Err(InitError::TimerInterruptActive);
                }
            }
            self.clock.snapshot_ready().map_err(InitError::Clock)?;
            self.clock
                .read_counter()
                .map(|c| c.ticks())
                .map_err(InitError::Clock)
        }
        #[cfg(not(feature = "chip-ws53"))]
        {
            // Other channels must also be quiescent by the unsafe handoff
            // contract. No ownership is inferred from these alarm checks.
            if self.timer.is_enabled(CHANNEL) {
                return Err(InitError::TimerRunning);
            }
            if interrupt::is_enabled(IRQ)
                || interrupt::is_pending(IRQ)
                || self.timer.interrupt_pending(CHANNEL)
            {
                return Err(InitError::TimerInterruptActive);
            }
            if !self.clock.is_enabled() {
                return Err(InitError::Clock(ClockError::Disabled));
            }
            self.clock
                .read_counter()
                .ok_or(InitError::Clock(ClockError::SnapshotTimeout))
        }
    }
}
impl Backend for Hardware {
    fn mask_interrupt() {
        unsafe { interrupt::disable(IRQ) };
    }
    fn unmask_interrupt() {
        unsafe { interrupt::enable(IRQ) };
    }
    fn sample(&mut self) -> nb::Result<u64, ClockError> {
        #[cfg(feature = "chip-ws53")]
        {
            self.clock.poll_counter().map(|c| c.ticks())
        }
        #[cfg(not(feature = "chip-ws53"))]
        {
            self.clock.poll_counter().map_err(|e| match e {
                nb::Error::WouldBlock => nb::Error::WouldBlock,
                nb::Error::Other(()) => nb::Error::Other(ClockError::Disabled),
            })
        }
    }
    fn stop(&mut self) {
        self.timer.disable(CHANNEL);
        self.timer.set_interrupt_mask(CHANNEL, true);
        self.timer.clear_interrupt(CHANNEL);
        interrupt::clear_pending(IRQ);
    }
    fn arm(&mut self, counts: u32) {
        #[cfg(feature = "chip-ws53")]
        self.timer.configure(
            CHANNEL,
            TimerMode::OneShot,
            hisi_hal::timer::TimerTicks::from_count(counts).expect("nonzero alarm segment"),
        );
        #[cfg(not(feature = "chip-ws53"))]
        self.timer.configure(CHANNEL, TimerMode::OneShot, counts);
        self.timer.enable(CHANNEL);
    }
}
