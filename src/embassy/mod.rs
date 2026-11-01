//! One Embassy time registration, selected through the runtime facade.
//!
//! `embassy` shares the scheduler port and its millisecond-resolution clock.
//! `embassy-only + unstable + chip-*` consumes HAL TIMER/TCXO explicitly and
//! does not compile or start the thread scheduler. These profiles must be built
//! separately: Cargo features unify across packages in a single invocation.
//! Both use the 1 MHz tick ABI; an ABI tick is not a physical accuracy guarantee.
//!
#![cfg_attr(
    feature = "embassy-only",
    doc = r#"
The standalone profile deliberately does not expose scheduler startup or its
storage. An accidental Cargo feature union cannot start a second timer owner:
```compile_fail,E0432
use hisi_rtos::start_cooperative;
```
```compile_fail,E0432
use hisi_rtos::start_with_port;
```
```compile_fail,E0432
use hisi_rtos::SchedulerStorage;
```
"#
)]
#![cfg_attr(
    not(feature = "embassy-only"),
    doc = r#"
The shared profile retains the existing scheduler startup and storage API:
```no_run
use hisi_rtos::{start_cooperative, start_with_port, SchedulerStorage};
```
"#
)]

use core::task::Waker;
use embassy_time_driver::Driver;
#[cfg(feature = "embassy-only")]
mod backend;
#[cfg(feature = "embassy-only")]
mod math;
pub(crate) mod queue;
#[cfg(feature = "embassy-only")]
mod standalone;
#[cfg(feature = "embassy-only")]
pub use backend::ClockError;
#[cfg(feature = "embassy-only")]
pub use standalone::{ClockStatus, InitError, Status, init, on_alarm_interrupt, status};

/// Maximum distinct retained wakers in the selected profile's single queue.
pub const QUEUE_CAPACITY: usize = queue::QUEUE_CAPACITY;
/// Selected hardware alarm cause. Only the standalone profile owns this IRQ.
#[cfg(feature = "embassy-only")]
pub const ALARM_IRQ: u32 = backend::ALARM_IRQ;
/// Configured timestamp granularity in microseconds, not measured accuracy.
/// The scheduler profile injects milliseconds despite the 1 MHz tick ABI.
pub const RESOLUTION_US: u32 = if cfg!(feature = "embassy-only") {
    1
} else {
    1_000
};

struct RuntimeTimeDriver;
impl Driver for RuntimeTimeDriver {
    fn now(&self) -> u64 {
        #[cfg(feature = "embassy-only")]
        {
            standalone::now()
        }
        #[cfg(not(feature = "embassy-only"))]
        {
            crate::runtime::time::embassy_now_ticks()
        }
    }
    fn schedule_wake(&self, at: u64, waker: &Waker) {
        #[cfg(feature = "embassy-only")]
        standalone::schedule_wake(at, waker);
        #[cfg(not(feature = "embassy-only"))]
        crate::runtime::time::embassy_schedule_wake(at, waker);
    }
}
embassy_time_driver::time_driver_impl!(static EMBASSY_TIME_DRIVER: RuntimeTimeDriver = RuntimeTimeDriver);

/// Number of retained task wakers; cancellation may leave bounded stale entries.
pub fn pending_wakers() -> usize {
    #[cfg(feature = "embassy-only")]
    {
        standalone::status().pending_wakers
    }
    #[cfg(not(feature = "embassy-only"))]
    {
        crate::runtime::time::embassy_pending_wakers()
    }
}
