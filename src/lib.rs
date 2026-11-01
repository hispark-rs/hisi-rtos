//! Task scheduler and radio runtime backend.
//!
//! `hisi-rtos` maintains one single-hart scheduler. Thread priority selects the
//! next eligible task, while `RunPolicy` controls when the running task may be
//! forcibly switched. Target-backed operation uses one trap-frame/mret path;
//! the explicitly port-less profile is cooperative-only.
//!
//! With the `embassy` feature, this crate also owns the process-wide
//! `embassy-time` driver so native threads, vendor workers, and Embassy futures
//! share one timer contract. With `embassy-only,unstable,chip-*`, only the
//! standalone time profile is compiled; scheduler startup is unavailable.

#![no_std]
#[cfg(test)]
extern crate std;
#[cfg(all(feature = "embassy-only", not(feature = "unstable")))]
compile_error!("embassy-only requires unstable");
#[cfg(all(
    any(feature = "chip-ws53", feature = "chip-bs21"),
    not(feature = "embassy-only")
))]
compile_error!(
    "WS53/BS2X currently require the embassy-only profile; no scheduler port is implemented"
);
/// Embassy integration: one selected profile, one queue and one time provider.
#[cfg(feature = "embassy")]
pub mod embassy;

#[cfg(not(feature = "embassy-only"))]
mod config;
#[cfg(not(feature = "embassy-only"))]
mod context;
#[cfg(not(feature = "embassy-only"))]
mod runtime;
#[cfg(not(feature = "embassy-only"))]
mod scheduling;
#[cfg(not(feature = "embassy-only"))]
mod storage;
#[cfg(all(feature = "chip-ws63", not(feature = "embassy-only")))]
pub mod ws63;

#[cfg(not(feature = "embassy-only"))]
pub use config::*;
#[cfg(not(feature = "embassy-only"))]
pub use hisi_rf_rtos_driver::TaskId;
#[cfg(not(feature = "embassy-only"))]
pub use runtime::*;
#[cfg(not(feature = "embassy-only"))]
pub use scheduling::{BudgetSpec, RunPolicy};
#[allow(deprecated)]
#[cfg(not(feature = "embassy-only"))]
pub use storage::SchedulerStackArena;
#[cfg(not(feature = "embassy-only"))]
pub use storage::{InstalledSchedulerStorage, SchedulerArena, SchedulerStorage, StorageError};
