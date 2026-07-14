//! Task scheduler and radio runtime backend.
//!
//! `hisi-rtos` maintains one single-hart scheduler. Thread priority selects the
//! next eligible task, while [`RunPolicy`] controls when the running task may be
//! forcibly switched. Target-backed operation uses one trap-frame/mret path;
//! the explicitly port-less profile is cooperative-only.
//!
//! With the `embassy` feature, this crate also owns the process-wide
//! `embassy-time` driver so native threads, vendor workers, and Embassy futures
//! share one timer contract.

#![no_std]

mod config;
mod context;
mod runtime;
mod scheduling;

pub use config::*;
pub use hisi_rf_rtos_driver::TaskId;
pub use runtime::*;
pub use scheduling::{BudgetSpec, RunPolicy};
