# hisi-rtos

`no_std` scheduler and runtime services for HiSilicon embedded Rust firmware.
Applications inject allocation and monotonic-time resources, then start exactly
one runtime before initializing radio firmware.

The crate maintains one single-hart scheduler backend. Each thread chooses
`RunPolicy::Cooperative`, `RunPolicy::Budgeted`, or `RunPolicy::Preemptive`.
All ready tasks use effective numeric priority and FIFO within one priority;
`RunPolicy` only controls when the currently running task may be forcibly
switched. Preemptive tasks optionally use equal-priority time slicing. Budgeted
tasks use a periodic CPU quota and become ineligible until replenishment after
exhaustion.
An internal lowest-priority idle thread remains eligible while application and
vendor threads are sleeping or budget-throttled; it cannot be allocated or
reconfigured through the public runtime contract.
TIMER and software interrupts drive deferred preemption through the runtime's
unified 272-byte task/trap frame. Interrupt handlers acknowledge, record, and
wake; the common trap epilogue selects the next task, rearms the selected task's
deadline, and restores it with `mret`.

`CooperativeConfig` cannot express a preemptive policy. `PortedConfig` is accepted
only by `start_with_port`, whose returned capability is required for policy
changes. A firmware that hosts vendor workers selects an explicit budget;
ordinary Rust/Embassy execution remains cooperative unless deliberately changed.
Exited stacks are reclaimed by another task,
and nested scheduler locks suppress preemption until the outermost unlock.
The configured scheduler-lock deadline is a fail-stop contract: a target port
must report/halt or reset rather than resume a task that exceeded the bound.
Recursive mutexes use priority-ordered waiters, direct handoff, timeout cleanup,
and transitive priority inheritance. Enable `embassy` to make this crate the
firmware's `embassy-time` driver. The driver uses the injected millisecond clock
at 1 ms resolution while preserving the ecosystem-wide
`embassy-time/tick-hz-1_000_000` ABI. RTOS sleep/time-
slice and Embassy deadlines share the same `SchedulerPort` timer; HAL must not
install a second time driver in the same firmware. Peripheral async traits stay
in `hisi-hal`.

Vendor LiteOS is a behavior and disassembly oracle for the WS63 blob ABI, not a
backend or dependency of this crate. `hisi-rtos` is the sole maintained native
runtime; the WS63 compatibility adapter maps only the symbols actually required
by a versioned radio archive onto `hisi-rf-rtos-driver` capabilities.

The normative scheduler contract and machine-readable evidence map live in
[`docs/spec/scheduling.md`](docs/spec/scheduling.md) and
[`docs/spec/requirements.toml`](docs/spec/requirements.toml). The periodic
CPU-quota model is executable with TLC from `spec/SchedulerBudget.tla`.
