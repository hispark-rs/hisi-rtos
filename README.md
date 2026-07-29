# hisi-rtos

`no_std` scheduler and runtime services for HiSilicon embedded Rust firmware.
Applications install caller-owned task-stack storage and start exactly one
runtime before initializing radio firmware.

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

## WS63 startup

Enable `chip-ws63` to let the RTOS own the WS63 scheduler timer and deferred
software interrupt. The application declares both the dynamic task quota and
the stack arena; no allocator callback is part of the WS63 happy path:

```rust,ignore
hisi_rtos::bind_interrupts!(struct RtosIrqs {
    TIMER_INT0 => hisi_rtos::ws63::TimerInterrupt;
    SOFT_INT0 => hisi_rtos::ws63::SoftwareInterrupt;
});

static STORAGE: hisi_rtos::SchedulerStorage<15> =
    hisi_rtos::SchedulerStorage::new();
static STACKS: hisi_rtos::SchedulerStackArena<{ 7 * 24 * 1024 + 512 }> =
    hisi_rtos::SchedulerStackArena::new();

let storage = STORAGE.install(&STACKS)?;
let runtime = hisi_rtos::ws63::start(
    hisi_rtos::ws63::Config::default(),
    hisi_rtos::ws63::Resources {
        timer: peripherals.TIMER,
        software_interrupt: peripherals.SYS_CTL1,
        storage,
        contract_violation,
        irqs: RtosIrqs::new(),
    },
)?;
```

This is the normal WS63 target path. It consumes both peripheral singletons,
installs TIMER_INT0 and SOFT_INT0, supplies the 24 MHz monotonic clock, enables
global interrupts after scheduler installation, and routes Cooperative,
Budgeted, and Preemptive switching through the shared trap-frame/`mret` path.
The lower-level `start_with_port` API remains available for new chip ports and
conformance fixtures.

Vendor LiteOS is a behavior and disassembly oracle for the WS63 blob ABI, not a
backend or dependency of this crate. `hisi-rtos` is the sole maintained native
runtime; the WS63 compatibility adapter maps only the symbols actually required
by a versioned radio archive onto `hisi-rf-rtos-driver` capabilities.
Contract v1.3 retains the advisory 15-slot capacity snapshot and adds atomic,
owner-bound reservations. The adopted main and internal idle tasks do not
consume that quota. Ordinary task creation cannot consume slots promised to a
live reservation; reserved spawns consume exactly one promised slot, and stale,
released, or exhausted generation-bearing tokens fail closed.

The normative scheduler contract and machine-readable evidence map live in
[`docs/spec/scheduling.md`](docs/spec/scheduling.md) and
[`docs/spec/requirements.toml`](docs/spec/requirements.toml). Executable TLA+
models under `spec/` cover periodic CPU quota, resource/wait lifecycle, wait
linearization, bounded priority inheritance, timer re-arm generation, and
switch-intent ownership. CI validates every implementation, host-test, TLA+,
Kani, and HIL-marker reference, then publishes a JSON requirement inventory and
the complete TLA+ state-space logs as build artifacts. A requirement carrying a
HIL marker remains `hil-required`; software evidence never silently graduates it.
