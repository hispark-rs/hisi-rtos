# hisi-rtos

`no_std` scheduler and runtime services for HiSilicon embedded Rust firmware.
Applications inject allocation and monotonic-time resources, then start exactly
one runtime before initializing radio firmware.

The single-hart backend supports cooperative and priority scheduling. Under
`SchedulingPolicy::Priority`, TIMER and software interrupts drive deferred
preemption through the runtime's unified 272-byte task/trap frame. Interrupt
handlers acknowledge, record, and wake; the common trap epilogue selects the
next task and restores it with `mret`.

`Config::default()` remains cooperative for compatibility, so priority behavior
is an explicit application choice. Exited stacks are reclaimed by another task,
and nested scheduler locks suppress preemption until the outermost unlock.
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
