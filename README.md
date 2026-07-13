# hisi-rtos

`no_std` scheduler and runtime services for HiSilicon embedded Rust firmware.
Applications inject allocation and monotonic-time resources, then start exactly
one runtime before initializing radio firmware.

The initial backend preserves the connectivity-proven single-hart cooperative
scheduler while ownership moves out of the RF adapter. Exited stacks are
reclaimed from another task, and task priorities are recorded through the
runtime-neutral driver contract. `SchedulingPolicy::Cooperative` remains the
default proven RF policy; priority selection is explicit and does not claim
timer-driven preemption. TIMER/software-interrupt preemption and Embassy
integration remain planned work.
