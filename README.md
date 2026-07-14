# hisi-rtos

`no_std` scheduler and runtime services for HiSilicon embedded Rust firmware.
Applications inject allocation and monotonic-time resources, then start exactly
one runtime before initializing radio firmware.

The single-hart backend supports cooperative and priority scheduling. Under
`SchedulingPolicy::Priority`, an IRQ that wakes a higher-priority task defers
the context switch until the runtime has restored the interrupted task's stack;
the switch never occurs on the shared IRQ stack. The WS63 connectivity image
exercises this path through init, scan, WPA2 association, DHCP, ARP, and ping.

`Config::default()` remains cooperative for compatibility, so priority behavior
is an explicit application choice. Exited stacks are reclaimed by another task,
and nested scheduler locks suppress preemption until the outermost unlock.
TIMER/software-interrupt time slicing, priority inheritance, and Embassy
integration remain planned work.
