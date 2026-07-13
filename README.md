# hisi-rtos

`no_std` scheduler and runtime services for HiSilicon embedded Rust firmware.
Applications inject allocation and monotonic-time resources, then start exactly
one runtime before initializing radio firmware.

The initial backend preserves the connectivity-proven single-hart cooperative
scheduler while ownership moves out of the RF adapter. Priority preemption,
deferred stack reclamation and Embassy integration remain planned work and are
not claimed by the alpha release.

