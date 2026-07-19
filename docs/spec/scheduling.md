# hisi-rtos Single-Hart Scheduling Contract

This document is the normative scheduling contract for `hisi-rtos`. Requirement
IDs are stable across implementation changes and are mapped to executable
evidence in `requirements.toml`.

## Identity And Ownership

- **RTOS-ID-001:** A task identity is `{ slot, generation }`. Reusing a slot
  increments its generation; an identity from an earlier generation is invalid.
- **RTOS-ID-002:** The current `TaskId` ABI stores the slot index in its low 8
  bits. The scheduler table must fit that encoding at compile time, and encoding
  rejects out-of-table slots and generation zero. Supporting more than 256 total
  slots requires a versioned identity ABI; an implementation must not truncate.
- **RTOS-STATE-001:** A task is exactly one of `Free`, `Ready`, `Running`,
  `Blocked`, `Sleeping`, or `Throttled`, and belongs to at most one
  ready/wait/throttle ownership set.
- **RTOS-STATE-002:** On a single hart, at most one task is `Running`.
- **RTOS-STATE-003:** Slot 0 adopts the caller and slot 1 is an always-eligible,
  lowest-priority idle fallback. It does not enter the ordinary ready queues or
  participate in FIFO/time-slice policy. A voluntary idle handoff leaves it
  eligible without queueing it. Dynamic task allocation cannot consume either
  reserved slot, so budget exhaustion never runs a throttled task. The
  current compatibility profile exposes 15 dynamic slots in addition to these
  two internal slots. The first 15 dynamic allocations succeed when storage is
  otherwise available; the next returns `NoTaskSlots`.

## Priority And Run Policy

The scheduler has one backend. Effective priority chooses the next eligible
task; a per-thread `RunPolicy` controls when the current task may be forcibly
switched.

- **RTOS-SCHED-001:** Lower numeric effective priority wins for every run policy.
  Equal-priority ready tasks are FIFO. A voluntary yield selects another eligible
  task before requeueing the yielding task at its priority tail.
- **RTOS-SCHED-002:** `Cooperative` permits switches only at explicit
  yield/block/sleep/exit or another explicit scheduler handoff. An IRQ wake or
  higher-priority ready task records work but does not immediately preempt the
  current cooperative user task. Idle is the fallback exception: the outermost
  IRQ epilogue immediately replaces a running idle task when ordinary work is
  ready. `Cooperative` has no equal-priority time slice.
- **RTOS-SCHED-003:** `Preemptive { time_slice }` permits a higher-priority ready
  task to preempt and equal-priority round-robin after that non-zero slice expires.
- **RTOS-SCHED-004:** `Budgeted(spec)` has cooperative switching semantics and a
  periodic CPU quota defined below.
- **RTOS-SCHED-005:** Changing a throttled task's policy starts a new policy
  lifetime. The old replenishment deadline is discarded, the task becomes
  `Ready`, and a newly selected Budgeted policy starts with a full budget.

## Periodic Budget

`BudgetSpec { capacity, replenishment_period }` is valid only when both values
are non-zero and `capacity <= replenishment_period`.

- **RTOS-BUDGET-001:** A budgeted task may consume at most `capacity` between
  phase-aligned replenishment boundaries, except for bounded scheduler-lock and
  interrupt latency described below.
- **RTOS-BUDGET-002:** Yield, block, and sleep preserve unused budget. They do not
  refill it.
- **RTOS-BUDGET-003:** At exhaustion, the task becomes `Throttled` and is removed
  from the eligible set until its next replenishment boundary. Idle CPU borrowing
  is not permitted.
- **RTOS-BUDGET-004:** Replenishment preserves the original phase. Missing one or
  more periods advances directly to the first boundary after `now` and restores
  exactly `capacity`.
- **RTOS-BUDGET-005:** The initial implementation charges dispatch-to-switch wall
  time, conservatively including interrupt latency. It must never under-account
  CPU use.
- **RTOS-BUDGET-006:** If capacity expires while scheduler lock depth is non-zero,
  exhaustion becomes pending. The outermost unlock must throttle and switch. If
  the lock crossed replenishment boundaries, eligibility is delayed to the first
  boundary strictly after unlock, so the lock cannot erase exhaustion.
- **RTOS-BUDGET-007:** The quota is an upper bound, not a minimum-service or CPU
  reservation guarantee. Priority, other eligible work, interrupt load, and
  cooperative handoff may prevent a task from consuming its full capacity.

## Scheduler Lock And Interrupts

- **RTOS-LOCK-001:** Scheduler lock is per-current-task, nested, and does not mask
  hardware interrupts. IRQ handlers may acknowledge, record, and wake tasks.
- **RTOS-LOCK-002:** No switch occurs while lock depth or runtime IRQ depth is
  non-zero. Pending preemption is handled after the outermost unlock/IRQ exit.
- **RTOS-LOCK-003:** Blocking, sleeping, or exiting while holding scheduler lock
  is invalid. Fallible runtime calls return `InvalidContext`; a task entry that
  returns while locked is a fail-stop contract violation.
- **RTOS-LOCK-004:** A ported runtime has a non-zero maximum continuous scheduler
  lock duration. The lock deadline shares the scheduler timer. Expiry invokes the
  port's non-returning contract-violation handler outside the scheduler critical
  section; the kernel must not resume a task after an unbounded lock overrun.
- **RTOS-IRQ-001:** User callbacks never run in an ISR, critical section, or
  scheduler lock. ISR work is acknowledge/record/wake only.

## Per-Thread Observability

- **RTOS-OBS-001:** Each task exposes saturating cumulative dispatch-to-switch
  wall time, dispatch count, longest continuous run, and longest ready-to-dispatch
  latency. A read-only snapshot includes the current running interval without
  mutating scheduler state.
- **RTOS-OBS-002:** CPU accounting conservatively includes interrupt handling
  while the task is the interrupted context. A separate cumulative outermost IRQ
  time, entry count, and longest IRQ span are exposed so diagnostics can estimate
  thread-mode time without pretending IRQ work belongs to another task.
- **RTOS-OBS-003:** Scheduler-lock diagnostics count only outermost lock
  acquisitions and expose the longest outermost hold interval, including an
  in-progress lock in a read-only snapshot.
- **RTOS-OBS-004:** Budget exhaustion is counted per task as well as globally.
  Saturating counters must not wrap and silently erase prior evidence.
- **RTOS-OBS-005:** Hot-path accounting performs bounded in-memory updates only.
  It must not format output, invoke user callbacks, or perform UART/MMIO logging
  while holding the scheduler critical section.

## Waits And Priority Inheritance

- **RTOS-WAIT-001:** Signal, timeout, interrupt wake, and cancellation compete at
  one scheduler-serialized linearization point; one wake reason wins.
- **RTOS-WAIT-002:** A semaphore grant is direct and cannot be stolen by a third
  task before the selected waiter runs. The selected waiter has the highest
  effective priority; waiters at the same priority remain FIFO.
- **RTOS-WAIT-003:** Destroying a semaphore with waiters, or a mutex with an owner
  or waiters, fails closed with `InvalidContext`. The contract remains unsafe:
  callers must still exclude concurrent and future use. Contract v1 does not
  promise detection of a stale or duplicate opaque resource handle.
- **RTOS-MUTEX-001:** Recursive mutex unlock is owner-only. Final unlock directly
  hands ownership to the highest-priority FIFO waiter.
- **RTOS-MUTEX-002:** Effective priority is the minimum numeric value of base
  priority and active donations. Donation propagates transitively and is removed
  on timeout, handoff, or final unlock.

## Timer And Mechanism

- **RTOS-PORT-001:** The port-less safe Rust API is cooperative-only. Its config
  cannot represent Budgeted or Preemptive policy, and its runtime handle exposes
  no policy-mutation capability. IRQ wake does not immediately switch tasks and
  deadlines are observed only when the scheduler next regains control.
- **RTOS-PORT-002:** Budgeted and Preemptive configuration and policy mutation
  require a `Ported` runtime handle created by `start_with_port`. Unsafe/FFI
  entry points retain runtime validation, but an invalid safe Rust call must fail
  to compile.
- **RTOS-PORT-003:** A ported thread-mode operation that must synchronously switch
  away and later resume requires machine interrupts enabled before it mutates
  scheduler ownership. Otherwise it returns `InvalidContext`; an infallible
  task-exit path fails stop. Non-blocking wake/signal operations may update state
  with MIE cleared and pend deferred rescheduling. ISR wakeups remain valid
  because the current trap epilogue delivers the deferred switch.
- **RTOS-PORT-004:** If an interrupt completes a task's pending handoff after
  thread mode detached a target but before it issues the explicit switch request,
  the resumed task must cancel that stale request and restore the detached target
  to its original ready queue. It must not create multiple `Running` tasks or
  strand an eligible task outside the ready queues.

- **RTOS-TIMER-001:** Sleep, wait timeout, preemptive slice, budget exhaustion,
  budget replenishment, and Embassy deadlines share the earliest one-shot timer.
- **RTOS-TIMER-002:** After a trap selects a different task, the timer is rearmed
  from the new current task's policy; the old task's deadline cannot leak across
  a context switch.
- **RTOS-TIMER-003:** Timer programming uses a generation ticket. If a nested
  rearm occurs while an older call performs MMIO, the older call must detect its
  stale ticket, recompute all RTOS and Embassy deadlines from one scheduler
  snapshot, and program again. A stale later deadline cannot be the final
  hardware state.
- **RTOS-CONTEXT-001:** WS63 task creation, cooperative switch, and interrupt
  switch use one 272-byte context ABI. Interrupts preserve full GPR/FPR/FCSR;
  cooperative calls may populate only ABI-required callee-saved slots; all
  restores finish with `mret`.

## Non-Goals And Future Extensions

This contract is single-hart. It does not claim that disabling interrupts is
cross-hart exclusion. SMP adds hart identity, affinity, IPI, memory ordering, and
cross-hart accounting without redefining these single-hart requirements.
