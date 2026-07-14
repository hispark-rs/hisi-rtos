use core::num::NonZeroU32;

/// Periodic CPU quota used by [`RunPolicy::Budgeted`].
///
/// `capacity` is the maximum wall-clock runtime available in each
/// `replenishment_period`. The first implementation conservatively charges IRQ
/// latency to the interrupted thread so accounting never understates CPU use.
/// This is an upper bound, not a guarantee that the scheduler can supply that
/// amount of CPU time in every period.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct BudgetSpec {
    capacity: NonZeroU32,
    replenishment_period: NonZeroU32,
}

impl BudgetSpec {
    /// Creates a quota when the capacity fits within its period.
    pub const fn try_new(capacity: NonZeroU32, replenishment_period: NonZeroU32) -> Option<Self> {
        if capacity.get() <= replenishment_period.get() {
            Some(Self {
                capacity,
                replenishment_period,
            })
        } else {
            None
        }
    }

    /// Maximum runtime available in one replenishment period.
    pub const fn capacity(self) -> NonZeroU32 {
        self.capacity
    }

    /// Length of one replenishment period.
    pub const fn replenishment_period(self) -> NonZeroU32 {
        self.replenishment_period
    }
}

/// Per-thread scheduling policy used by the unified scheduler backend.
#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
pub enum RunPolicy {
    /// Runs until it yields, blocks, or exits; ready tasks retain FIFO order.
    #[default]
    Cooperative,
    /// Cooperative execution bounded by a periodic CPU quota.
    Budgeted(BudgetSpec),
    /// Allows equal-priority round-robin preemption at the given time slice.
    Preemptive { time_slice: NonZeroU32 },
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) enum BudgetExpiry {
    NotBudgeted,
    StillAvailable,
    ThrottleNow,
    DeferredBySchedulerLock,
}

/// Pure periodic-quota state used directly by the production scheduler.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(crate) struct BudgetState {
    spec: Option<BudgetSpec>,
    remaining: u32,
    replenishes_at: u64,
    running_since: Option<u64>,
    lock_overrun_pending: bool,
}

impl BudgetState {
    pub(crate) const fn none() -> Self {
        Self {
            spec: None,
            remaining: 0,
            replenishes_at: 0,
            running_since: None,
            lock_overrun_pending: false,
        }
    }

    pub(crate) fn for_policy(policy: RunPolicy, now: u64) -> Self {
        let RunPolicy::Budgeted(spec) = policy else {
            return Self::none();
        };
        Self {
            spec: Some(spec),
            remaining: spec.capacity().get(),
            replenishes_at: now.saturating_add(spec.replenishment_period().get() as u64),
            running_since: None,
            lock_overrun_pending: false,
        }
    }

    pub(crate) fn remaining(&self) -> u32 {
        self.remaining
    }

    pub(crate) fn replenishes_at(&self) -> u64 {
        self.replenishes_at
    }

    pub(crate) fn lock_overrun_pending(&self) -> bool {
        self.lock_overrun_pending
    }

    pub(crate) fn on_dispatch(&mut self, now: u64) {
        self.replenish_if_due(now);
        if self.spec.is_some() {
            self.running_since = Some(now);
        }
    }

    pub(crate) fn on_switch_out(&mut self, now: u64) {
        self.charge_until(now);
        self.running_since = None;
    }

    pub(crate) fn exhaustion_deadline(&self) -> Option<u64> {
        let started = self.running_since?;
        (self.remaining != 0).then(|| started.saturating_add(self.remaining as u64))
    }

    pub(crate) fn on_timer(&mut self, now: u64, scheduler_locked: bool) -> BudgetExpiry {
        if self.spec.is_none() {
            return BudgetExpiry::NotBudgeted;
        }
        self.charge_until(now);
        if self.remaining != 0 {
            return BudgetExpiry::StillAvailable;
        }
        self.running_since = None;
        if scheduler_locked {
            self.lock_overrun_pending = true;
            BudgetExpiry::DeferredBySchedulerLock
        } else {
            BudgetExpiry::ThrottleNow
        }
    }

    /// Applies a lock-deferred exhaustion and returns the next eligible time.
    pub(crate) fn throttle_after_lock_overrun(&mut self, now: u64) -> Option<u64> {
        if !self.lock_overrun_pending {
            return None;
        }
        let spec = self.spec?;
        self.lock_overrun_pending = false;
        self.remaining = 0;
        self.running_since = None;

        let period = spec.replenishment_period().get() as u64;
        while self.replenishes_at <= now {
            let next = self.replenishes_at.saturating_add(period);
            if next == self.replenishes_at {
                self.replenishes_at = u64::MAX;
                break;
            }
            self.replenishes_at = next;
        }
        Some(self.replenishes_at)
    }

    /// Replenishes a non-running task while preserving the original phase.
    pub(crate) fn replenish_if_due(&mut self, now: u64) -> bool {
        let Some(spec) = self.spec else {
            return false;
        };
        if self.lock_overrun_pending || now < self.replenishes_at {
            return false;
        }

        let period = spec.replenishment_period().get() as u64;
        let elapsed_periods = (now - self.replenishes_at) / period + 1;
        self.replenishes_at = self
            .replenishes_at
            .saturating_add(elapsed_periods.saturating_mul(period));
        self.remaining = spec.capacity().get();
        true
    }

    fn charge_until(&mut self, now: u64) {
        let Some(started) = self.running_since else {
            return;
        };
        let elapsed = now.saturating_sub(started).min(u32::MAX as u64) as u32;
        self.remaining = self.remaining.saturating_sub(elapsed);
        self.running_since = Some(now);
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn budget(capacity: u32, period: u32) -> BudgetSpec {
        BudgetSpec::try_new(
            NonZeroU32::new(capacity).unwrap(),
            NonZeroU32::new(period).unwrap(),
        )
        .unwrap()
    }

    #[test]
    fn rejects_capacity_larger_than_period() {
        assert!(
            BudgetSpec::try_new(NonZeroU32::new(11).unwrap(), NonZeroU32::new(10).unwrap())
                .is_none()
        );
    }

    #[test]
    fn switch_out_preserves_unused_budget() {
        let mut state = BudgetState::for_policy(RunPolicy::Budgeted(budget(5, 20)), 100);
        state.on_dispatch(102);
        state.on_switch_out(104);
        assert_eq!(state.remaining(), 3);
        assert_eq!(state.replenishes_at(), 120);
    }

    #[test]
    fn exhausted_budget_throttles_until_phase_aligned_replenishment() {
        let mut state = BudgetState::for_policy(RunPolicy::Budgeted(budget(5, 20)), 100);
        state.on_dispatch(100);
        assert_eq!(state.on_timer(105, false), BudgetExpiry::ThrottleNow);
        assert_eq!(state.remaining(), 0);
        assert!(!state.replenish_if_due(119));
        assert!(state.replenish_if_due(120));
        assert_eq!(state.remaining(), 5);
        assert_eq!(state.replenishes_at(), 140);
    }

    #[test]
    fn scheduler_lock_cannot_erase_an_exhaustion_by_crossing_periods() {
        let mut state = BudgetState::for_policy(RunPolicy::Budgeted(budget(5, 20)), 100);
        state.on_dispatch(100);
        assert_eq!(
            state.on_timer(105, true),
            BudgetExpiry::DeferredBySchedulerLock
        );
        assert!(!state.replenish_if_due(145));
        assert_eq!(state.throttle_after_lock_overrun(145), Some(160));
        assert_eq!(state.remaining(), 0);
        assert!(!state.replenish_if_due(159));
        assert!(state.replenish_if_due(160));
    }

    #[test]
    fn long_inactivity_advances_phase_without_looping_each_period() {
        let mut state = BudgetState::for_policy(RunPolicy::Budgeted(budget(5, 20)), 100);
        assert!(state.replenish_if_due(1_000));
        assert_eq!(state.remaining(), 5);
        assert_eq!(state.replenishes_at(), 1_020);
    }
}

#[cfg(kani)]
mod proofs {
    use super::*;

    #[kani::proof]
    fn remaining_never_exceeds_capacity() {
        let capacity: u32 = kani::any();
        let period: u32 = kani::any();
        kani::assume(capacity > 0 && period >= capacity);
        let spec = BudgetSpec::try_new(
            NonZeroU32::new(capacity).unwrap(),
            NonZeroU32::new(period).unwrap(),
        )
        .unwrap();
        let mut state = BudgetState::for_policy(RunPolicy::Budgeted(spec), 0);
        let dispatch: u64 = kani::any();
        let switch_out: u64 = kani::any();
        kani::assume(switch_out >= dispatch);
        state.on_dispatch(dispatch);
        state.on_switch_out(switch_out);
        assert!(state.remaining() <= capacity);
    }
}
