//! Shared integer time conversion; hardware scheduling remains backend-specific.

use core::num::NonZeroU32;

/// Convert an integer-ratio hardware counter to time ticks without overflow.
pub(crate) const fn counter_ticks(count: u64, counts_per_tick: NonZeroU32) -> u64 {
    count / counts_per_tick.get() as u64
}

/// Compute one nonzero timer segment, preserving the absolute deadline at the
/// caller. A deadline farther away than the timer's width is segmented.
pub(crate) const fn alarm_counts(at: u64, now: u64, counts_per_tick: NonZeroU32) -> u32 {
    let delta = at.saturating_sub(now);
    let rate = counts_per_tick.get() as u64;
    if delta == 0 {
        1
    } else if delta > u32::MAX as u64 / rate {
        u32::MAX
    } else {
        (delta * rate) as u32
    }
}

#[cfg(all(test, not(target_arch = "riscv32")))]
mod tests {
    use super::*;
    use proptest::prelude::*;

    proptest! {
        #[test]
        fn counter_and_alarm_match_wide_reference(count in any::<u64>(), at in any::<u64>(), now in any::<u64>(), rate in 1u32..=u32::MAX) {
            let rate = NonZeroU32::new(rate).unwrap();
            prop_assert_eq!(counter_ticks(count, rate), (u128::from(count) / u128::from(rate.get())) as u64);
            let reference = (u128::from(at.saturating_sub(now)) * u128::from(rate.get())).clamp(1, u128::from(u32::MAX));
            prop_assert_eq!(alarm_counts(at, now, rate), reference as u32);
        }
    }

    #[test]
    fn large_uptime_and_distant_deadline_do_not_overflow() {
        for rate in [24, 32] {
            let rate = NonZeroU32::new(rate).unwrap();
            let before = u64::MAX / 1_000_000;
            assert_eq!(
                counter_ticks(before + 1, rate),
                (before + 1) / u64::from(rate.get())
            );
            assert_eq!(
                counter_ticks(u64::MAX, rate),
                u64::MAX / u64::from(rate.get())
            );
            assert_eq!(alarm_counts(u64::MAX - 1, 0, rate), u32::MAX);
            let bound = u64::from(u32::MAX) / u64::from(rate.get());
            assert_eq!(
                alarm_counts(bound, 0, rate),
                (bound * u64::from(rate.get())) as u32
            );
            assert_eq!(alarm_counts(bound + 1, 0, rate), u32::MAX);
        }
    }
}
