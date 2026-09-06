//! Commutation-ripple tachometer: motor-shaft speed from the TEL current
//! series alone. A brushed 3-slot motor commutates 6 times per rotor rev,
//! stamping a ~kHz ripple on the winding current - well above the pot's
//! bandwidth but comfortably under the 20 kHz tick rate, so a constant-duty
//! sweep's per-tick current carries a direct, gear-slip-immune speed
//! signal.
//!
//! Method: detrend with a sliding mean (kills DC and the slow bemf
//! collapse), then find the dominant period by normalized autocorrelation
//! over the lag range of interest, with parabolic interpolation around the
//! peak lag for sub-sample resolution. At 20.1 kHz and 0.5-4 kHz ripple
//! the raw lag grid is 5-40 samples, so interpolation is what buys the
//! ~1-2% frequency resolution; the estimate degrades gracefully (low
//! `strength`) instead of lying when no periodicity stands out.

/// One ripple-frequency estimate over a current window.
#[derive(Copy, Clone, Debug)]
pub struct RippleEstimate {
    /// Dominant ripple frequency, Hz.
    pub freq_hz: f64,
    /// Motor shaft speed, rev/s (= freq / ripple_per_rev).
    pub motor_rev_s: f64,
    /// Normalized autocorrelation at the peak (0..1); below ~0.15 the
    /// caller should not trust the estimate (ripple_speed returns None
    /// there already).
    pub strength: f64,
}

/// Lowest ripple frequency the autocorr band tracks; also fixes the max lag
/// (fs/F_LO samples) and thus ripple_speed's minimum input length.
const F_LO: f64 = 500.0;

/// Minimum sample count ripple_speed needs at `fs`: 4 periods of the slowest
/// tracked ripple. cumulative_phase in [`crate::lut`] sizes its window off it.
pub fn min_window(fs: f64) -> usize {
    4 * (fs / F_LO).round() as usize
}

/// Estimate ripple frequency in `i` sampled at `fs` Hz. `ripple_per_rev`
/// = commutation events per rotor rev (6 for a 3-slot brushed motor).
/// Searches 500 Hz..fs/5; None when the series is too short, flat, or no
/// autocorrelation peak clears the confidence floor.
pub fn ripple_speed(i: &[f64], fs: f64, ripple_per_rev: f64) -> Option<RippleEstimate> {
    const MIN_STRENGTH: f64 = 0.15;
    // band = F_LO .. fs/5: below 5 samples per period the parabolic
    // interpolation has nothing to stand on
    let lag_min = 5usize;
    let lag_max = (fs / F_LO).round() as usize;
    if i.len() < 4 * lag_max || fs <= 0.0 || ripple_per_rev <= 0.0 {
        return None;
    }
    let d = detrend(i, lag_max);
    let e: f64 = d.iter().map(|x| x * x).sum();
    if e <= 0.0 || !e.is_finite() {
        return None;
    }
    // normalized autocorrelation over the lag band
    let ac: Vec<f64> = (lag_min..=lag_max)
        .map(|lag| {
            let s: f64 = d[..d.len() - lag]
                .iter()
                .zip(&d[lag..])
                .map(|(a, b)| a * b)
                .sum();
            s / e
        })
        .collect();
    let k = ac
        .iter()
        .enumerate()
        .max_by(|a, b| a.1.total_cmp(b.1))
        .map(|(k, _)| k)?;
    // Reject a peak at either band edge: at lag_min it is short-range sample
    // correlation (the acceleration/reversal transient locks here), at lag_max
    // it is the detrend tail - neither is a real ripple period, and only an
    // interior peak has the two neighbors parabolic interpolation needs.
    if k == 0 || k + 1 == ac.len() {
        return None;
    }
    let strength = ac[k];
    if strength < MIN_STRENGTH {
        return None;
    }
    // parabolic interpolation around the peak lag (guaranteed interior above)
    let lag = (lag_min + k) as f64 + {
        let (a, b, c) = (ac[k - 1], ac[k], ac[k + 1]);
        let den = a - 2.0 * b + c;
        if den.abs() > 1e-12 {
            (0.5 * (a - c) / den).clamp(-0.5, 0.5)
        } else {
            0.0
        }
    };
    let freq_hz = fs / lag;
    Some(RippleEstimate {
        freq_hz,
        motor_rev_s: freq_hz / ripple_per_rev,
        strength,
    })
}

/// Gear-ratio cross-check: motor rev/s over output rev/s, with the output
/// speed taken as pot slope / `counts_per_output_rev`.
///
/// CAVEAT: the pot's 4096 counts span its electrical travel (~300 deg),
/// not a full output revolution, so with the default 4096.0 this is a
/// RELATIVE ratio - constant across rungs on a healthy train (its rung-to-
/// rung stability is the Ke-slip check), but not the physical gear ratio
/// until the pot's degrees-per-count is calibrated.
pub fn gear_ratio_check(motor_rev_s: f64, pot_omega_cps: f64, counts_per_output_rev: f64) -> f64 {
    motor_rev_s / (pot_omega_cps / counts_per_output_rev)
}

/// Subtract a centered sliding mean of width ~2*half+1 (clamped at the
/// edges); wide enough to pass the ripple band untouched.
fn detrend(v: &[f64], half: usize) -> Vec<f64> {
    let n = v.len();
    let mut out = Vec::with_capacity(n);
    // prefix sums make each window mean O(1)
    let mut pre = Vec::with_capacity(n + 1);
    pre.push(0.0);
    for &x in v {
        pre.push(pre.last().unwrap() + x);
    }
    for (k, &x) in v.iter().enumerate() {
        let lo = k.saturating_sub(half);
        let hi = (k + half + 1).min(n);
        let m = (pre[hi] - pre[lo]) / (hi - lo) as f64;
        out.push(x - m);
    }
    out
}

#[cfg(test)]
mod tests {
    use super::*;

    struct Lcg(u64);
    impl Lcg {
        fn next(&mut self, half: f64) -> f64 {
            self.0 = self
                .0
                .wrapping_mul(6364136223846793005)
                .wrapping_add(1442695040888963407);
            ((self.0 >> 11) as f64 / (1u64 << 53) as f64 * 2.0 - 1.0) * half
        }
    }

    const FS: f64 = 20_100.0;

    fn ripple_series(freq: f64, amp: f64, noise: f64, drift: f64, n: usize) -> Vec<f64> {
        let mut lcg = Lcg(41);
        (0..n)
            .map(|k| {
                let t = k as f64 / FS;
                60.0 + drift * t
                    + amp * (2.0 * std::f64::consts::PI * freq * t).sin()
                    + lcg.next(noise)
            })
            .collect()
    }

    #[test]
    fn recovers_synthetic_ripple_frequency() {
        // 1.8 kHz ripple, amp 4 counts, noise +-3 counts, bemf-collapse
        // drift: 2% recovery despite the 11-sample raw lag grid
        let i = ripple_series(1800.0, 4.0, 3.0, -40.0, 4000);
        let e = ripple_speed(&i, FS, 6.0).expect("ripple found");
        assert!(
            (e.freq_hz - 1800.0).abs() / 1800.0 < 0.02,
            "freq {}",
            e.freq_hz
        );
        assert!((e.motor_rev_s - 300.0).abs() / 300.0 < 0.02);
        assert!(e.strength > 0.3, "strength {}", e.strength);
    }

    #[test]
    fn flat_or_pure_noise_returns_none() {
        assert!(ripple_speed(&vec![512.0; 4000], FS, 6.0).is_none());
        let mut lcg = Lcg(9);
        let noise: Vec<f64> = (0..4000).map(|_| 512.0 + lcg.next(3.0)).collect();
        assert!(ripple_speed(&noise, FS, 6.0).is_none(), "no periodicity");
        assert!(ripple_speed(&[1.0; 10], FS, 6.0).is_none(), "too short");
    }

    #[test]
    fn gear_ratio_check_is_rung_consistent() {
        // same physical train sampled at two speeds: the relative ratio
        // must agree even though 4096.0 is not a physical output rev
        let ratio_a = gear_ratio_check(300.0, 3679.0, 4096.0);
        let ratio_b = gear_ratio_check(150.0, 3679.0 / 2.0, 4096.0);
        assert!((ratio_a - ratio_b).abs() / ratio_a < 1e-12);
        assert!((ratio_a - 300.0 / (3679.0 / 4096.0)).abs() < 1e-9);
    }
}
