//! Gear-slip / stripped-teeth metric from a constant-duty ripple sweep.
//!
//! Over a constant-duty full-travel traverse the pot must advance in fixed
//! proportion to motor rotation: pot counts per motor rev is ~constant for a
//! sound train. A slipping or stripped mesh makes a segment spike or drop
//! toward zero (motor spins, pot frozen). Splitting the run into equal-index
//! segments and measuring per-segment counts/motor-rev exposes that - a high
//! coefficient of variation, or any core segment near zero, flags the mesh.
//!
//! Rail-adjacent segments are excluded: mechanical stiffness or a jam near a
//! hard stop mimics slip. The metric is RIPPLE_PER_REV-independent (a ratio of
//! ratios), pure, and wasm-safe. Single sweep is advisory only - a stripped
//! gear whose slip zone is missed can read healthy, and a bad capture can
//! false-positive, so the caller must say so.

/// CoV of rail-excluded per-segment counts/motor-rev above this flags slip.
pub const SLIP_COV_MAX: f64 = 0.20;
/// A core segment below this fraction of the median counts as stuck.
pub const STUCK_FRAC: f64 = 0.40;

/// Slip metrics over a rail-excluded constant-duty sweep (see [`slip_metrics`]).
pub struct SlipReport {
    /// CoV (pstdev/mean) of rail-excluded per-segment counts/motor-rev.
    pub slip_cov: f64,
    /// Min core segment / median positive core segment (stuck -> ~0).
    pub min_over_median: f64,
    /// Core segments below STUCK_FRAC * median.
    pub stuck_count: usize,
    /// Core segments evaluated (after edge exclusion).
    pub core_segments: usize,
    /// slip_cov > SLIP_COV_MAX || stuck_count >= 1.
    pub flagged: bool,
}

/// Slip metrics from a constant-duty sweep: time-aligned raw pot samples and
/// the cumulative motor phase (revs) from lut::cumulative_phase. Splits into
/// `segments` equal-index chunks, drops `edge_exclude` chunks at each end
/// (rail stiffness mimics slip), and measures per-segment counts/motor-rev.
/// RIPPLE_PER_REV-independent (a ratio of ratios). None when input is too
/// short/mismatched or the run never moved (no positive segment).
pub fn slip_metrics(
    pos: &[u16],
    motor_phase: &[f64],
    segments: usize,
    edge_exclude: usize,
) -> Option<SlipReport> {
    let n = pos.len();
    if n != motor_phase.len() || segments < 3 || 2 * edge_exclude >= segments || n < 2 * segments {
        return None;
    }
    // per-segment counts/motor-rev over equal-index chunks; near-zero phase
    // advance is a stall -> 0 (not a divide-by-tiny spike).
    let mut locals = Vec::with_capacity(segments);
    for i in 0..segments {
        let a = i * n / segments;
        let z = (i + 1) * n / segments;
        let dph = motor_phase[z - 1] - motor_phase[a];
        let dpos = (pos[z - 1] as i32 - pos[a] as i32).unsigned_abs() as f64;
        locals.push(if dph > 1e-3 { dpos / dph } else { 0.0 });
    }
    let core = &locals[edge_exclude..segments - edge_exclude];
    if core.len() < 3 {
        return None;
    }
    let mut positives: Vec<f64> = core.iter().copied().filter(|&x| x > 0.0).collect();
    if positives.is_empty() {
        return None;
    }
    let med = median(&mut positives);
    // mean over ALL core (zeros included), so a stuck segment rightly inflates
    // the CoV rather than being averaged out of the denominator.
    let mean = core.iter().sum::<f64>() / core.len() as f64;
    if mean <= 0.0 {
        return None;
    }
    let var = core.iter().map(|&x| (x - mean) * (x - mean)).sum::<f64>() / core.len() as f64;
    let slip_cov = var.sqrt() / mean;
    let min = core.iter().copied().fold(f64::INFINITY, f64::min);
    let stuck_count = core.iter().filter(|&&x| x < STUCK_FRAC * med).count();
    let flagged = slip_cov > SLIP_COV_MAX || stuck_count >= 1;
    Some(SlipReport {
        slip_cov,
        min_over_median: min / med,
        stuck_count,
        core_segments: core.len(),
        flagged,
    })
}

/// Median of a non-empty slice; sorts in place.
fn median(v: &mut [f64]) -> f64 {
    v.sort_by(f64::total_cmp);
    let m = v.len() / 2;
    if v.len() % 2 == 1 {
        v[m]
    } else {
        (v[m - 1] + v[m]) / 2.0
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    const N: usize = 800;
    const SEG: usize = N / 10; // 80 samples/segment at segments=10

    /// Build a sweep: phase advances uniformly (+0.01/sample); pos advances by
    /// `step(chunk)` counts/sample. Chunk index is `k / SEG`.
    fn build(step: impl Fn(usize) -> i32) -> (Vec<u16>, Vec<f64>) {
        let mut pos = Vec::with_capacity(N);
        let mut phase = Vec::with_capacity(N);
        let mut val = 0i32;
        for k in 0..N {
            pos.push(val as u16);
            phase.push(0.01 * k as f64);
            val += step(k / SEG);
        }
        (pos, phase)
    }

    #[test]
    fn healthy_uniform_is_clean() {
        let (pos, phase) = build(|_| 2);
        let r = slip_metrics(&pos, &phase, 10, 1).expect("report");
        assert_eq!(r.core_segments, 8);
        assert!(r.slip_cov < 1e-9, "cov {}", r.slip_cov);
        assert_eq!(r.stuck_count, 0);
        assert!(
            (r.min_over_median - 1.0).abs() < 1e-9,
            "mom {}",
            r.min_over_median
        );
        assert!(!r.flagged);
    }

    #[test]
    fn stripped_stuck_core_segment_flags() {
        // core chunk 5 frozen (motor spins via phase, pot flat) -> local 0
        let (pos, phase) = build(|c| if c == 5 { 0 } else { 2 });
        let r = slip_metrics(&pos, &phase, 10, 1).expect("report");
        assert!(r.stuck_count >= 1, "stuck {}", r.stuck_count);
        assert!(r.min_over_median < 0.01, "mom {}", r.min_over_median);
        assert!(r.flagged);
    }

    #[test]
    fn high_variance_no_stuck_flags_on_cov() {
        // core chunks alternate 2/4 counts/sample -> locals 200/400, spread
        // wide enough to cross the CoV gate with no single stuck segment.
        let (pos, phase) = build(|c| {
            if (1..=8).contains(&c) && c % 2 == 0 {
                4
            } else {
                2
            }
        });
        let r = slip_metrics(&pos, &phase, 10, 1).expect("report");
        assert_eq!(r.stuck_count, 0, "no segment should be stuck");
        assert!(r.slip_cov > 0.20, "cov {}", r.slip_cov);
        assert!(r.flagged);
    }

    #[test]
    fn rail_edge_stuck_is_excluded() {
        // edge chunk 0 frozen but every core chunk clean -> not flagged,
        // proving the first/last edge_exclude chunks are dropped.
        let (pos, phase) = build(|c| if c == 0 { 0 } else { 2 });
        let r = slip_metrics(&pos, &phase, 10, 1).expect("report");
        assert_eq!(r.stuck_count, 0);
        assert!(r.slip_cov < 1e-9, "cov {}", r.slip_cov);
        assert!(!r.flagged);
    }

    #[test]
    fn degenerate_inputs_are_none() {
        let (pos, phase) = build(|_| 2);
        // length mismatch
        assert!(slip_metrics(&pos, &phase[..N / 2], 10, 1).is_none());
        // too few samples for the segment count
        assert!(slip_metrics(&pos[..10], &phase[..10], 10, 1).is_none());
        // too few segments
        assert!(slip_metrics(&pos, &phase, 2, 0).is_none());
        // edge exclusion consumes every segment
        assert!(slip_metrics(&pos, &phase, 10, 5).is_none());
    }
}
