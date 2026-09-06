//! Ripple-referenced pot linearization LUT: builder + host-side apply.
//!
//! The pot is nonlinear, so a 2-point linear map (raw_min at one rail,
//! raw_max at the other) mis-reports angle mid-travel. This module builds a
//! 55-knot correction table from a calibration sweep and applies it on the
//! host - the firmware only STORES the block, it never converts.
//!
//! Angle clock: over a constant-duty full-travel sweep the motor commutation
//! ripple on the winding current is a motor-shaft angle clock. Its cumulative
//! phase is a drift-resistant TRUE-angle axis (no constant-velocity
//! assumption, unlike sampling against wall time); raw pot counts sampled
//! against that axis reveal the pot's nonlinearity. One monotonic pot
//! channel, so the result is a plain raw->fraction curve (no 2D search).
//!
//! LUT semantics (host-defined; firmware does not apply the block):
//! - 55 knots evenly spaced in raw counts, raw_i = raw_min + i*span/54.
//! - corrected(raw_i) = raw_i + corr[i]; linear interp between knots; raw
//!   outside [raw_min,raw_max] clamps to the rail.
//! - linearize(raw) = (corrected(raw) - raw_min)/span clamped to [0,1].
//! - all-zero corr == identity == the 2-point-linear baseline: corr shifts
//!   each knot's corrected value off the straight line, so zeros leave
//!   linearize equal to the plain linear fraction.

use crate::ripple;

/// Knot count = the firmware PotLutBlock's lut_corr length.
const N_KNOTS: usize = 55;
/// Intervals between knots.
const N_INTERVALS: usize = N_KNOTS - 1;
/// Below this many aligned samples the sweep can't define a curve.
const MIN_SAMPLES: usize = 4;
/// cumulative_phase needs a majority of windows to find ripple, else the
/// sweep SNR is too low to trust as an angle clock.
const MIN_GOOD_FRAC: f64 = 0.5;

/// A sweep covering less than this fraction of the rail span leaves too much of
/// travel for `build`'s affine anchoring to extrapolate through: the anchoring
/// assumes local linearity over the uncovered insets, which fails once whole
/// rail regions are uncovered. Below this coverage `build` returns identity.
pub const MIN_SPAN_COVER: f64 = 0.7;

/// Host mirror of the firmware PotLutBlock (raw_min/raw_max/lut_corr).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct PotLut {
    pub raw_min: u16,
    pub raw_max: u16,
    pub corr: [i16; N_KNOTS],
}

impl PotLut {
    /// All-zero corr: linearize reduces to the plain linear fraction.
    pub fn identity(raw_min: u16, raw_max: u16) -> PotLut {
        PotLut {
            raw_min,
            raw_max,
            corr: [0; N_KNOTS],
        }
    }

    /// Corrected raw value at knot i: on-line position plus its correction.
    fn corrected_knot(&self, i: usize) -> f64 {
        let span = self.raw_max as f64 - self.raw_min as f64;
        self.raw_min as f64 + i as f64 * span / N_INTERVALS as f64 + self.corr[i] as f64
    }

    /// Normalized linearized fraction in [0,1]. Degenerate span -> 0.
    pub fn linearize(&self, raw: u16) -> f64 {
        let lo = self.raw_min as f64;
        let hi = self.raw_max as f64;
        let span = hi - lo;
        if span <= 0.0 {
            return 0.0;
        }
        let r = (raw as f64).clamp(lo, hi);
        let pos = ((r - lo) / span * N_INTERVALS as f64).clamp(0.0, N_INTERVALS as f64);
        let i0 = pos.floor() as usize;
        let corrected = if i0 >= N_INTERVALS {
            self.corrected_knot(N_INTERVALS)
        } else {
            let frac = pos - i0 as f64;
            let c0 = self.corrected_knot(i0);
            let c1 = self.corrected_knot(i0 + 1);
            c0 + frac * (c1 - c0)
        };
        ((corrected - lo) / span).clamp(0.0, 1.0)
    }

    /// Angle in centi-degrees, composing the linearized fraction with the
    /// kinematics endpoints (angle_min_cdeg at raw_min, angle_max at raw_max).
    pub fn angle_cdeg(&self, raw: u16, angle_min_cdeg: i16, angle_max_cdeg: i16) -> f64 {
        let f = self.linearize(raw);
        angle_min_cdeg as f64 + f * (angle_max_cdeg as f64 - angle_min_cdeg as f64)
    }
}

/// Cumulative motor phase (revs) per sample from a constant-duty sweep's
/// current series, paired with the fraction of windows that found ripple (a
/// coverage/confidence metric in `MIN_GOOD_FRAC..=1.0`). Slides ripple_speed
/// over the series, integrates the per-window rev/s (drift-resistant: an
/// occasional dropped window is interpolated across, not extrapolated from one
/// global rate), and returns a monotonic phase. None when input is too short
/// or a majority of windows fail the ripple confidence floor.
pub fn cumulative_phase(current: &[f64], fs: f64, ripple_per_rev: f64) -> Option<(Vec<f64>, f64)> {
    let n = current.len();
    if fs <= 0.0 || ripple_per_rev <= 0.0 {
        return None;
    }
    let win = ripple::min_window(fs);
    if win == 0 || n < win {
        return None;
    }
    let hop = (win / 4).max(1);
    // (window center sample, motor rev/s) for windows that clear the floor
    let mut centers: Vec<(f64, f64)> = Vec::new();
    let mut total = 0usize;
    let mut start = 0usize;
    while start + win <= n {
        total += 1;
        if let Some(e) = ripple::ripple_speed(&current[start..start + win], fs, ripple_per_rev) {
            centers.push((start as f64 + win as f64 / 2.0, e.motor_rev_s));
        }
        start += hop;
    }
    if centers.len() < 2 || (centers.len() as f64) < MIN_GOOD_FRAC * total as f64 {
        return None;
    }
    let good_frac = centers.len() as f64 / total as f64;
    // per-sample rev/s: linear interp across window centers, clamped at the
    // ends; cumulative-trapezoid to revs. rev/s >= 0 keeps phase monotone.
    let mut phase = Vec::with_capacity(n);
    let mut acc = 0.0;
    let mut prev = rate_at(&centers, 0.0);
    phase.push(0.0);
    for k in 1..n {
        let rs = rate_at(&centers, k as f64);
        acc += 0.5 * (prev + rs) / fs;
        phase.push(acc);
        prev = rs;
    }
    Some((phase, good_frac))
}

/// Fraction of the rail span [raw_min,raw_max] the sweep's raw pot samples
/// actually covered, from robust 2/98 percentiles of raw_pot rather than
/// absolute min/max: (p98 - p2) / (raw_max - raw_min), clamped to [0,1]. The
/// percentiles keep a couple of outlier samples near a rail from inflating a
/// partial sweep to full coverage. 0.0 on empty input or a zero/degenerate
/// denominator.
pub fn span_coverage(raw_pot: &[u16], raw_min: u16, raw_max: u16) -> f64 {
    if raw_pot.is_empty() {
        return 0.0;
    }
    let denom = raw_max as f64 - raw_min as f64;
    if denom <= 0.0 {
        return 0.0;
    }
    let mut sorted = raw_pot.to_vec();
    sorted.sort_unstable();
    let n = sorted.len();
    let idx = |p: f64| ((n - 1) as f64 * p).round() as usize;
    let lo = sorted[idx(0.02)] as f64;
    let hi = sorted[idx(0.98)] as f64;
    ((hi - lo) / denom).clamp(0.0, 1.0)
}

/// Build the LUT from time-aligned raw pot samples and the cumulative motor
/// phase (monotonic, from cumulative_phase). Anchors the captured phase shape
/// into full-rail true-fraction space via the covered pot endpoints, forms a
/// monotone raw->fraction curve, and sets each knot's corr to the count offset
/// that lands its corrected value on the curve. Degenerate input (too few
/// samples, length mismatch, zero raw or phase span) -> identity.
pub fn build(raw_pot: &[u16], motor_phase: &[f64], raw_min: u16, raw_max: u16) -> PotLut {
    let n = raw_pot.len();
    let span = raw_max as i32 - raw_min as i32;
    if n < MIN_SAMPLES || n != motor_phase.len() || span <= 0 {
        return PotLut::identity(raw_min, raw_max);
    }
    let p0 = motor_phase[0];
    let pspan = motor_phase[n - 1] - p0;
    if !pspan.is_finite() || pspan == 0.0 {
        return PotLut::identity(raw_min, raw_max);
    }
    // partial-coverage guard: the affine anchoring below assumes local
    // linearity over the uncovered rail insets. That holds for the small
    // insets of a near-full sweep, but a sweep over only part of travel leaves
    // whole rail regions to extrapolate through, where the assumption breaks
    // and the corrections become untrustworthy. Below MIN_SPAN_COVER, bail.
    if span_coverage(raw_pot, raw_min, raw_max) < MIN_SPAN_COVER {
        return PotLut::identity(raw_min, raw_max);
    }
    // (raw, rel) pairs; rel = phase fraction over the CAPTURED range (0..1).
    // Orient rel to increase with raw: raw is monotonic in true angle and so
    // is phase, making rel monotone in raw regardless of sweep direction.
    let mut pts: Vec<(f64, f64)> = (0..n)
        .map(|k| (raw_pot[k] as f64, (motor_phase[k] - p0) / pspan))
        .collect();
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    if pts[pts.len() - 1].1 < pts[0].1 {
        for p in pts.iter_mut() {
            p.1 = 1.0 - p.1;
        }
    }
    fit_knots(pts, raw_min, raw_max)
}

/// Fit the 55-knot correction table from `pts` = (raw, rel), where rel is in
/// [0,1] and ALREADY oriented to increase with raw. Anchors the covered pot
/// endpoints affinely into full-rail true-fraction space, pins the rails to
/// frac 0/1, forms a monotone raw->fraction curve, and sets each knot's corr
/// to the count offset landing its corrected value on the curve. Degenerate
/// input (single raw value, <2 curve points) -> identity.
fn fit_knots(mut pts: Vec<(f64, f64)>, raw_min: u16, raw_max: u16) -> PotLut {
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    let span_f = raw_max as f64 - raw_min as f64;
    // Anchor rel into full-rail true-fraction space. rel spans 0..1 over only
    // the covered pot range [rp_lo,rp_hi]; map it affinely onto that range's
    // fractions of the FULL rail span, assuming local linearity over the small
    // uncovered insets. A full-rail sweep has rp_lo=raw_min, rp_hi=raw_max ->
    // tf_lo=0, tf_hi=1 -> the mapping is the identity (backward compatible).
    let rp_lo = pts[0].0;
    let rp_hi = pts[pts.len() - 1].0;
    let tf_lo = (rp_lo - raw_min as f64) / span_f;
    let tf_hi = (rp_hi - raw_min as f64) / span_f;
    if tf_hi == tf_lo {
        return PotLut::identity(raw_min, raw_max);
    }
    for p in pts.iter_mut() {
        p.1 = tf_lo + p.1 * (tf_hi - tf_lo);
    }
    // pin the mechanical rails to frac 0/1: raw_min/raw_max are the kinematics
    // endpoints, so linearize must return exactly 0/1 there. interp then links
    // each rail linearly to the nearest covered endpoint, tapering inset corr to
    // 0 at the rail. A full-rail sweep already carries tf_lo=0/tf_hi=1 there, so
    // the dedup pass averages the duplicate rail cleanly (behavior unchanged).
    pts.push((raw_min as f64, 0.0));
    pts.push((raw_max as f64, 1.0));
    pts.sort_by(|a, b| a.0.total_cmp(&b.0));
    // average duplicate raws (pot flat spots, rounding collisions)
    let mut curve: Vec<(f64, f64)> = Vec::with_capacity(pts.len());
    let mut i = 0;
    while i < pts.len() {
        let r = pts[i].0;
        let mut s = 0.0;
        let mut c = 0.0;
        while i < pts.len() && pts[i].0 == r {
            s += pts[i].1;
            c += 1.0;
            i += 1;
        }
        curve.push((r, s / c));
    }
    if curve.len() < 2 {
        return PotLut::identity(raw_min, raw_max);
    }
    // enforce non-decreasing frac (noise guard)
    for j in 1..curve.len() {
        if curve[j].1 < curve[j - 1].1 {
            curve[j].1 = curve[j - 1].1;
        }
    }
    let mut corr = [0i16; N_KNOTS];
    for (k, c) in corr.iter_mut().enumerate() {
        let raw_i = raw_min as f64 + k as f64 * span_f / N_INTERVALS as f64;
        // uncovered-inset knots lie on the interp segment from the rail anchor
        // (frac 0/1) to the nearest covered endpoint, so their corr tapers to 0
        // at the rail; the span_coverage gate above rejects sweeps whose
        // uncovered regions would grow that taper large.
        let frac = interp(&curve, raw_i);
        let corrected = raw_min as f64 + frac * span_f;
        *c = sat_i16((corrected - raw_i).round());
    }
    PotLut {
        raw_min,
        raw_max,
        corr,
    }
}

/// Per-count cumulative motor revs stitched from several chunks over the
/// shared pos axis. `cum` is indexed by bin (bin b = count raw_min+b); `fc`/
/// `lc` bracket the covered bins; `coverage` is the covered fraction of the
/// rail span.
struct Stitch {
    cum: Vec<f64>,
    fc: usize,
    lc: usize,
    coverage: f64,
}

/// Integrate a per-count motor-rev density over the rail span from multiple
/// clean chunks. Each chunk contributes its local rev/count density (motor
/// phase from cumulative_phase); the pos axis is global and monotonic in true
/// angle, so the chunks share one count grid and bridge each other's gaps. A
/// chunk whose ripple phase can't be recovered is skipped, not fatal. None when
/// the span is degenerate, fewer than two bins are covered, or the integrated
/// rev span is non-positive.
///
/// Density comes from TIME-ORDERED steps, not a pos-sorted scan: phase is the
/// clean monotone clock while pot pos is noisy (the ADC dithers +/-several
/// counts around a slow advance). Depositing each step's dphi into the count(s)
/// occupied at that instant makes each bin accumulate exactly the revs turned
/// while sitting near that count, so a chunk's deposits sum to its raw phase
/// span with no loss. A pos-sorted dedup instead averages phase across every
/// time a jittered pos revisits a count, which breaks phase monotonicity vs pos
/// (spurious reversals) and, once negative windows are dropped, both loses
/// coverage and over-counts revs.
fn stitch_slope(
    chunks: &[(Vec<u16>, Vec<f64>)],
    fs: f64,
    ripple_per_rev: f64,
    raw_min: u16,
    raw_max: u16,
) -> Option<Stitch> {
    let span = raw_max as i32 - raw_min as i32;
    if span <= 0 {
        return None;
    }
    let bins = span as usize + 1;
    // acc[b] = sum over covering chunks of that chunk's local rev/count density
    // at bin b; chunk_cov[b] = how many chunks covered b. Density = acc/chunk_cov
    // (chunks that overlap in pos measured the same physical count and average).
    let mut acc = vec![0.0; bins];
    let mut chunk_cov = vec![0u32; bins];
    let mut local = vec![0.0; bins];
    let mut touched = vec![false; bins];
    for (pos, current) in chunks {
        if pos.len() != current.len() || pos.len() < MIN_SAMPLES {
            continue;
        }
        let phi = if let Some((phi, _)) = cumulative_phase(current, fs, ripple_per_rev) {
            phi
        } else {
            continue;
        };
        local.iter_mut().for_each(|x| *x = 0.0);
        touched.iter_mut().for_each(|x| *x = false);
        for k in 0..pos.len() - 1 {
            let dphi = (phi[k + 1] - phi[k]).max(0.0);
            let (a, b) = (pos[k].min(pos[k + 1]), pos[k].max(pos[k + 1]));
            if a == b {
                // sat at one count for this step: all of dphi lands in bin a.
                // A sample sitting exactly at raw_max folds into the top interval
                // (bin raw_max-1) rather than being dropped, so a rail dwell
                // keeps its revs.
                if a >= raw_min && a <= raw_max {
                    let idx = (a.min(raw_max - 1) - raw_min) as usize;
                    local[idx] += dphi;
                    touched[idx] = true;
                }
            } else {
                // crossed [a,b): spread dphi evenly over the counts traversed
                let s = dphi / (b - a) as f64;
                for c in a..b {
                    if c >= raw_min && c < raw_max {
                        let idx = (c - raw_min) as usize;
                        local[idx] += s;
                        touched[idx] = true;
                    }
                }
            }
        }
        for b in 0..bins {
            if touched[b] {
                acc[b] += local[b];
                chunk_cov[b] += 1;
            }
        }
    }
    let covered = chunk_cov.iter().filter(|&&c| c > 0).count();
    if covered < 2 {
        return None;
    }
    let fc = chunk_cov.iter().position(|&c| c > 0).unwrap();
    let lc = chunk_cov.iter().rposition(|&c| c > 0).unwrap();
    let coverage = covered as f64 / bins as f64;
    let mut avg = vec![0.0; bins];
    for b in 0..bins {
        if chunk_cov[b] > 0 {
            avg[b] = acc[b] / chunk_cov[b] as f64;
        }
    }
    // linearly interpolate uncovered interior bins between covered neighbours so
    // the integrated cumulative stays continuous and monotone across gaps. The
    // anchors are the MEDIAN density over up to GAP_ANCHOR covered bins scanning
    // inward from each edge, not the single boundary bin: a chunk's outermost
    // bin is partially occupied (the chunk was cut off mid-count) and reads low,
    // so anchoring a wide gap on it under-fills the revs the motor turned while
    // crossing that gap. The median over a small interior neighbourhood is the
    // representative local density.
    let mut prev = fc;
    for b in (fc + 1)..=lc {
        if chunk_cov[b] > 0 {
            if b > prev + 1 {
                let y0 = anchor_density(&avg, &chunk_cov, prev, -1, fc, lc);
                let y1 = anchor_density(&avg, &chunk_cov, b, 1, fc, lc);
                let dx = (b - prev) as f64;
                for (j, g) in avg[(prev + 1)..b].iter_mut().enumerate() {
                    *g = y0 + (y1 - y0) * ((j + 1) as f64 / dx);
                }
            }
            prev = b;
        }
    }
    let mut cum = vec![0.0; bins];
    for b in (fc + 1)..=lc {
        cum[b] = cum[b - 1] + avg[b];
    }
    for b in (lc + 1)..bins {
        cum[b] = cum[lc];
    }
    if cum[lc] - cum[fc] <= 0.0 {
        return None;
    }
    Some(Stitch {
        cum,
        fc,
        lc,
        coverage,
    })
}

/// Build the pot LUT from MULTIPLE clean sweep chunks (each (pos, current)),
/// stitched over the shared pos axis. Chunks may cover overlapping or disjoint
/// pos ranges with gaps between them; the per-count slope integration bridges
/// the gaps. Identity when coverage < MIN_SPAN_COVER or the stitch is degenerate.
pub fn build_multi(
    chunks: &[(Vec<u16>, Vec<f64>)],
    fs: f64,
    ripple_per_rev: f64,
    raw_min: u16,
    raw_max: u16,
) -> PotLut {
    let st = match stitch_slope(chunks, fs, ripple_per_rev, raw_min, raw_max) {
        Some(s) => s,
        None => return PotLut::identity(raw_min, raw_max),
    };
    if st.coverage < MIN_SPAN_COVER {
        return PotLut::identity(raw_min, raw_max);
    }
    let total = st.cum[st.lc] - st.cum[st.fc];
    if total <= 0.0 {
        return PotLut::identity(raw_min, raw_max);
    }
    // (pos, rel), rel in [0,1] ascending with pos (no flip needed)
    let pts: Vec<(f64, f64)> = (st.fc..=st.lc)
        .map(|b| {
            (
                raw_min as f64 + b as f64,
                (st.cum[b] - st.cum[st.fc]) / total,
            )
        })
        .collect();
    fit_knots(pts, raw_min, raw_max)
}

/// Full-travel motor revs + coverage from the stitched chunks, for the gear/
/// travel anchor. Extrapolates the covered-span revs to the full rail span.
pub fn stitched_motor_revs(
    chunks: &[(Vec<u16>, Vec<f64>)],
    fs: f64,
    ripple_per_rev: f64,
    raw_min: u16,
    raw_max: u16,
) -> Option<(f64, f64)> {
    let st = stitch_slope(chunks, fs, ripple_per_rev, raw_min, raw_max)?;
    let revs = st.cum[st.lc] - st.cum[st.fc];
    let pot_disp = (st.lc - st.fc) as f64;
    let count_span = raw_max as f64 - raw_min as f64;
    let full = crate::kinematics::full_span_motor_revs(revs, pot_disp, count_span);
    (full != 0.0).then_some((full, st.coverage))
}

/// Count of covered bins scanned inward from a gap edge to form the anchor
/// median (see stitch_slope's interpolation).
const GAP_ANCHOR: usize = 8;

/// Median density over up to GAP_ANCHOR covered bins scanning from `start` in
/// direction `dir` (+1/-1), staying within [lo,lc]. Skips uncovered bins (gap
/// fills), so anchors are always real measurements. 0.0 if none found.
fn anchor_density(
    avg: &[f64],
    chunk_cov: &[u32],
    start: usize,
    dir: i32,
    lo: usize,
    hi: usize,
) -> f64 {
    let mut vals: Vec<f64> = Vec::with_capacity(GAP_ANCHOR);
    let mut b = start as i64;
    while vals.len() < GAP_ANCHOR && b >= lo as i64 && b <= hi as i64 {
        let idx = b as usize;
        if chunk_cov[idx] > 0 {
            vals.push(avg[idx]);
        }
        b += dir as i64;
    }
    if vals.is_empty() {
        return 0.0;
    }
    vals.sort_by(f64::total_cmp);
    vals[vals.len() / 2]
}

/// Rate (rev/s) at sample index x by linear interp over window centers,
/// clamped to the first/last center outside the covered span.
fn rate_at(centers: &[(f64, f64)], x: f64) -> f64 {
    interp(centers, x)
}

/// Linear interpolation over an x-sorted table, clamping outside the range.
fn interp(pts: &[(f64, f64)], x: f64) -> f64 {
    let last = pts.len() - 1;
    if x <= pts[0].0 {
        return pts[0].1;
    }
    if x >= pts[last].0 {
        return pts[last].1;
    }
    let (mut lo, mut hi) = (0usize, last);
    while hi - lo > 1 {
        let mid = (lo + hi) / 2;
        if pts[mid].0 <= x {
            lo = mid;
        } else {
            hi = mid;
        }
    }
    let (x0, y0) = pts[lo];
    let (x1, y1) = pts[hi];
    if x1 == x0 {
        return y0;
    }
    y0 + (y1 - y0) * (x - x0) / (x1 - x0)
}

fn sat_i16(v: f64) -> i16 {
    if !v.is_finite() {
        return 0;
    }
    v.clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::f64::consts::PI;

    const RAW_MIN: u16 = 200;
    const RAW_MAX: u16 = 3900;
    const SPAN: f64 = (RAW_MAX - RAW_MIN) as f64;

    // Monotonic nonlinear pot: raw = raw_min + span*(f + a*sin(pi*f)).
    // a < 1/pi keeps it monotone; endpoints fixed (sin(0)=sin(pi)=0).
    fn pot_raw(f: f64, a: f64) -> u16 {
        (RAW_MIN as f64 + SPAN * (f + a * (PI * f).sin())).round() as u16
    }

    fn sweep(a: f64, m: usize) -> (Vec<u16>, Vec<f64>) {
        let mut raw = Vec::with_capacity(m);
        let mut phase = Vec::with_capacity(m);
        for k in 0..m {
            let f = k as f64 / (m - 1) as f64;
            raw.push(pot_raw(f, a));
            // rigid gear: motor phase proportional to true angle
            phase.push(3.0 * f);
        }
        (raw, phase)
    }

    #[test]
    fn identity_linearize_is_linear_fraction() {
        let lut = PotLut::identity(RAW_MIN, RAW_MAX);
        for raw in [RAW_MIN, 1000, 2048, 3000, RAW_MAX] {
            let expect = (raw as f64 - RAW_MIN as f64) / SPAN;
            assert!((lut.linearize(raw) - expect).abs() < 1e-12, "raw {raw}");
        }
        assert_eq!(lut.angle_cdeg(RAW_MIN, 0, 19000), 0.0);
        assert!((lut.angle_cdeg(RAW_MAX, 0, 19000) - 19000.0).abs() < 1e-9);
    }

    #[test]
    fn out_of_range_raw_clamps() {
        let lut = PotLut::identity(RAW_MIN, RAW_MAX);
        assert_eq!(lut.linearize(0), 0.0);
        assert_eq!(lut.linearize(u16::MAX), 1.0);
    }

    #[test]
    fn build_linear_pot_yields_near_zero_corr() {
        let (raw, phase) = sweep(0.0, 300);
        let lut = build(&raw, &phase, RAW_MIN, RAW_MAX);
        // only rounding of the u16 raw samples separates corr from zero
        assert!(
            lut.corr.iter().all(|&c| c.abs() <= 2),
            "corr {:?}",
            lut.corr
        );
    }

    #[test]
    fn build_recovers_nonlinear_pot_and_beats_identity() {
        let a = 0.15;
        let (raw, phase) = sweep(a, 400);
        let lut = build(&raw, &phase, RAW_MIN, RAW_MAX);
        let ident = PotLut::identity(RAW_MIN, RAW_MAX);
        let mut lut_max = 0.0f64;
        let mut ident_max = 0.0f64;
        for (k, &r) in raw.iter().enumerate() {
            let f = k as f64 / 399.0;
            lut_max = lut_max.max((lut.linearize(r) - f).abs());
            ident_max = ident_max.max((ident.linearize(r) - f).abs());
        }
        // fraction error is a fraction of span; < 1% of span required
        assert!(lut_max < 0.01, "lut err {lut_max}");
        // identity is off by ~a*sin(pi*f) mid-travel; LUT must trounce it
        assert!(ident_max > 0.1, "ident err {ident_max}");
        assert!(
            lut_max < ident_max / 10.0,
            "lut {lut_max} ident {ident_max}"
        );
    }

    #[test]
    fn build_degenerate_is_identity() {
        assert_eq!(
            build(&[], &[], RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
        assert_eq!(
            build(&[1, 2], &[0.0, 1.0], RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
        // zero raw span
        let (raw, phase) = sweep(0.1, 100);
        assert_eq!(build(&raw, &phase, 500, 500), PotLut::identity(500, 500));
        // length mismatch
        assert_eq!(
            build(&raw, &phase[..50], RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
    }

    #[test]
    fn span_coverage_fraction_and_degenerate() {
        // full-span samples -> ~1.0
        assert!((span_coverage(&[RAW_MIN, RAW_MAX], RAW_MIN, RAW_MAX) - 1.0).abs() < 1e-12);
        // middle half of the span -> ~0.5
        let q1 = RAW_MIN + (SPAN * 0.25) as u16;
        let q3 = RAW_MIN + (SPAN * 0.75) as u16;
        assert!((span_coverage(&[q1, q3], RAW_MIN, RAW_MAX) - 0.5).abs() < 0.01);
        // empty -> 0.0
        assert_eq!(span_coverage(&[], RAW_MIN, RAW_MAX), 0.0);
        // degenerate raw_min == raw_max -> 0.0
        assert_eq!(span_coverage(&[500, 600], 500, 500), 0.0);
    }

    #[test]
    fn build_partial_coverage_is_identity() {
        // nonlinear pot sampled over only the middle 40% of travel: phase is
        // valid but coverage is below MIN_SPAN_COVER, so build falls back to
        // identity rather than fabricating rail corrections.
        let a = 0.15;
        let m = 300;
        let mut raw = Vec::with_capacity(m);
        let mut phase = Vec::with_capacity(m);
        for k in 0..m {
            // f in [0.3, 0.7] -> ~40% of travel, centered mid-rail
            let f = 0.3 + 0.4 * k as f64 / (m - 1) as f64;
            raw.push(pot_raw(f, a));
            phase.push(3.0 * f);
        }
        assert!(span_coverage(&raw, RAW_MIN, RAW_MAX) < MIN_SPAN_COVER);
        let lut = build(&raw, &phase, RAW_MIN, RAW_MAX);
        assert_eq!(lut, PotLut::identity(RAW_MIN, RAW_MAX));
        assert!(lut.corr.iter().all(|&c| c == 0), "corr {:?}", lut.corr);
    }

    #[test]
    fn build_inset_sweep_no_rail_cliff() {
        // 94%-coverage sweep: f in [0.03,0.97], nonlinear pot inset ~3% from
        // each rail. The un-anchored normalization stretched the covered pot
        // range onto the full rails, fabricating large rail-region corrections
        // (hundreds+ here, unbounded as the inset grows). Affine anchoring plus
        // rail-anchor pins tapers near-rail corr toward 0 and pins the rails to
        // frac 0/1, while the interior keeps the measured shape.
        let a = 0.15;
        let m = 400;
        let mut raw = Vec::with_capacity(m);
        let mut phase = Vec::with_capacity(m);
        for k in 0..m {
            let f = 0.03 + 0.94 * k as f64 / (m - 1) as f64;
            raw.push(pot_raw(f, a));
            phase.push(3.0 * f);
        }
        // (a) coverage clears the gate, so build produces a real curve
        assert!(
            span_coverage(&raw, RAW_MIN, RAW_MAX) > MIN_SPAN_COVER,
            "coverage {}",
            span_coverage(&raw, RAW_MIN, RAW_MAX)
        );
        let lut = build(&raw, &phase, RAW_MIN, RAW_MAX);
        // (b) near-rail knot corrections taper small (no fabricated cliff)
        assert!(lut.corr[1].abs() < 30, "corr[1] {}", lut.corr[1]);
        assert!(
            lut.corr[N_KNOTS - 2].abs() < 30,
            "corr[N-2] {}",
            lut.corr[N_KNOTS - 2]
        );
        // rails pin to frac 0/1 (kinematics endpoints)
        assert!((lut.linearize(RAW_MIN) - 0.0).abs() < 1e-9);
        assert!((lut.linearize(RAW_MAX) - 1.0).abs() < 1e-9);
        // (c) linearize tracks the true fraction to < 2% of span over the
        // covered range (the anchor's local-linearity residual for a=0.15)
        let mut emax = 0.0f64;
        for (k, &r) in raw.iter().enumerate() {
            let f = 0.03 + 0.94 * k as f64 / (m - 1) as f64;
            emax = emax.max((lut.linearize(r) - f).abs());
        }
        assert!(emax < 0.02, "linearize err {emax}");
    }

    #[test]
    fn span_coverage_ignores_outliers() {
        // ~40%-coverage sweep (middle of travel) plus one outlier near each
        // rail. Absolute min/max would report ~full coverage; robust 2/98
        // percentiles reject the outliers and keep it below the 0.7 gate.
        let m = 300;
        let mut raw: Vec<u16> = Vec::with_capacity(m + 2);
        for k in 0..m {
            let f = 0.3 + 0.4 * k as f64 / (m - 1) as f64;
            raw.push(pot_raw(f, 0.0));
        }
        raw.push(RAW_MIN + 1);
        raw.push(RAW_MAX - 1);
        let cov = span_coverage(&raw, RAW_MIN, RAW_MAX);
        assert!(cov < MIN_SPAN_COVER, "outliers inflated coverage to {cov}");
        assert!(cov > 0.35 && cov < 0.45, "coverage {cov}");
    }

    #[test]
    fn linearize_zero_span_is_zero() {
        let lut = PotLut::identity(500, 500);
        assert_eq!(lut.linearize(500), 0.0);
    }

    // --- cumulative_phase ---

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

    fn ripple_series(freq: f64, amp: f64, noise: f64, n: usize) -> Vec<f64> {
        let mut lcg = Lcg(41);
        (0..n)
            .map(|k| {
                let t = k as f64 / FS;
                60.0 + amp * (2.0 * PI * freq * t).sin() + lcg.next(noise)
            })
            .collect()
    }

    #[test]
    fn cumulative_phase_recovers_total_revs() {
        let n = 4000;
        let i = ripple_series(1800.0, 4.0, 3.0, n);
        let (phase, good) = cumulative_phase(&i, FS, 6.0).expect("phase found");
        assert_eq!(phase.len(), n);
        assert!((MIN_GOOD_FRAC..=1.0).contains(&good), "coverage {good}");
        // monotone non-decreasing
        assert!(phase.windows(2).all(|w| w[1] >= w[0]));
        // 1800 Hz / 6 = 300 rev/s over (n-1)/fs s
        let expect = 300.0 * (n - 1) as f64 / FS;
        let got = phase[n - 1];
        assert!(
            (got - expect).abs() / expect < 0.03,
            "revs {got} vs {expect}"
        );
    }

    #[test]
    fn cumulative_phase_rejects_noise_and_short() {
        let mut lcg = Lcg(9);
        let noise: Vec<f64> = (0..4000).map(|_| 512.0 + lcg.next(3.0)).collect();
        assert!(cumulative_phase(&noise, FS, 6.0).is_none(), "pure noise");
        assert!(cumulative_phase(&[1.0; 10], FS, 6.0).is_none(), "too short");
    }

    // --- build_multi / stitched_motor_revs ---

    // Full nonlinear sweep with a CONSTANT-frequency ripple current: constant
    // duty => constant motor speed => true fraction f linear in sample index.
    fn ripple_sweep(a: f64, freq: f64, n: usize) -> (Vec<u16>, Vec<f64>) {
        let pos: Vec<u16> = (0..n)
            .map(|k| pot_raw(k as f64 / (n - 1) as f64, a))
            .collect();
        let cur = ripple_series(freq, 4.0, 3.0, n);
        (pos, cur)
    }

    fn chunkify(pos: &[u16], cur: &[f64], ranges: &[(usize, usize)]) -> Vec<(Vec<u16>, Vec<f64>)> {
        ranges
            .iter()
            .map(|&(lo, hi)| (pos[lo..hi].to_vec(), cur[lo..hi].to_vec()))
            .collect()
    }

    #[test]
    fn build_multi_recovers_full_sweep_from_chunks() {
        let a = 0.15;
        let n = 4500;
        let (pos, cur) = ripple_sweep(a, 1800.0, n);
        // 5 seq-contiguous chunks with ~100-sample gaps dropped between them
        let ranges = [
            (0, 850),
            (950, 1750),
            (1850, 2700),
            (2800, 3600),
            (3700, 4500),
        ];
        let chunks = chunkify(&pos, &cur, &ranges);
        let lut = build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX);
        let ident = PotLut::identity(RAW_MIN, RAW_MAX);
        // (a) not identity
        assert!(
            lut.corr.iter().any(|&c| c != 0),
            "corr all zero {:?}",
            lut.corr
        );
        // (b)/(c) error over the covered chunk samples
        let mut lut_max = 0.0f64;
        let mut ident_max = 0.0f64;
        for &(lo, hi) in ranges.iter() {
            for (j, &r) in pos[lo..hi].iter().enumerate() {
                let f = (lo + j) as f64 / (n - 1) as f64;
                lut_max = lut_max.max((lut.linearize(r) - f).abs());
                ident_max = ident_max.max((ident.linearize(r) - f).abs());
            }
        }
        assert!(lut_max < 0.025, "lut err {lut_max}");
        assert!(lut_max < ident_max / 5.0, "lut {lut_max} ident {ident_max}");
    }

    #[test]
    fn build_multi_coverage_gate_is_identity() {
        // chunks covering only the middle ~40% of travel -> below the gate
        let a = 0.15;
        let n = 4500;
        let (pos, cur) = ripple_sweep(a, 1800.0, n);
        let lo = (0.30 * (n - 1) as f64) as usize;
        let mid = (0.50 * (n - 1) as f64) as usize;
        let hi = (0.70 * (n - 1) as f64) as usize;
        let chunks = chunkify(&pos, &cur, &[(lo, mid), (mid, hi)]);
        let lut = build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX);
        assert_eq!(lut, PotLut::identity(RAW_MIN, RAW_MAX));
    }

    #[test]
    fn build_multi_single_chunk_reasonable() {
        let a = 0.15;
        let n = 4500;
        let (pos, cur) = ripple_sweep(a, 1800.0, n);
        let chunks = chunkify(&pos, &cur, &[(0, n)]);
        let lut = build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX);
        assert!(
            lut.corr.iter().any(|&c| c != 0),
            "corr all zero {:?}",
            lut.corr
        );
        let mut emax = 0.0f64;
        for (k, &r) in pos.iter().enumerate() {
            let f = k as f64 / (n - 1) as f64;
            emax = emax.max((lut.linearize(r) - f).abs());
        }
        assert!(emax < 0.025, "linearize err {emax}");
    }

    #[test]
    fn stitched_motor_revs_recovers_total() {
        let a = 0.15;
        let n = 4500;
        let (pos, cur) = ripple_sweep(a, 1800.0, n);
        let ranges = [
            (0, 850),
            (950, 1750),
            (1850, 2700),
            (2800, 3600),
            (3700, 4500),
        ];
        let chunks = chunkify(&pos, &cur, &ranges);
        let (full, cov) = stitched_motor_revs(&chunks, FS, 6.0, RAW_MIN, RAW_MAX).expect("revs");
        // 1800 Hz / 6 = 300 rev/s over N/FS s
        let total = 300.0 * n as f64 / FS;
        assert!(
            (full - total).abs() / total < 0.05,
            "revs {full} vs {total}"
        );
        assert!(cov > MIN_SPAN_COVER && cov <= 1.0, "coverage {cov}");
    }

    #[test]
    fn build_multi_and_revs_degenerate() {
        assert_eq!(
            build_multi(&[], FS, 6.0, RAW_MIN, RAW_MAX),
            PotLut::identity(RAW_MIN, RAW_MAX)
        );
        assert!(stitched_motor_revs(&[], FS, 6.0, RAW_MIN, RAW_MAX).is_none());
    }

    // Oversampled + jittered sweep: pos advances well under 1 count/sample (the
    // pot ADC lands many samples on each count) and dithers +/-1 count. Mirrors
    // the real capture that undercounted revs ~oversampling-fold before the
    // per-chunk pos dedup in stitch_slope.
    fn jitter_ripple_sweep(a: f64, freq: f64, n: usize) -> (Vec<u16>, Vec<f64>) {
        let mut lcg = Lcg(7);
        let pos: Vec<u16> = (0..n)
            .map(|k| {
                let f = k as f64 / (n - 1) as f64;
                let j = lcg.next(1.5).round() as i32; // -1, 0, or +1 count
                (pot_raw(f, a) as i32 + j).clamp(RAW_MIN as i32, RAW_MAX as i32) as u16
            })
            .collect();
        let cur = ripple_series(freq, 4.0, 3.0, n);
        (pos, cur)
    }

    #[test]
    fn stitched_motor_revs_survives_oversampling_and_jitter() {
        let a = 0.15;
        // ~8 samples per count over the 3700-count span
        let n = 30_000;
        let (pos, cur) = jitter_ripple_sweep(a, 1800.0, n);
        // 1800 Hz / 6 = 300 rev/s over N/FS s
        let total = 300.0 * n as f64 / FS;
        // single oversampled chunk: dedup must recover the full rev count, not
        // an oversampling-diluted fraction of it
        let single = vec![(pos.clone(), cur.clone())];
        let (full1, cov1) = stitched_motor_revs(&single, FS, 6.0, RAW_MIN, RAW_MAX).expect("revs");
        assert!(
            (full1 - total).abs() / total < 0.05,
            "single {full1} vs {total}"
        );
        assert!(cov1 > MIN_SPAN_COVER && cov1 <= 1.0, "coverage {cov1}");
        // same data split into gapped chunks
        let ranges = [
            (0, 6000),
            (6500, 12000),
            (12500, 18000),
            (18500, 24000),
            (24500, 30000),
        ];
        let chunks = chunkify(&pos, &cur, &ranges);
        let (fullc, _) = stitched_motor_revs(&chunks, FS, 6.0, RAW_MIN, RAW_MAX).expect("revs");
        assert!(
            (fullc - total).abs() / total < 0.05,
            "chunked {fullc} vs {total}"
        );
        // build_multi's normalized shape still tracks true fraction on jitter
        let lut = build_multi(&chunks, FS, 6.0, RAW_MIN, RAW_MAX);
        let mut emax = 0.0f64;
        for &(lo, hi) in ranges.iter() {
            for (j, &r) in pos[lo..hi].iter().enumerate() {
                let f = (lo + j) as f64 / (n - 1) as f64;
                emax = emax.max((lut.linearize(r) - f).abs());
            }
        }
        assert!(emax < 0.03, "linearize err {emax}");
    }
}
