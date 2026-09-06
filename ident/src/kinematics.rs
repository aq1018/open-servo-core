//! Count <-> real-world-unit resolution for the calibration band. Turns the
//! endstop sweep (pos_min/max_phys, which equal the pot LUT endpoints
//! raw_min/raw_max) plus the human-supplied travel angles into the on-servo
//! `CalibKinematics` values: angle endpoints in centi-degrees and the
//! physical gear ratio. Pure math; the caller writes the encoded values to
//! the three calib regs in a later commit.

use crate::ripple;

/// Encoded kinematics, ready for the three calib regs. All-zero is the
/// firmware's "unset" sentinel.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct KinematicsResult {
    pub angle_min_cdeg: i16,
    pub angle_max_cdeg: i16,
    pub gear_ratio_centi: u16,
}

/// Degrees of output travel per pot count over the phys span. 0.0 on a
/// zero/non-finite span.
pub fn deg_per_count(
    angle_min_cdeg: i16,
    angle_max_cdeg: i16,
    pos_min_phys: i32,
    pos_max_phys: i32,
) -> f64 {
    let count_span = pos_max_phys as f64 - pos_min_phys as f64;
    if count_span == 0.0 {
        return 0.0;
    }
    let angle_span_deg = (angle_max_cdeg as f64 - angle_min_cdeg as f64) / 100.0;
    angle_span_deg / count_span
}

/// Encode the human's angle at each phys end into centi-degree endpoints.
/// angle_at_min pairs with pos_min_phys, angle_at_max with pos_max_phys.
pub fn angle_endpoints(angle_at_min_deg: f64, angle_at_max_deg: f64) -> (i16, i16) {
    (
        cdeg_saturating(angle_at_min_deg),
        cdeg_saturating(angle_at_max_deg),
    )
}

/// Physical gear ratio (motor revs per output rev) x100, saturating to u16.
/// 0 on degenerate input (zero pot speed, zero/non-finite deg/count).
pub fn gear_ratio_centi(motor_rev_s: f64, pot_omega_cps: f64, deg_per_count: f64) -> u16 {
    if pot_omega_cps == 0.0 || deg_per_count == 0.0 || !deg_per_count.is_finite() {
        return 0;
    }
    // counts per full output rev closes ripple::gear_ratio_check's relative
    // ratio to the physical one (360 deg/rev)
    let counts_per_output_rev = 360.0 / deg_per_count;
    let ratio = ripple::gear_ratio_check(motor_rev_s, pot_omega_cps, counts_per_output_rev);
    let centi = (ratio.abs() * 100.0).round();
    if !centi.is_finite() {
        return 0;
    }
    centi.min(u16::MAX as f64) as u16
}

/// Total motor revs over a FULL rail-to-rail traverse, extrapolated from a
/// partial (soft-bounded) sweep via the per-count rate: anchor-free geometry,
/// so a run turning `motor_revs_run` over `pot_disp` counts scales to the whole
/// `count_span`. 0.0 when the sweep covered no counts or the span is non-finite.
pub fn full_span_motor_revs(motor_revs_run: f64, pot_disp: f64, count_span: f64) -> f64 {
    if pot_disp > 0.0 && count_span.is_finite() {
        motor_revs_run * count_span / pot_disp
    } else {
        0.0
    }
}

/// Output travel (deg) implied by a counted gear ratio and the full-traverse
/// motor revs: `motor_revs_full * 360 / gear_ratio`. The inverse of the
/// travel-anchored gear derivation. 0.0 on a zero/non-finite gear ratio.
pub fn travel_deg_from_gear(motor_revs_full: f64, gear_ratio: f64) -> f64 {
    if gear_ratio > 0.0 && gear_ratio.is_finite() {
        motor_revs_full * 360.0 / gear_ratio
    } else {
        0.0
    }
}

/// Assemble the three encoded values from the endstop sweep and the
/// human-supplied travel angles.
pub fn resolve(
    pos_min_phys: i32,
    pos_max_phys: i32,
    angle_at_min_deg: f64,
    angle_at_max_deg: f64,
    motor_rev_s: f64,
    pot_omega_cps: f64,
) -> KinematicsResult {
    let (angle_min_cdeg, angle_max_cdeg) = angle_endpoints(angle_at_min_deg, angle_at_max_deg);
    let dpc = deg_per_count(angle_min_cdeg, angle_max_cdeg, pos_min_phys, pos_max_phys);
    KinematicsResult {
        angle_min_cdeg,
        angle_max_cdeg,
        gear_ratio_centi: gear_ratio_centi(motor_rev_s, pot_omega_cps, dpc),
    }
}

fn cdeg_saturating(deg: f64) -> i16 {
    if !deg.is_finite() {
        return 0;
    }
    (deg * 100.0)
        .round()
        .clamp(i16::MIN as f64, i16::MAX as f64) as i16
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn endpoints_and_deg_per_count_round_trip() {
        // 190 deg of travel over a 3900-count phys span
        let (lo, hi) = angle_endpoints(0.0, 190.0);
        assert_eq!((lo, hi), (0, 19000));
        let dpc = deg_per_count(lo, hi, 100, 4000);
        assert!((dpc - 190.0 / 3900.0).abs() < 1e-12, "dpc {dpc}");
    }

    #[test]
    fn cdeg_rounds_and_saturates() {
        assert_eq!(cdeg_saturating(12.345), 1235);
        assert_eq!(cdeg_saturating(-12.345), -1235);
        assert_eq!(cdeg_saturating(400.0), i16::MAX);
        assert_eq!(cdeg_saturating(-400.0), i16::MIN);
        assert_eq!(cdeg_saturating(f64::NAN), 0);
    }

    #[test]
    fn deg_per_count_degenerate_span_is_zero() {
        assert_eq!(deg_per_count(0, 19000, 512, 512), 0.0);
    }

    #[test]
    fn gear_ratio_from_synthetic_inputs() {
        // dpc 0.05 deg/count, pot 4000 cps -> output 0.5556 rev/s;
        // motor 83.333 rev/s implies 150:1
        let g = gear_ratio_centi(83.333_333, 4000.0, 0.05);
        assert!((g as i32 - 15000).abs() <= 1, "gear {g}");
    }

    #[test]
    fn gear_ratio_degenerate_is_zero() {
        assert_eq!(gear_ratio_centi(83.0, 0.0, 0.05), 0);
        assert_eq!(gear_ratio_centi(83.0, 4000.0, 0.0), 0);
        assert_eq!(gear_ratio_centi(83.0, 4000.0, f64::NAN), 0);
    }

    #[test]
    fn gear_ratio_saturates_to_u16() {
        assert_eq!(gear_ratio_centi(1e9, 1.0, 0.05), u16::MAX);
    }

    #[test]
    fn travel_from_gear_round_trips_through_gear_ratio_centi() {
        // gear 150, motor_revs_full 79.1667 -> travel 190 deg over a 3900-count
        // span; feeding count_span as pot speed and travel/count_span as dpc
        // recovers the gear (gear_ratio_centi = motor_revs_full*360/travel).
        let motor_revs_full = 79.166_667;
        let gear = 150.0;
        let count_span = 3900.0;
        let travel = travel_deg_from_gear(motor_revs_full, gear);
        assert!((travel - 190.0).abs() < 1e-3, "travel {travel}");
        let dpc = travel / count_span;
        let g = gear_ratio_centi(motor_revs_full, count_span, dpc);
        assert!((g as f64 / 100.0 - gear).abs() < 0.02, "gear {g}");
    }

    #[test]
    fn full_span_scales_by_count_ratio() {
        // 20 motor revs over 1000 counts of a 4000-count span -> 80 full revs
        assert!((full_span_motor_revs(20.0, 1000.0, 4000.0) - 80.0).abs() < 1e-9);
    }

    #[test]
    fn full_span_and_travel_degenerate_guards() {
        assert_eq!(full_span_motor_revs(20.0, 0.0, 4000.0), 0.0);
        assert_eq!(full_span_motor_revs(20.0, 1000.0, f64::INFINITY), 0.0);
        assert_eq!(full_span_motor_revs(20.0, 1000.0, f64::NAN), 0.0);
        assert_eq!(travel_deg_from_gear(80.0, 0.0), 0.0);
        assert_eq!(travel_deg_from_gear(80.0, -1.0), 0.0);
        assert_eq!(travel_deg_from_gear(80.0, f64::NAN), 0.0);
        assert_eq!(travel_deg_from_gear(80.0, f64::INFINITY), 0.0);
    }

    #[test]
    fn resolve_assembles_all_three() {
        let r = resolve(100, 4000, 0.0, 190.0, 83.333_333, 4000.0);
        assert_eq!(r.angle_min_cdeg, 0);
        assert_eq!(r.angle_max_cdeg, 19000);
        // dpc = 190/3900 = 0.048718; output = 4000*dpc/360; gear ~ 154
        let dpc: f64 = 190.0 / 3900.0;
        let expect = (83.333_333 / (4000.0 * dpc / 360.0) * 100.0).round() as u16;
        assert_eq!(r.gear_ratio_centi, expect);
    }
}
