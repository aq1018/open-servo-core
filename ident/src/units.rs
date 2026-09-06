//! Count -> real-world-unit conversion factors and a text report. Derives
//! amps/volts/degrees per count and the torque constants from the CalibSense
//! constants the earlier bands pinned plus the calibration outputs
//! (kinematics + motor Ke). Pure math; matches gains.rs's sense-scale
//! reasoning (both current and vmotor channels reference VDD as the ADC ref).

use core::f64::consts::PI;
use core::fmt::Write as _;

use crate::kinematics::{self, KinematicsResult};

/// Wire-readable CalibSense fields (mirrors regs::calib). tick_hz is carried
/// for completeness but no factor here needs it: firmware reports velocity in
/// counts/second, so per-tick conversion never enters (see [`derive`]).
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct SenseParams {
    pub shunt_r_mohm: u16,
    pub gain_milli: u16,
    pub vmotor_div_top: u16,
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
    pub tick_hz: u16,
}

/// Real-unit conversion factors. Non-optional factors are 0.0 on a degenerate
/// input (zero denominator) rather than an error. The velocity factor is
/// deg_per_count itself: firmware's omega_hat is counts/second, so
/// (deg/s per c/s) == deg/count with no tick_hz term - hence no separate field.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct UnitFactors {
    pub amps_per_count: f64,
    pub volts_per_count: f64,
    pub deg_per_count: f64,
    pub kt_output_ideal_nm_per_a: Option<f64>,
    pub kt_motor_nm_per_a: Option<f64>,
}

/// Derive every conversion factor. 4096 = 12-bit ADC full scale (ref = VDD),
/// 1000 = milli scaling, 100 = centi, 180/PI = deg<->rad.
pub fn derive(
    sense: &SenseParams,
    kin: &KinematicsResult,
    pos_min_phys: i32,
    pos_max_phys: i32,
    ke_vpc_q: u16,
) -> UnitFactors {
    let adc_lsb_v = sense.vdd_mv as f64 / 1000.0 / 4096.0;
    let shunt_ohm = sense.shunt_r_mohm as f64 / 1000.0;
    let gain = sense.gain_milli as f64 / 1000.0;

    let sense_denom = gain * shunt_ohm;
    let amps_per_count = if sense_denom > 0.0 {
        adc_lsb_v / sense_denom
    } else {
        0.0
    };

    let volts_per_count = if sense.vmotor_div_bot != 0 {
        let vdiv = (sense.vmotor_div_top as f64 + sense.vmotor_div_bot as f64)
            / sense.vmotor_div_bot as f64;
        adc_lsb_v * vdiv
    } else {
        0.0
    };

    let deg_per_count = kinematics::deg_per_count(
        kin.angle_min_cdeg,
        kin.angle_max_cdeg,
        pos_min_phys,
        pos_max_phys,
    );

    // ke_vpc_q is Q4.12 vcounts per OUTPUT c/s: it was fit against pot (output)
    // speed, so the gear ratio is already baked in and the OUTPUT torque
    // constant needs no explicit ratio. This is the lossless upper bound - real
    // output torque is lower by the unknown gear efficiency.
    let ke_vpc_phys = ke_vpc_q as f64 / 4096.0;
    let ke_out_v_per_cps = ke_vpc_phys * volts_per_count;
    let out_rad_per_cps = deg_per_count.abs() * PI / 180.0;
    let kt_output_ideal_nm_per_a = if out_rad_per_cps > 0.0 && ke_out_v_per_cps.is_finite() {
        let kt = ke_out_v_per_cps / out_rad_per_cps;
        kt.is_finite().then_some(kt)
    } else {
        None
    };

    // Motor-side torque constant divides the output constant back down by the
    // measured gear ratio. gear_ratio_centi == 0 is the "unset" sentinel.
    let kt_motor_nm_per_a = match kt_output_ideal_nm_per_a {
        Some(kt) if kin.gear_ratio_centi != 0 => {
            let gear_ratio = kin.gear_ratio_centi as f64 / 100.0;
            Some(kt / gear_ratio)
        }
        _ => None,
    };

    UnitFactors {
        amps_per_count,
        volts_per_count,
        deg_per_count,
        kt_output_ideal_nm_per_a,
        kt_motor_nm_per_a,
    }
}

fn opt(v: Option<f64>) -> String {
    match v {
        Some(v) => format!("{v:.6}"),
        None => "unset".into(),
    }
}

/// Render the `[units]` report section: each factor with its real unit, plus a
/// caveat that output torque is a lossless upper bound and that angle/torque
/// accuracy ride on the pot LUT and the measured gear ratio.
pub fn render(f: &UnitFactors) -> String {
    let mut s = String::new();
    render_into(&mut s, f);
    s
}

/// [`render`] appending into a caller-owned buffer.
pub fn render_into(s: &mut String, f: &UnitFactors) {
    let _ = writeln!(s, "[units]");
    let _ = writeln!(s, "  amps_per_count   {:.9} A/count", f.amps_per_count);
    let _ = writeln!(s, "  volts_per_count  {:.9} V/count", f.volts_per_count);
    let _ = writeln!(
        s,
        "  deg_per_count    {:.6} deg/count (also deg/s per c/s)",
        f.deg_per_count
    );
    let _ = writeln!(
        s,
        "  kt_output_ideal  {} N.m/A (lossless upper bound)",
        opt(f.kt_output_ideal_nm_per_a)
    );
    let _ = writeln!(s, "  kt_motor         {} N.m/A", opt(f.kt_motor_nm_per_a));
    let _ = writeln!(
        s,
        "  note: output torque is a lossless upper bound - real is lower by the\n  \
         unknown gear efficiency; angle and torque accuracy depend on the pot\n  \
         LUT and the measured gear ratio."
    );
}

#[cfg(test)]
mod tests {
    use super::*;

    fn kin(gear_ratio_centi: u16) -> KinematicsResult {
        // 190 deg over a 3800-count span -> deg_per_count = 0.05
        KinematicsResult {
            angle_min_cdeg: 0,
            angle_max_cdeg: 19000,
            gear_ratio_centi,
        }
    }

    #[test]
    fn amps_and_volts_per_count_hand_computed() {
        // vdd 3300 mV -> lsb = 3.3/4096 = 805.664 uV; shunt 0.033 ohm, G 15.0
        let sense = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };
        let f = derive(&sense, &kin(0), 100, 3900, 0);
        let lsb = 3.3 / 4096.0;
        assert!((f.amps_per_count - lsb / (15.0 * 0.033)).abs() < 1e-12);
        let vdiv = 28_200.0 / 10_000.0;
        assert!((f.volts_per_count - lsb * vdiv).abs() < 1e-12);
    }

    #[test]
    fn deg_per_count_passthrough_and_torque_derivation() {
        // Choose values so kt_output_ideal is a round number.
        // deg_per_count = 0.05 -> out_rad_per_cps = 0.05*PI/180.
        // volts_per_count = lsb*vdiv, ke_vpc_q/4096 * volts_per_count = ke_out_v.
        let sense = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };
        let ke_vpc_q = 4096; // ke_vpc_phys = 1.0
        let f = derive(&sense, &kin(15000), 100, 3900, ke_vpc_q);
        assert!((f.deg_per_count - 0.05).abs() < 1e-12);

        let lsb = 3.3 / 4096.0;
        let vpc = lsb * 28_200.0 / 10_000.0;
        let out_rad = 0.05 * PI / 180.0;
        let expect_out = vpc / out_rad;
        assert!((f.kt_output_ideal_nm_per_a.unwrap() - expect_out).abs() < 1e-9);

        // kt_motor = kt_output_ideal / (gear_ratio_centi/100)
        let gear = 15000.0 / 100.0;
        assert!((f.kt_motor_nm_per_a.unwrap() - expect_out / gear).abs() < 1e-9);
        assert!(
            (f.kt_motor_nm_per_a.unwrap() - f.kt_output_ideal_nm_per_a.unwrap() / gear).abs()
                < 1e-12
        );
    }

    #[test]
    fn kt_motor_none_when_gear_unset() {
        let sense = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };
        let f = derive(&sense, &kin(0), 100, 3900, 4096);
        assert!(f.kt_output_ideal_nm_per_a.is_some());
        assert!(f.kt_motor_nm_per_a.is_none());
    }

    #[test]
    fn degenerate_guards_no_panic() {
        let base = SenseParams {
            shunt_r_mohm: 33,
            gain_milli: 15_000,
            vmotor_div_top: 18_200,
            vmotor_div_bot: 10_000,
            vdd_mv: 3300,
            tick_hz: 20_100,
        };

        // zero shunt -> amps_per_count 0
        let f = derive(
            &SenseParams {
                shunt_r_mohm: 0,
                ..base
            },
            &kin(15000),
            100,
            3900,
            4096,
        );
        assert_eq!(f.amps_per_count, 0.0);

        // zero gain -> amps_per_count 0
        let f = derive(
            &SenseParams {
                gain_milli: 0,
                ..base
            },
            &kin(15000),
            100,
            3900,
            4096,
        );
        assert_eq!(f.amps_per_count, 0.0);

        // zero divider bottom -> volts_per_count 0 and torque None (no volts)
        let f = derive(
            &SenseParams {
                vmotor_div_bot: 0,
                ..base
            },
            &kin(15000),
            100,
            3900,
            4096,
        );
        // zero volts scale propagates to a zero torque constant (the division
        // denominator out_rad is still finite, so it is Some(0.0), not None)
        assert_eq!(f.volts_per_count, 0.0);
        assert_eq!(f.kt_output_ideal_nm_per_a, Some(0.0));
        assert_eq!(f.kt_motor_nm_per_a, Some(0.0));

        // zero deg_per_count (zero phys span) -> torque None
        let f = derive(&base, &kin(15000), 512, 512, 4096);
        assert_eq!(f.deg_per_count, 0.0);
        assert!(f.kt_output_ideal_nm_per_a.is_none());
        assert!(f.kt_motor_nm_per_a.is_none());
    }

    #[test]
    fn render_shows_section_and_unset() {
        let f = UnitFactors {
            amps_per_count: 1.6e-3,
            volts_per_count: 2.27e-3,
            deg_per_count: 0.05,
            kt_output_ideal_nm_per_a: Some(0.5),
            kt_motor_nm_per_a: None,
        };
        let s = render(&f);
        assert!(s.contains("[units]"));
        assert!(s.contains("amps_per_count"));
        assert!(s.contains("kt_motor"));
        assert!(
            s.contains("unset"),
            "kt_motor None should render unset:\n{s}"
        );
        assert!(s.contains("lossless upper bound"));
    }
}
