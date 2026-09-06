//! Fixed-gain 3-state fusion observer (control-theory "The Fusion Filter").
//! Predict pushes theta/omega through the mechanical model - b_i_q313 =
//! round(B * 8192) where B bakes Kt*Ts/J as (c/s per medium tick) per
//! ccount (rig-measured B runs ~3.4, so Q3.13 holds it with headroom
//! where Q0.16 saturated), and the shift-0 product << 3 couples current
//! counts into csQ16 - and correct nudges all three states against the
//! measured pot position. The third state is the
//! disturbance torque the model cannot explain, in current counts; it feeds
//! stall/collision detection and telemetry. Gains are host-synthesized
//! constants (no runtime matrix math on this chip).

use crate::math::q_mul;

/// Predict skips the Coulomb term below 1 c/s: at rest omega dithers around
/// zero by sub-count amounts, and a sign-chattering sgn(omega)*fric_fc would
/// inject phantom +-fric_fc drive into every predict.
const FRIC_OMEGA_EPS_CSQ16: i32 = 1 << 16;

/// theta is NOT clamped to the pot span - it must track the measurement
/// freely so e stays honest past the ends. The bound only guards i32: pot
/// tops at 4095<<16 < 2^28, so |pos<<16 - theta| <= 2^28 + 2^29 fits i32.
const THETA_LIM_CQ16: i32 = 1 << 29;

/// Innovation clamp, 128 counts. The worst correct-step product is a Q8.8
/// gain at u16::MAX: 65535 * 2^23 >> 8 = 65535 << 15 < i32::MAX, so no
/// q_mul result wraps for any gain encoding; also bounds glitch response.
const E_LIM_CQ16: i32 = 1 << 23;

/// i16 c/s full scale in csQ16 (SG90 tops ~9000 c/s, 3.6x headroom).
const OMEGA_LIM_CSQ16: i32 = 32767 << 16;

/// Full shunt scale in ccQ16 - a disturbance beyond +-4095 current counts
/// is not resolvable by the model anyway.
const TAU_D_LIM_CCQ16: i32 = 4095 << 16;

/// Model-input clamp, 2x shunt full scale: 65535 * 8192 < 2^31 keeps the
/// shift-0 b_i product i32-exact for any gain encoding; only the << 3 to
/// csQ16 saturates (velocity.rs shift discipline).
const ACCEL_LIM_CC: i32 = 8192;

/// CALIB motor (b_i, fric_fc) + CONFIG fusion correction gains, loaded fresh
/// each step by the kernel.
#[derive(Copy, Clone)]
pub struct FusionGains {
    pub b_i_q313: u16,
    pub l1_q016: u16,
    pub l2_q88: u16,
    pub l3_q88: u16,
    pub l_bemf_q016: u16,
    pub fric_fc_counts: u16,
}

fn fric_c(omega_q16: i32, fric_fc_counts: u16) -> i32 {
    if omega_q16 > FRIC_OMEGA_EPS_CSQ16 {
        fric_fc_counts as i32
    } else if omega_q16 < -FRIC_OMEGA_EPS_CSQ16 {
        -(fric_fc_counts as i32)
    } else {
        0
    }
}

/// States: theta cQ16 (pot counts), omega csQ16, tau_d ccQ16.
#[derive(Default)]
pub struct FusionObs {
    theta_q16: i32,
    omega_q16: i32,
    tau_d_q16: i32,
}

impl FusionObs {
    pub const fn new() -> Self {
        Self {
            theta_q16: 0,
            omega_q16: 0,
            tau_d_q16: 0,
        }
    }

    /// Reset to the measurement. Kernel calls at install and on the
    /// torque-enable edge so stale states never kick a fresh enable.
    pub fn seed(&mut self, pos_meas: u16) {
        self.theta_q16 = (pos_meas as i32) << 16;
        self.omega_q16 = 0;
        self.tau_d_q16 = 0;
    }

    /// One MEDIUM-tick predict+correct. `i_counts` is i_use, already
    /// resolved by the caller: i_meas when the shunt window is valid, else
    /// i_ref - the observer never sees the validity flag. `dt_med_q32` =
    /// 2^32 / MED_HZ (MED_HZ >= 2 keeps it under 2^31, so the i32 cast is
    /// value-preserving).
    pub fn step(
        &mut self,
        i_counts: i32,
        pos_meas: u16,
        omega_bemf_cps_q16: Option<i32>,
        dt_med_q32: u32,
        gains: &FusionGains,
    ) {
        // Predict. b_i is Q3.13 of B (c/s per ccount per tick), so the
        // shift-0 product lands in csQ13; the << 3 to csQ16 saturates only
        // beyond omega full scale (ACCEL_LIM_CC keeps the product itself
        // i32-exact). Saturating subs guard a hostile i_counts, everything
        // downstream is clamp-bounded.
        let fric = fric_c(self.omega_q16, gains.fric_fc_counts);
        let accel = i_counts
            .saturating_sub(fric)
            .saturating_sub(self.tau_d_q16 >> 16)
            .clamp(-ACCEL_LIM_CC, ACCEL_LIM_CC);
        self.omega_q16 = self
            .omega_q16
            .saturating_add(q_mul(gains.b_i_q313 as i32, accel, 0).saturating_mul(1 << 3))
            .clamp(-OMEGA_LIM_CSQ16, OMEGA_LIM_CSQ16);
        // |omega| <= 2^31, dt < 2^31 -> |delta| < 2^30
        self.theta_q16 = self
            .theta_q16
            .saturating_add(q_mul(self.omega_q16, dt_med_q32 as i32, 32))
            .clamp(-THETA_LIM_CQ16, THETA_LIM_CQ16);

        // Correct. Plain sub is safe: 2^28 + 2^29 < 2^31 (theta clamp).
        let e = (((pos_meas as i32) << 16) - self.theta_q16).clamp(-E_LIM_CQ16, E_LIM_CQ16);
        self.theta_q16 = self
            .theta_q16
            .saturating_add(q_mul(gains.l1_q016 as i32, e, 16))
            .clamp(-THETA_LIM_CQ16, THETA_LIM_CQ16);
        self.omega_q16 = self
            .omega_q16
            .saturating_add(q_mul(gains.l2_q88 as i32, e, 8))
            .clamp(-OMEGA_LIM_CSQ16, OMEGA_LIM_CSQ16);
        self.tau_d_q16 = self
            .tau_d_q16
            .saturating_sub(q_mul(gains.l3_q88 as i32, e, 8))
            .clamp(-TAU_D_LIM_CCQ16, TAU_D_LIM_CCQ16);

        // bemf blend; config defaults l_bemf 0 = off until bench-validated.
        if gains.l_bemf_q016 > 0
            && let Some(w) = omega_bemf_cps_q16
        {
            let e_w = w.saturating_sub(self.omega_q16);
            self.omega_q16 = self
                .omega_q16
                .saturating_add(q_mul(gains.l_bemf_q016 as i32, e_w, 16))
                .clamp(-OMEGA_LIM_CSQ16, OMEGA_LIM_CSQ16);
        }
    }

    pub fn theta_q16(&self) -> i32 {
        self.theta_q16
    }

    pub fn omega_q16(&self) -> i32 {
        self.omega_q16
    }

    /// Whole current counts. The state clamp already bounds to +-4095; the
    /// i16 clamp is the saturating ABI cast.
    pub fn tau_d_counts(&self) -> i16 {
        (self.tau_d_q16 >> 16).clamp(i16::MIN as i32, i16::MAX as i32) as i16
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // 2 kHz MEDIUM rate, matching the kernel's DT_MED_Q32 derivation.
    const DT: u32 = ((1u64 << 32) / 2000) as u32;

    // Hand-picked stable set for the 2 kHz discrete observer: b_i = 819 =
    // Q3.13 of B = 0.1 c/s per tick per ccount (low rig-physical range;
    // the rig measures ~3.4), l1 = 0.25, l2 = 4.0 c/s per count, l3 = 8.0
    // cc per count. With the coupling live, the e -> tau_d -> omega ->
    // theta loop gain scales as l3 * B, so l3 must shrink as b_i grows:
    // 64.0 rails the filter at this B (integer-sim verified), 8.0 sits
    // inside the stable region with the disturbance step still settling
    // exactly. If a convergence test oscillates the fix is smaller gains,
    // not more iterations.
    const G: FusionGains = FusionGains {
        b_i_q313: 819,
        l1_q016: 16384,
        l2_q88: 1024,
        l3_q88: 2048,
        l_bemf_q016: 0,
        fric_fc_counts: 0,
    };

    #[test]
    fn seed_identity() {
        let mut f = FusionObs::new();
        f.seed(1234);
        assert_eq!(f.theta_q16(), 1234 << 16);
        assert_eq!(f.omega_q16(), 0);
        assert_eq!(f.tau_d_counts(), 0);
    }

    #[test]
    fn static_convergence() {
        // Seed 10 counts below the measurement, zero current: theta locks
        // to the measurement, omega and tau_d bleed back to ~0 (the live
        // b_i bleed path settles by ~10000 ticks; 40000 keeps margin).
        // Residuals below the quantization floors (omega under ~2000 q16
        // moves theta by 0 per tick: q_mul(omega, DT, 32) truncates
        // omega/2000 to zero), pinned exact.
        let mut f = FusionObs::new();
        f.seed(1990);
        for _ in 0..40000 {
            f.step(0, 2000, None, DT, &G);
        }
        assert_eq!(f.theta_q16(), 2000 << 16, "pin");
        assert_eq!(f.omega_q16(), 1964, "pin");
        assert_eq!(f.tau_d_counts(), 0, "pin");
    }

    #[test]
    fn current_couples_at_full_scale() {
        // One step from rest, e = 0 going in: predict alone moves omega by
        // q_mul(b_i, i, 0) << 3 = 819 * 100 * 8 = 655200 q16 (~10 c/s) -
        // the Q3.13 encoding couples whole ccounts into csQ16 through the
        // << 3 (the old shift-16 form moved <= 1 q16 per ccount and left
        // b_i inert). The correct step then subtracts l2*e for the 327-q16
        // theta advance: 655200 - (1024 * 327 >> 8) = 653892.
        let mut f = FusionObs::new();
        f.seed(2000);
        f.step(100, 2000, None, DT, &G);
        assert_eq!(f.omega_q16(), 653892, "pin");
    }

    #[test]
    fn constant_velocity_tracking() {
        // pos ramps 1 count/tick = 2000 c/s; omega settles onto the ramp
        // rate (measured residual ~0.25%, assert 1%).
        let mut f = FusionObs::new();
        f.seed(0);
        for n in 1..=3000u16 {
            f.step(0, n, None, DT, &G);
        }
        let target = 2000i32 << 16;
        let err = (f.omega_q16() - target).abs();
        assert!(err <= target / 100, "omega={} err={}", f.omega_q16(), err);
    }

    #[test]
    fn disturbance_step() {
        // Constant drive current with the measurement pinned: predict keeps
        // pushing theta up, e goes negative, so tau_d -= l3*e rises until
        // tau_d ~= i (a load exactly absorbing the drive), while the
        // correct step keeps theta/omega anchored to the measurement.
        let mut f = FusionObs::new();
        f.seed(2000);
        for _ in 0..20000 {
            f.step(500, 2000, None, DT, &G);
        }
        // the live bleed path settles tau_d onto the drive exactly
        assert_eq!(f.tau_d_counts(), 500, "pin");
        let theta_err = (f.theta_q16() - (2000 << 16)).abs();
        assert!(theta_err < 1 << 16, "theta={}", f.theta_q16());
        assert!(f.omega_q16().abs() < 1 << 16, "omega={}", f.omega_q16());
    }

    #[test]
    fn friction_zero_band() {
        // At rest with a large Coulomb term configured, nothing moves:
        // fric_c(0) = 0, e = 0, all deltas exactly zero.
        let g = FusionGains {
            fric_fc_counts: 1000,
            ..G
        };
        let mut f = FusionObs::new();
        f.seed(2048);
        for _ in 0..100 {
            f.step(0, 2048, None, DT, &g);
            assert_eq!(f.theta_q16(), 2048 << 16);
            assert_eq!(f.omega_q16(), 0);
            assert_eq!(f.tau_d_counts(), 0);
        }
    }

    #[test]
    fn bemf_blend_off_at_zero_gain() {
        let mut with = FusionObs::new();
        let mut without = FusionObs::new();
        with.seed(1000);
        without.seed(1000);
        for n in 0..500u16 {
            with.step(200, 1000 + n, Some(5000 << 16), DT, &G);
            without.step(200, 1000 + n, None, DT, &G);
            assert_eq!(with.theta_q16(), without.theta_q16());
            assert_eq!(with.omega_q16(), without.omega_q16());
            assert_eq!(with.tau_d_counts(), without.tau_d_counts());
        }
    }

    #[test]
    fn bemf_blend_pulls_omega() {
        let g = FusionGains {
            l_bemf_q016: 16384,
            ..G
        };
        let mut f = FusionObs::new();
        f.seed(2000);
        f.step(0, 2000, Some(1000 << 16), DT, &g);
        assert!(f.omega_q16() > 0, "omega={}", f.omega_q16());
    }

    #[test]
    fn clamps_under_hostile_gains() {
        // Max-encoded gains, extreme inputs: debug overflow checks are the
        // wrap detector; states must stay inside their clamps.
        let g = FusionGains {
            b_i_q313: u16::MAX,
            l1_q016: u16::MAX,
            l2_q88: u16::MAX,
            l3_q88: u16::MAX,
            l_bemf_q016: u16::MAX,
            fric_fc_counts: u16::MAX,
        };
        let mut f = FusionObs::new();
        for n in 0..2000 {
            let pos = if n & 1 == 0 { 0 } else { 4095 };
            let i = if n & 2 == 0 { i32::MAX } else { i32::MIN };
            let w = Some(if n & 4 == 0 { i32::MAX } else { i32::MIN });
            f.step(i, pos, w, DT, &g);
            assert!(f.theta_q16().abs() <= THETA_LIM_CQ16);
            assert!(f.omega_q16().abs() <= OMEGA_LIM_CSQ16);
            assert!((f.tau_d_q16).abs() <= TAU_D_LIM_CCQ16);
        }
    }
}
