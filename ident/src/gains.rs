//! Fitted plant parameters -> loop and fusion gains -> table Q encodings.
//! Pole placement runs in f64; every encoder reports what quantization did
//! to it. The integer verification suite in tests mirrors the firmware
//! updates (current.rs / velocity.rs / fusion.rs) and is the authority on
//! stability - the placement formulas are only as good as those runs.
//!
//! Q formats (verified against firmware/lib/core, Q3.13 fusion coupling):
//! i_kp_q88 vcounts/ccount; i_ki/i_kaw_q412 per FAST tick; v_kp_q88
//! ccounts per c/s; v_ki/v_kaw_q412 per MEDIUM tick; j_ff_q88 = 256/B;
//! p_kp_q88 c/s per count; l1_q016; l2_q88 c/s per count; l3_q88 cc per
//! count; b_i_q313 = round(B*8192) (rig B runs ~3.4, Q0.16 saturated);
//! r_q12/ke_vpc_q Q4.12; recip_ke_q Q6.10; fric_fv_q016 Q0.16;
//! fric_fc/breakaway whole ccounts.

/// Bench model-card inductance (SG90 clone donor motor), henries. L is not
/// identifiable from the fixed-phase shunt sampling - this is the CLI
/// `--l-henries` default.
pub const DEFAULT_L_HENRIES: f64 = 0.5e-3;

/// Anti-hunt deadband policy. The deadband must exceed pot-noise excursions
/// or the hold chatters (bench: 16 counts held clean at sigma 6.9 = 2.3
/// sigma). Hold coasts on position rest alone (firmware position.rs), so the
/// only synthesized field is the deadband; the floor keeps a suspiciously
/// quiet E0 from synthesizing a deadband too tight to engage.
const DEADBAND_SIGMA_MULT: f64 = 2.5;
const DEADBAND_FLOOR_COUNTS: f64 = 4.0;

/// SI inductance -> count domain. vcounts and ccounts share the ADC lsb
/// (both channels reference VDD), so it cancels and only the front-end
/// scales remain:
///
///   L_cd [vcount*s/ccount] = L_H * div / (R_shunt * G)
///   div = bot / (top + bot), R_shunt = shunt_r_mohm / 1000, G = gain_milli / 1000
///
/// Sanity: the same map takes the rig's 4.7 ohm to r_vpc ~3.37, matching
/// the E2 fit - the four inputs are wire-readable CalibSense fields.
pub fn l_cd_from_si(
    l_henries: f64,
    shunt_r_mohm: u16,
    gain_milli: u16,
    div_top: u16,
    div_bot: u16,
) -> Option<f64> {
    let shunt = shunt_r_mohm as f64 / 1000.0;
    let gain = gain_milli as f64 / 1000.0;
    let denom = (div_top as f64) + (div_bot as f64);
    if shunt <= 0.0 || gain <= 0.0 || denom <= 0.0 || div_bot == 0 {
        return None;
    }
    Some(l_henries * (div_bot as f64 / denom) / (shunt * gain))
}

/// Loop bandwidth targets, Hz. Defaults per the band plan; E5/E6 verify.
#[derive(Copy, Clone, Debug)]
pub struct BwTargets {
    pub f_ci: f64,
    pub f_cv: f64,
    pub f_cp: f64,
    pub f_o: f64,
}

impl Default for BwTargets {
    fn default() -> Self {
        Self {
            f_ci: 1000.0,
            f_cv: 200.0,
            f_cp: 25.0,
            f_o: 15.0,
        }
    }
}

/// Identified plant, count domain: R and Ke as fitted (vcounts/ccount,
/// vcounts per c/s), friction line, B ((c/s per medium tick) per ccount),
/// pot noise sigma (counts), inductance via [`l_cd_from_si`], and the
/// tick rates the encodings are anchored to.
#[derive(Copy, Clone, Debug)]
pub struct PlantParams {
    pub r_vpc: f64,
    pub ke_vpc: f64,
    pub fc: f64,
    pub fv: f64,
    pub b: f64,
    pub sigma_theta: f64,
    pub l_cd: f64,
    pub tick_hz: f64,
    pub f_med: f64,
}

/// Synthesized gains, physical units (the Q encoding is [`encode`]'s job).
#[derive(Copy, Clone, Debug)]
pub struct GainSet {
    pub i_kp: f64,
    pub i_ki: f64,
    pub i_kaw: f64,
    pub v_kp: f64,
    pub v_ki: f64,
    pub v_kaw: f64,
    pub j_ff: f64,
    pub p_kp: f64,
    pub l1: f64,
    pub l2: f64,
    pub l3: f64,
    pub b: f64,
    pub r_vpc: f64,
    pub ke_vpc: f64,
    pub recip_ke: f64,
    pub fric_fc: f64,
    pub fric_fv: f64,
    pub fric_breakaway: f64,
    /// Projected omega noise from the pot, c/s: l2 * sigma_theta per
    /// correct step - report fodder, not a table field.
    pub omega_noise_cps: f64,
    /// Anti-hunt park window, counts: noise-derived lower bound (an
    /// application may widen it for a softer, quieter hold).
    pub pos_deadband: f64,
}

/// Pole placement.
///
/// Current PI, pole-zero cancel on the R + L*s plant (current.rs): kp =
/// w_ci * L_cd cancels the electrical pole, ki = w_ci * R / tick_hz per
/// fast tick sets the crossover, kaw = 2 * ki (current.rs back-calc
/// convention, matching the bench-proven kaw/ki = 2 ratio of the tests).
///
/// Velocity PI on the integrator plant domega/dtick = K * i, K = B (per
/// medium tick): kp = w_cv / (B * f_med) ccounts per c/s, ki = kp * w_cv /
/// (4 * f_med) per medium tick (zero a quarter-decade under crossover),
/// kaw = 2 * ki, j_ff = 1/B.
///
/// Position P: p_kp = 2*pi*f_cp c/s per count (position.rs Q8.8 map).
///
/// Fusion l1/l2/l3, coincident poles at p = exp(-2*pi*f_o / f_med). The
/// fusion.rs update is the alpha-beta-gamma filter with tau_d as a scaled
/// acceleration state: predict does omega += B*(i - fric - tau), theta +=
/// omega*dt, so tau maps to the classic accel state a = -B*tau/dt. The
/// standard critically-damped placement corrects theta += alpha*e, v +=
/// (beta/dt)*e, a += (gamma/(2*dt^2))*e with alpha = 1 - p^3, beta =
/// 1.5*(1-p)^2*(1+p), gamma = (1-p)^3. Mapping to the table gains (dt =
/// 1/f_med; the tau_d correction reaches omega only through the predict's
/// -B*tau term, so a = -B*tau/dt and tau -= l3*e is a += (B*l3/dt)*e):
///
///   l1 = alpha = 1 - p^3
///   l2 = beta / dt = 1.5 * (1-p)^2 * (1+p) * f_med     [c/s per count]
///   l3 = gamma / (2 * dt * B) = (1-p)^3 * f_med / (2*B) [cc per count]
///
/// The theta predict omits the a*dt^2/2 half-step - a structural deviation
/// the integer verification suite absorbs. l_bemf = 0 (blend off in v1).
pub fn synthesize(p: &PlantParams, t: &BwTargets) -> GainSet {
    let w_ci = core::f64::consts::TAU * t.f_ci;
    let w_cv = core::f64::consts::TAU * t.f_cv;
    let i_ki = w_ci * p.r_vpc / p.tick_hz;
    let k_vel = p.b * p.f_med;
    let v_kp = w_cv / k_vel;
    let v_ki = v_kp * w_cv / (4.0 * p.f_med);
    let pole = (-core::f64::consts::TAU * t.f_o / p.f_med).exp();
    let q = 1.0 - pole;
    let l2 = 1.5 * q * q * (1.0 + pole) * p.f_med;
    let l3 = q * q * q * p.f_med / (2.0 * p.b);
    GainSet {
        i_kp: w_ci * p.l_cd,
        i_ki,
        i_kaw: 2.0 * i_ki,
        v_kp,
        v_ki,
        v_kaw: 2.0 * v_ki,
        j_ff: 1.0 / p.b,
        p_kp: core::f64::consts::TAU * t.f_cp,
        l1: 1.0 - pole * pole * pole,
        l2,
        l3,
        b: p.b,
        r_vpc: p.r_vpc,
        ke_vpc: p.ke_vpc,
        recip_ke: if p.ke_vpc > 0.0 { 1.0 / p.ke_vpc } else { 0.0 },
        fric_fc: p.fc,
        fric_fv: p.fv,
        fric_breakaway: 0.0,
        omega_noise_cps: l2 * p.sigma_theta,
        pos_deadband: (DEADBAND_SIGMA_MULT * p.sigma_theta).max(DEADBAND_FLOOR_COUNTS),
    }
}

/// One encoded table field: the physical value asked for, the raw u16 the
/// table gets, the relative error quantization introduced, and whether the
/// encoding clamped (never wraps).
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct Encoded {
    pub physical: f64,
    pub raw: u16,
    pub quantization_pct: f64,
    pub saturated: bool,
}

/// physical * scale -> u16 with round-to-nearest, clamp at the u16 rail.
/// Negative physicals clamp to 0 flagged (every table gain is unsigned).
fn enc(physical: f64, scale: f64) -> Encoded {
    let ideal = physical * scale;
    let (raw, saturated) = if !ideal.is_finite() || ideal < 0.0 {
        (0u16, ideal != 0.0)
    } else if ideal > u16::MAX as f64 {
        (u16::MAX, true)
    } else {
        (ideal.round() as u16, false)
    };
    let decoded = raw as f64 / scale;
    let quantization_pct = if physical != 0.0 {
        (decoded - physical).abs() / physical.abs() * 100.0
    } else {
        0.0
    };
    Encoded {
        physical,
        raw,
        quantization_pct,
        saturated,
    }
}

/// The full encoded write set, one [`Encoded`] per table field.
#[derive(Copy, Clone, Debug)]
pub struct EncodedGains {
    pub i_kp_q88: Encoded,
    pub i_ki_q412: Encoded,
    pub i_kaw_q412: Encoded,
    pub v_kp_q88: Encoded,
    pub v_ki_q412: Encoded,
    pub v_kaw_q412: Encoded,
    pub j_ff_q88: Encoded,
    pub p_kp_q88: Encoded,
    pub pos_deadband_counts: Encoded,
    pub l1_q016: Encoded,
    pub l2_q88: Encoded,
    pub l3_q88: Encoded,
    pub b_i_q313: Encoded,
    pub r_q12: Encoded,
    pub ke_vpc_q: Encoded,
    pub recip_ke_q: Encoded,
    pub fric_fc_counts: Encoded,
    pub fric_fv_q016: Encoded,
    pub fric_breakaway_counts: Encoded,
}

impl EncodedGains {
    /// (name, field) pairs in table-write order - the report and the CLI
    /// write-back both iterate this.
    pub fn fields(&self) -> [(&'static str, Encoded); 19] {
        [
            ("r_q12", self.r_q12),
            ("ke_vpc_q", self.ke_vpc_q),
            ("recip_ke_q", self.recip_ke_q),
            ("b_i_q313", self.b_i_q313),
            ("fric_fc_counts", self.fric_fc_counts),
            ("fric_fv_q016", self.fric_fv_q016),
            ("fric_breakaway_counts", self.fric_breakaway_counts),
            ("i_kp_q88", self.i_kp_q88),
            ("i_ki_q412", self.i_ki_q412),
            ("i_kaw_q412", self.i_kaw_q412),
            ("v_kp_q88", self.v_kp_q88),
            ("v_ki_q412", self.v_ki_q412),
            ("v_kaw_q412", self.v_kaw_q412),
            ("j_ff_q88", self.j_ff_q88),
            ("p_kp_q88", self.p_kp_q88),
            ("pos_deadband_counts", self.pos_deadband_counts),
            ("l1_q016", self.l1_q016),
            ("l2_q88", self.l2_q88),
            ("l3_q88", self.l3_q88),
        ]
    }
}

pub fn encode(g: &GainSet) -> EncodedGains {
    EncodedGains {
        i_kp_q88: enc(g.i_kp, 256.0),
        i_ki_q412: enc(g.i_ki, 4096.0),
        i_kaw_q412: enc(g.i_kaw, 4096.0),
        v_kp_q88: enc(g.v_kp, 256.0),
        v_ki_q412: enc(g.v_ki, 4096.0),
        v_kaw_q412: enc(g.v_kaw, 4096.0),
        j_ff_q88: enc(g.j_ff, 256.0),
        p_kp_q88: enc(g.p_kp, 256.0),
        pos_deadband_counts: enc(g.pos_deadband, 1.0),
        l1_q016: enc(g.l1, 65536.0),
        l2_q88: enc(g.l2, 256.0),
        l3_q88: enc(g.l3, 256.0),
        b_i_q313: enc(g.b, 8192.0),
        r_q12: enc(g.r_vpc, 4096.0),
        ke_vpc_q: enc(g.ke_vpc, 4096.0),
        recip_ke_q: enc(g.recip_ke, 1024.0),
        fric_fc_counts: enc(g.fric_fc, 1.0),
        fric_fv_q016: enc(g.fric_fv, 65536.0),
        fric_breakaway_counts: enc(g.fric_breakaway, 1.0),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Exact firmware q_mul (math.rs): i64 widen, arithmetic shift, i32
    /// truncate.
    fn q_mul(a: i32, b: i32, k: u32) -> i32 {
        ((a as i64 * b as i64) >> k) as i32
    }

    fn rig_plant() -> PlantParams {
        PlantParams {
            r_vpc: 3.37,
            ke_vpc: 0.2,
            fc: 20.0,
            fv: 0.001,
            b: 0.1,
            sigma_theta: 1.0,
            l_cd: 3.58e-4,
            tick_hz: 20_100.0,
            f_med: 2_010.0,
        }
    }

    #[test]
    fn l_cd_from_si_matches_the_r_map() {
        // The same scale map must take 4.7 ohm to the fitted ~3.37
        // vcounts/ccount: R_cd = R * div / (shunt * gain).
        let l = l_cd_from_si(DEFAULT_L_HENRIES, 33, 15_000, 18_200, 10_000).unwrap();
        let div: f64 = 10_000.0 / 28_200.0;
        let r_cd = 4.7 * div / (0.033 * 15.0);
        assert!((r_cd - 3.37).abs() < 0.02, "r_cd={r_cd}");
        assert!((l - 0.5e-3 * div / 0.495).abs() < 1e-9);
        assert!(l_cd_from_si(1e-3, 0, 15_000, 1, 1).is_none());
    }

    #[test]
    fn encoders_round_trip_and_saturate() {
        let g = synthesize(&rig_plant(), &BwTargets::default());
        let e = encode(&g);
        for (name, f) in e.fields() {
            assert!(!f.saturated, "{name} saturated: {f:?}");
            assert!(f.quantization_pct < 5.0, "{name} quant {f:?}");
        }
        // B too big for Q3.13 saturates and flags, never wraps
        let big = GainSet { b: 9.0, ..g };
        let e = encode(&big);
        assert_eq!(e.b_i_q313.raw, u16::MAX);
        assert!(e.b_i_q313.saturated);
        // negative clamps to zero flagged
        let neg = GainSet { fric_fc: -3.0, ..g };
        assert!(encode(&neg).fric_fc_counts.saturated);
        assert_eq!(encode(&neg).fric_fc_counts.raw, 0);
    }

    #[test]
    fn synthesis_magnitudes_are_rig_sane() {
        let g = synthesize(&rig_plant(), &BwTargets::default());
        assert!(g.i_kp > 1.0 && g.i_kp < 10.0, "i_kp={}", g.i_kp);
        assert!(g.i_ki > 0.2 && g.i_ki < 5.0, "i_ki={}", g.i_ki);
        assert!((g.i_kaw - 2.0 * g.i_ki).abs() < 1e-12);
        assert!(g.v_kp > 1.0 && g.v_kp < 100.0, "v_kp={}", g.v_kp);
        assert!((g.j_ff - 10.0).abs() < 1e-9);
        assert!((g.p_kp - 157.08).abs() < 0.01);
        assert!(g.l1 > 0.05 && g.l1 < 0.5, "l1={}", g.l1);
        assert!(g.l2 > 1.0 && g.l2 < 50.0, "l2={}", g.l2);
        assert!(g.l3 > 0.1 && g.l3 < 20.0, "l3={}", g.l3);
        // sigma 1.0: the deadband sits on its floor
        assert_eq!(g.pos_deadband, DEADBAND_FLOOR_COUNTS);
        // rig-scale noise: the noise-derived deadband clears the floor
        let noisy = PlantParams {
            sigma_theta: 6.9,
            ..rig_plant()
        };
        let g = synthesize(&noisy, &BwTargets::default());
        assert!(
            (g.pos_deadband - 2.5 * 6.9).abs() < 1e-9,
            "deadband={}",
            g.pos_deadband
        );
    }

    /// Mirror of estimator/fusion.rs (Q3.13 b_i coupling), integer-exact.
    struct FusionMirror {
        theta: i32,
        omega: i32,
        tau: i32,
    }

    struct FusionRaw {
        b_i: u16,
        l1: u16,
        l2: u16,
        l3: u16,
        fric_fc: u16,
    }

    impl FusionMirror {
        fn seed(pos: u16) -> Self {
            Self {
                theta: (pos as i32) << 16,
                omega: 0,
                tau: 0,
            }
        }

        fn step(&mut self, i_counts: i32, pos_meas: u16, dt_med_q32: u32, g: &FusionRaw) {
            const E_LIM: i32 = 1 << 23;
            const THETA_LIM: i32 = 1 << 29;
            const OMEGA_LIM: i32 = 32767 << 16;
            const TAU_LIM: i32 = 4095 << 16;
            const ACCEL_LIM: i32 = 8192;
            let fric = if self.omega > 1 << 16 {
                g.fric_fc as i32
            } else if self.omega < -(1 << 16) {
                -(g.fric_fc as i32)
            } else {
                0
            };
            let accel = i_counts
                .saturating_sub(fric)
                .saturating_sub(self.tau >> 16)
                .clamp(-ACCEL_LIM, ACCEL_LIM);
            self.omega = self
                .omega
                .saturating_add(q_mul(g.b_i as i32, accel, 0).saturating_mul(1 << 3))
                .clamp(-OMEGA_LIM, OMEGA_LIM);
            self.theta = self
                .theta
                .saturating_add(q_mul(self.omega, dt_med_q32 as i32, 32))
                .clamp(-THETA_LIM, THETA_LIM);
            let e = (((pos_meas as i32) << 16) - self.theta).clamp(-E_LIM, E_LIM);
            self.theta = self
                .theta
                .saturating_add(q_mul(g.l1 as i32, e, 16))
                .clamp(-THETA_LIM, THETA_LIM);
            self.omega = self
                .omega
                .saturating_add(q_mul(g.l2 as i32, e, 8))
                .clamp(-OMEGA_LIM, OMEGA_LIM);
            self.tau = self
                .tau
                .saturating_sub(q_mul(g.l3 as i32, e, 8))
                .clamp(-TAU_LIM, TAU_LIM);
        }
    }

    fn fusion_raw(e: &EncodedGains) -> FusionRaw {
        FusionRaw {
            b_i: e.b_i_q313.raw,
            l1: e.l1_q016.raw,
            l2: e.l2_q88.raw,
            l3: e.l3_q88.raw,
            fric_fc: 0,
        }
    }

    fn lcg(state: &mut u64) -> f64 {
        // same LCG family as the fitmath tests: uniform in [-0.5, 0.5)
        *state = state
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        ((*state >> 33) as f64 / (1u64 << 31) as f64) - 0.5
    }

    #[test]
    fn fusion_verification_grid() {
        // The commit's core: the synthesized ENCODED l-set must run the
        // real integer update stably across the space the fits can land
        // in. Constant-velocity tracking, then a disturbance step.
        let f_med = 2_010.0;
        let dt_q32 = ((1u64 << 32) / 2_010) as u32;
        for b in [0.03, 0.1, 0.3] {
            for sigma in [0.5, 2.0] {
                let p = PlantParams {
                    b,
                    sigma_theta: sigma,
                    fc: 0.0,
                    ..rig_plant()
                };
                let g = synthesize(&p, &BwTargets::default());
                let e = encode(&g);
                let raw = fusion_raw(&e);
                let mut rng = 0x1234_5678u64;
                let mut noisy = |truth: f64| -> u16 {
                    (truth + lcg(&mut rng) * 2.0 * sigma)
                        .round()
                        .clamp(0.0, 4095.0) as u16
                };

                // ramp 1 count per medium tick = f_med c/s, i = 0; stays
                // inside the pot span (noisy() clamps at 4095)
                let mut f = FusionMirror::seed(300);
                let mut errs = Vec::new();
                let mut omegas = Vec::new();
                for k in 0..3000i32 {
                    let truth = 300.0 + k as f64;
                    f.step(0, noisy(truth), dt_q32, &raw);
                    if k >= 2500 {
                        errs.push(f.theta as f64 / 65536.0 - truth);
                        omegas.push(f.omega as f64 / 65536.0);
                    }
                }
                let theta_err = errs.iter().sum::<f64>() / errs.len() as f64;
                let omega_mean = omegas.iter().sum::<f64>() / omegas.len() as f64;
                assert!(
                    theta_err.abs() < 3.0 * sigma + 2.0,
                    "B={b} sigma={sigma} theta_err={theta_err}"
                );
                assert!(
                    (omega_mean - f_med).abs() < 0.1 * f_med + 3.0 * g.l2 * sigma,
                    "B={b} sigma={sigma} omega={omega_mean}"
                );

                // disturbance step: pos pinned, i = 500 -> tau -> 500
                let mut f = FusionMirror::seed(2000);
                let mut taus = Vec::new();
                for k in 0..6000 {
                    f.step(500, noisy(2000.0), dt_q32, &raw);
                    if k >= 5000 {
                        taus.push(f.tau as f64 / 65536.0);
                    }
                }
                let tau_mean = taus.iter().sum::<f64>() / taus.len() as f64;
                let tau_spread = taus.iter().cloned().fold(f64::MIN, f64::max)
                    - taus.iter().cloned().fold(f64::MAX, f64::min);
                assert!(
                    (tau_mean - 500.0).abs() < 75.0,
                    "B={b} sigma={sigma} tau={tau_mean}"
                );
                // a bad placement rails or limit-cycles by hundreds; the
                // healthy jitter scales with l3 * sigma
                assert!(
                    tau_spread < 40.0 * g.l3 * sigma + 60.0,
                    "B={b} sigma={sigma} spread={tau_spread} l3={}",
                    g.l3
                );
            }
        }
    }

    /// Mirror of kernel/current.rs step, integer-exact.
    struct CurrentMirror {
        integ: i32,
    }

    impl CurrentMirror {
        // arg-for-arg with the firmware step; fidelity beats arity here
        #[allow(clippy::too_many_arguments)]
        fn step(
            &mut self,
            i_ref: i32,
            i_meas: Option<i32>,
            vbus: u16,
            recip: u32,
            kp: u16,
            ki: u16,
            kaw: u16,
            duty_max: u16,
        ) -> i16 {
            const E_LIM: i32 = 8192;
            const AW_LIM: i32 = 8192;
            let e = match i_meas {
                Some(i) => i_ref.saturating_sub(i).clamp(-E_LIM, E_LIM),
                None => 0,
            };
            let u_pi = q_mul(kp as i32, e, 8).saturating_add(self.integ >> 16);
            let u = u_pi; // ke ff unused here (omega fed zero)
            let v_max = q_mul(duty_max as i32, vbus as i32, 15);
            let u_cl = u.clamp(-v_max, v_max);
            if i_meas.is_some() {
                let aw = u_cl.saturating_sub(u).clamp(-AW_LIM, AW_LIM);
                let ki_term = q_mul(ki as i32, e, 0).saturating_mul(1 << 4);
                let aw_term = q_mul(kaw as i32, aw, 0).saturating_mul(1 << 4);
                let lim = v_max.saturating_mul(1 << 16);
                self.integ = self
                    .integ
                    .saturating_add(ki_term)
                    .saturating_add(aw_term)
                    .clamp(-lim, lim);
            }
            let recip = recip.min(i32::MAX as u32) as i32;
            q_mul(u_cl, recip, 15).clamp(-(i16::MAX as i32), i16::MAX as i32) as i16
        }
    }

    #[test]
    fn current_pi_verification() {
        // Encoded PI against the discrete R-L count-domain plant at rig
        // scale: settles fast and clean, and the kaw path unwinds a
        // saturated integrator promptly.
        let p = rig_plant();
        let g = synthesize(&p, &BwTargets::default());
        let e = encode(&g);
        let vbus: u16 = 1713;
        let recip = ((32767u64 << 15) / vbus as u64) as u32;
        let a = (1.0 / p.tick_hz) / p.l_cd; // Euler step gain, stable (a*R < 2)

        let mut m = CurrentMirror { integ: 0 };
        let mut i = 0.0f64;
        let mut peak = 0.0f64;
        for k in 0..200 {
            let duty = m.step(
                500,
                Some(i.round() as i32),
                vbus,
                recip,
                e.i_kp_q88.raw,
                e.i_ki_q412.raw,
                e.i_kaw_q412.raw,
                32767,
            );
            let v = duty as f64 * vbus as f64 / 32767.0;
            i += a * (v - p.r_vpc * i);
            peak = peak.max(i);
            if k > 100 {
                assert!((i - 500.0).abs() < 15.0, "tick {k}: i={i}");
            }
        }
        assert!(peak < 650.0, "overshoot: peak={peak}");

        // windup: unreachable ref (v_max/R ~ 508), then drop to 200
        let mut m = CurrentMirror { integ: 0 };
        let mut i = 0.0f64;
        for _ in 0..1000 {
            let duty = m.step(
                4000,
                Some(i.round() as i32),
                vbus,
                recip,
                e.i_kp_q88.raw,
                e.i_ki_q412.raw,
                e.i_kaw_q412.raw,
                32767,
            );
            i += a * (duty as f64 * vbus as f64 / 32767.0 - p.r_vpc * i);
        }
        assert!(i > 480.0, "saturated plateau i={i}");
        let mut settled = None;
        for k in 0..100 {
            let duty = m.step(
                200,
                Some(i.round() as i32),
                vbus,
                recip,
                e.i_kp_q88.raw,
                e.i_ki_q412.raw,
                e.i_kaw_q412.raw,
                32767,
            );
            i += a * (duty as f64 * vbus as f64 / 32767.0 - p.r_vpc * i);
            if settled.is_none() && (i - 200.0).abs() < 20.0 {
                settled = Some(k);
            }
        }
        assert!(
            settled.is_some_and(|k| k <= 30),
            "windup recovery too slow: {settled:?}"
        );
    }

    /// Mirror of kernel/velocity.rs step (ff paths fed zero), integer-exact.
    struct VelocityMirror {
        integ: i32,
    }

    impl VelocityMirror {
        fn step(
            &mut self,
            omega_ref: i32,
            omega_hat: i32,
            lim: i32,
            kp: u16,
            ki: u16,
            kaw: u16,
        ) -> i32 {
            const E_LIM: i32 = 16384 << 16;
            const AW_LIM: i32 = 8192;
            let e = omega_ref.saturating_sub(omega_hat).clamp(-E_LIM, E_LIM);
            let i_p = q_mul(kp as i32, e, 24);
            let i_raw = i_p.saturating_add(self.integ >> 16);
            let i_ref = i_raw.clamp(-lim, lim);
            let aw = i_ref.saturating_sub(i_raw).clamp(-AW_LIM, AW_LIM);
            let ki_term = q_mul(ki as i32, e, 16).saturating_mul(1 << 4);
            let aw_term = q_mul(kaw as i32, aw, 0).saturating_mul(1 << 4);
            let l = lim.saturating_mul(1 << 16);
            self.integ = self
                .integ
                .saturating_add(ki_term)
                .saturating_add(aw_term)
                .clamp(-l, l);
            i_ref
        }
    }

    #[test]
    fn velocity_pi_verification() {
        // Encoded PI against the per-medium-tick integrator plant across
        // the B grid: tracks a 500 c/s step, bounded command, no
        // oscillation blow-up, and a folded band does not wind up.
        for b in [0.03, 0.1, 0.3] {
            let p = PlantParams { b, ..rig_plant() };
            let g = synthesize(&p, &BwTargets::default());
            let e = encode(&g);
            let mut m = VelocityMirror { integ: 0 };
            let mut omega = 0.0f64;
            let mut peak = 0.0f64;
            for k in 0..600 {
                let i_ref = m.step(
                    500 << 16,
                    (omega * 65536.0) as i32,
                    2000,
                    e.v_kp_q88.raw,
                    e.v_ki_q412.raw,
                    e.v_kaw_q412.raw,
                );
                assert!(i_ref.abs() <= 2000);
                omega += b * (i_ref as f64 - p.fv * omega);
                peak = peak.max(omega);
                if k > 400 {
                    assert!(
                        (omega - 500.0).abs() < 25.0,
                        "B={b} tick {k}: omega={omega}"
                    );
                }
            }
            assert!(peak < 800.0, "B={b} overshoot peak={peak}");

            // band folded to 50: integrator must not wind past the fold
            let mut m = VelocityMirror { integ: 0 };
            let mut omega = 0.0f64;
            for _ in 0..2000 {
                let i_ref = m.step(
                    500 << 16,
                    (omega * 65536.0) as i32,
                    50,
                    e.v_kp_q88.raw,
                    e.v_ki_q412.raw,
                    e.v_kaw_q412.raw,
                );
                assert!(i_ref.abs() <= 50);
                omega += b * (i_ref as f64 - p.fv * omega);
            }
            // slow crawl still converges; release must not overshoot 2x
            let mut peak = 0.0f64;
            for _ in 0..600 {
                let i_ref = m.step(
                    500 << 16,
                    (omega * 65536.0) as i32,
                    2000,
                    e.v_kp_q88.raw,
                    e.v_ki_q412.raw,
                    e.v_kaw_q412.raw,
                );
                omega += b * (i_ref as f64 - p.fv * omega);
                peak = peak.max(omega);
            }
            assert!(peak < 1000.0, "B={b} post-fold peak={peak}");
        }
    }
}
