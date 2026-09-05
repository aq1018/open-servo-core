//! Kernel-level orchestration tests plus the closed-loop plant smoke suite.
//! The per-block tests carry the numeric precision; these pin the wiring:
//! gate, ack, dispatch shapes, publishes, and qualitative closed-loop
//! behavior against a crude integer plant.

use super::*;
use crate::regions::config::StallResponse;
use crate::traits::Sensors;
use crate::{RegionStorage, Shared};

const BIAS: u16 = 2048;
const ARR: u16 = 1200;

// The chip-side const-eval for a 20 kHz FAST rate (MED 2 kHz).
const TIMING: KernelTiming = KernelTiming {
    pwm_arr: ARR,
    recip_arr_q24: (1 << bemf::RECIP_ARR_SHIFT) / ARR as u32,
    tick_hz: 20_000,
    dt_med_q32: ((1u64 << 32) / 2000) as u32,
    med_ticks_per_ms_q16: 2 << 16,
};

struct FakeSensors;
impl Sensors for FakeSensors {
    fn frame(&mut self) -> SensorFrame {
        SensorFrame::default()
    }
}

struct FakeMotor {
    last: Option<MotorCmd>,
}
impl Motor for FakeMotor {
    fn write(&mut self, cmd: MotorCmd) {
        self.last = Some(cmd);
    }
}

struct FakeIo {
    sensors: FakeSensors,
    motor: FakeMotor,
}
impl ControlIo for FakeIo {
    type Sensors = FakeSensors;
    type Motor = FakeMotor;
    fn parts(&mut self) -> (&mut FakeSensors, &mut FakeMotor) {
        (&mut self.sensors, &mut self.motor)
    }
}

fn kernel() -> Kernel<FakeIo> {
    Kernel::new(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        TIMING,
    )
}

fn last_cmd(k: &Kernel<FakeIo>) -> MotorCmd {
    k.io.motor.last.expect("a motor write happened")
}

/// Hand-stable rig baseline; tests override per case via `with_mut`.
fn seed(shared: &Shared) {
    shared.table.with_mut(|t| {
        let c = &mut t.config;
        c.pos_limits.pos_min_soft_counts = 0;
        c.pos_limits.pos_max_soft_counts = 4095;
        c.loop_current.i_kp_q88 = 256;
        c.loop_current.i_ki_q412 = 200;
        c.loop_current.i_kaw_q412 = 2048;
        c.loop_current.duty_max_q15 = 32767;
        c.loop_velocity.v_kp_q88 = 64;
        c.loop_velocity.v_ki_q412 = 40;
        c.loop_velocity.v_kaw_q412 = 2048;
        c.loop_velocity.j_ff_q88 = 0;
        c.loop_position.p_kp_q88 = 256;
        c.loop_position.pos_deadband_counts = 8;
        c.loop_position.velocity_limit_cps = 3000;
        c.loop_position.accel_limit_q88 = 50 << 8;
        c.limits.current_limit_counts = 1200;
        c.limits.stall_response = StallResponse::Yield;
        c.limits.drive_polarity = true;
        c.limits.stall_omega_max_cps = 500;
        c.limits.stall_time_ms = 50;
        c.limits.stall_yield_counts = 300;
        c.limits.stall_release_counts = 150;
        c.limits.stall_tau_trip_counts = 1200;
        c.limits.oc_trip_counts = 2400;
        c.limits.oc_trip_ticks = 4;
        c.limits.openloop_decay = DecaySelect::Slow;
        c.thermal.derate_start_cc = 8000;
        c.thermal.cutoff_cc = 10000;
        c.thermal.recover_cc = 9000;
        c.thermal.v_undervolt_counts = 2200;
        c.thermal.rtherm_i_min_counts = 300;
        c.thermal.rtherm_omega_max_cps = 400;
        c.fusion.l1_q016 = 16384;
        c.fusion.l2_q88 = 1024;
        // l3 * B is the tau_d loop gain: at the b_i-encoded B (1.0 below)
        // 0.5 cc per count is stable, 2.0 rails the filter
        c.fusion.l3_q88 = 128;
        c.fusion.l_bemf_q016 = 0;
        c.fault_cfg.pos_error_counts = 400;
        c.fault_cfg.pos_error_time_ms = 500;
        c.fault_cfg.sensor_delta_max = 256;
        c.fault_cfg.sensor_bad_count = 4;
        let cal = &mut t.calib;
        cal.sense.i_window_min_ticks = 100;
        cal.sense.v_window_min_ticks = 100;
        cal.motor.ke_vpc_q = 256; // 0.0625 vcounts per c/s
        cal.motor.r_q12 = 8192; // 2.0 vcounts/ccount
        cal.motor.recip_ke_q = 16 << 10; // 16 c/s per vcount
        // plant B ~= 1.33 c/s per ccount per medium tick (200/1500 x 10);
        // encoded 1.0 (the old Q0.16-ceiling dynamics, kept so the pinned
        // integer sims stand); correction gains absorb the gap
        cal.motor.b_i_q313 = 8192;
        t.telemetry.sensors.current_bias_counts = BIAS;
        t.control.lifecycle.torque_enable = false;
        t.control.lifecycle.mode = Mode::Position;
    });
}

fn frame(pos: u16, current: u16) -> SensorFrame {
    SensorFrame {
        pos,
        current,
        current_trough: current,
        vmotor_a: 3000,
        vmotor_a_trough: 3000,
        vmotor_b: 40,
        vmotor_b_trough: 40,
        vcal: 1200,
        tick: 0,
    }
}

// --- Gate / ack / dispatch ------------------------------------------------

#[test]
fn torque_off_disables_and_estimators_still_track() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..40 {
        k.on_tick(frame(2500, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // boot seeded the fusion at the first measurement
    assert_eq!(k.fusion.theta_q16(), 2500 << 16);
    // the pot moves while disabled: the observer follows it anyway (the
    // live b_i bleed at the plant's saturated B settles the 1500-count
    // step in ~920 medium ticks, integer-sim verified)
    for _ in 0..12000 {
        k.on_tick(frame(1000, BIAS), &sh);
    }
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    let err = (k.fusion.theta_q16() - (1000 << 16)).abs();
    assert!(err < 5 << 16, "theta_hat={}", k.fusion.theta_q16());
    // published while disabled
    sh.table.with(|t| {
        assert_eq!(t.telemetry.estimates.theta_hat_q16, k.fusion.theta_q16());
    });
}

#[test]
fn enable_edge_reseeds_without_transient() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..2000 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
    });
    // at-goal enable: every command in the first stretch is at rest or a
    // bounded nudge - no integrator kick, no profile teleport
    for _ in 0..50 {
        k.on_tick(frame(2000, BIAS), &sh);
        match last_cmd(&k) {
            MotorCmd::Coast | MotorCmd::Disabled => {}
            MotorCmd::Drive { duty, .. } => {
                assert!(duty.0.unsigned_abs() < 2000, "duty={}", duty.0)
            }
            MotorCmd::Brake => panic!("brake is never commanded"),
        }
    }
    assert_eq!(k.traj.theta_star_q16(), 2000 << 16);
}

#[test]
fn enable_edge_clears_hand_era_tau_d() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    // torque off, shaft hand-swept then snapped back: with the b_i bleed
    // live, steady-rate motion is model-explained and leaves tau_d near
    // zero - only the snap transient (e pinned at the clamp) pumps it
    for i in 0..4000u32 {
        let pos = 1000 + ((i / 4) % 2000) as u16;
        k.on_tick(frame(pos, BIAS), &sh);
    }
    for _ in 0..150 {
        k.on_tick(frame(1000, BIAS), &sh);
    }
    assert!(
        k.fusion.tau_d_counts().unsigned_abs() > 200,
        "precondition: hand motion railed tau_d, got {}",
        k.fusion.tau_d_counts()
    );
    // enable at rest: fusion reseeds, so the collision check never sees the
    // stale disturbance and STALL must not latch
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 0;
    });
    for _ in 0..200 {
        k.on_tick(frame(1500, BIAS), &sh);
    }
    assert_eq!(k.faults.mask(), 0, "stale tau_d re-latched STALL");
    assert!(k.fusion.tau_d_counts().unsigned_abs() < 50);
}

#[test]
fn endstop_allows_retreat_from_the_wall() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 120;
    });
    let mut k = kernel();
    // parked hard against the top wall (bench: horn ratcheted to the rail)
    for _ in 0..400 {
        k.on_tick(frame(4095, BIAS), &sh);
    }
    // inward goal is banded to zero: no drive into the wall
    match last_cmd(&k) {
        MotorCmd::Drive { duty, .. } => assert_eq!(duty.0, 0),
        MotorCmd::Coast | MotorCmd::Disabled => {}
        MotorCmd::Brake => panic!("brake is never commanded"),
    }
    // the door back out must be open: a retreat goal drives immediately
    // (the magnitude-fold deadlock read the zeroed i_ref as inward forever).
    // Reverse drive samples the OTHER terminal, so the frame carries the
    // rail on vmotor_b - a forward-shaped frame would read the low leg as
    // a collapsed rail and latch UNDER_VOLT.
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = -120);
    let mut rev = frame(4095, BIAS);
    (rev.vmotor_a, rev.vmotor_a_trough) = (40, 40);
    (rev.vmotor_b, rev.vmotor_b_trough) = (3000, 3000);
    let mut drove = false;
    for _ in 0..400 {
        k.on_tick(rev, &sh);
        if let MotorCmd::Drive { duty, .. } = last_cmd(&k) {
            drove |= duty.0 < 0;
        }
    }
    assert!(drove, "retreat from the wall never drove");
    assert_eq!(k.faults.mask(), 0);
}

#[test]
fn undervolt_needs_fresh_evidence_no_stale_relatch() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8192;
    });
    let mut k = kernel();
    // sagged rail: valid drive windows carry va under the undervolt floor
    let mut sagged = frame(2000, BIAS);
    sagged.vmotor_a = 1000;
    sagged.vmotor_a_trough = 1000;
    for _ in 0..400 {
        k.on_tick(sagged, &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_UNDER_VOLT, "sag latches");
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // bridge off -> no window -> vbus estimate frozen at the sag; the ack
    // must stick because there is no fresh evidence
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..400 {
        k.on_tick(sagged, &sh);
    }
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    // recovered rail from here on
    for _ in 0..2000 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert_eq!(k.faults.mask(), 0, "stale sag re-latched undervolt");
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // a genuinely low rail re-latches through the same retry probe
    for _ in 0..2000 {
        k.on_tick(sagged, &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_UNDER_VOLT);
}

#[test]
fn oc_latch_forces_disabled_despite_torque_enable() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 16000;
    });
    let mut k = kernel();
    // tick 1 drives (windows still invalid: previous duty was 0)
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // 4 consecutive valid over-trip samples latch on the 4th
    for _ in 0..3 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
        assert_eq!(k.faults.mask(), 0);
    }
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    // torque_enable still set: stays disabled, publish lands at medium
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
        assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    }
    sh.table.with(|t| {
        assert_eq!(t.telemetry.common.fault_flags, faults::BIT_OVER_CURRENT);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_OVER_CURRENT);
        assert_eq!(t.telemetry.mode.mode_active, Mode::OpenLoop as u8);
    });
}

#[test]
fn ack_clears_then_relatches_while_condition_persists() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 16000;
    });
    let mut k = kernel();
    for _ in 0..8 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    // ack: drop then raise torque_enable
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = true);
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), 0);
    assert!(matches!(last_cmd(&k), MotorCmd::Drive { .. }));
    // the overcurrent persists: a fresh window re-latches
    for _ in 0..5 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

#[test]
fn oc_gap_rearms_the_window() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 16000;
    });
    let mut k = kernel();
    k.on_tick(frame(2000, BIAS + 3000), &sh); // warmup: invalid window
    for _ in 0..3 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    // one clean sample re-arms; three more over-samples do not latch
    k.on_tick(frame(2000, BIAS + 100), &sh);
    for _ in 0..3 {
        k.on_tick(frame(2000, BIAS + 3000), &sh);
    }
    assert_eq!(k.faults.mask(), 0);
    // the 4th consecutive does
    k.on_tick(frame(2000, BIAS + 3000), &sh);
    assert_eq!(k.faults.mask(), faults::BIT_OVER_CURRENT);
}

#[test]
fn openloop_duty_passthrough_clamped_with_decay() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
        t.config.limits.openloop_decay = DecaySelect::Fast;
    });
    let mut k = kernel();
    k.on_tick(frame(2000, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { duty, decay } => {
            assert_eq!(duty.0, 8000);
            assert!(matches!(decay, DecayMode::Fast));
        }
        other => panic!("expected Drive, got {other:?}"),
    }
    // duty_max clamps the passthrough
    sh.table.with_mut(|t| {
        t.config.loop_current.duty_max_q15 = 5000;
        t.control.lifecycle.goal_duty = 8000;
    });
    k.on_tick(frame(2000, BIAS), &sh);
    match last_cmd(&k) {
        MotorCmd::Drive { duty, .. } => assert_eq!(duty.0, 5000),
        other => panic!("expected Drive, got {other:?}"),
    }
}

#[test]
fn current_mode_clamps_goal_to_i_lim() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 5000;
    });
    let mut k = kernel();
    k.on_tick(frame(2000, BIAS), &sh);
    assert_eq!(k.i_ref_cc, 1200);
    assert!(matches!(
        last_cmd(&k),
        MotorCmd::Drive {
            decay: DecayMode::Slow,
            ..
        }
    ));
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_current = -5000);
    k.on_tick(frame(2000, BIAS), &sh);
    assert_eq!(k.i_ref_cc, -1200);
}

#[test]
fn position_error_latches_after_persistence() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = 100;
        t.config.fault_cfg.pos_error_time_ms = 10; // 20 medium ticks
        // keep the stall path out of this test
        t.config.limits.stall_time_ms = 60000;
    });
    let mut k = kernel();
    // pot frozen at 500 while the profile marches away
    for _ in 0..4000 {
        k.on_tick(frame(500, BIAS), &sh);
        if k.faults.mask() != 0 {
            break;
        }
    }
    assert_eq!(k.faults.mask(), faults::BIT_POSITION_ERROR);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
}

#[test]
fn publishes_land_in_the_table() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = kernel();
    for _ in 0..20 {
        k.on_tick(frame(1234, 2100), &sh);
    }
    sh.table.with(|t| {
        assert_eq!(t.telemetry.sensors.pos, 1234);
        assert_eq!(t.telemetry.sensors.current, 2100);
        assert_eq!(t.telemetry.sensors.vmotor_a, 3000);
        assert_eq!(t.telemetry.estimates.theta_hat_q16, k.fusion.theta_q16());
        assert_eq!(t.telemetry.estimates.omega_hat_cps, k.fusion.omega_q16());
        assert_eq!(t.telemetry.estimates.i_lim_counts, 1200);
        assert_eq!(t.telemetry.estimates.duty_applied_q15, 0);
        assert_eq!(t.telemetry.estimates.vbus_counts, 0); // never driven
        assert_eq!(t.telemetry.mode.mode_active, Mode::Position as u8);
        assert_eq!(t.telemetry.mode.fault_code, faults::CODE_NONE);
        assert_eq!(t.telemetry.common.fault_flags, 0);
    });
}

// --- Ident aggregates -----------------------------------------------------

fn ident_setup(sh: &Shared) {
    seed(sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        // drive_ticks(8000) = 293 >= the 100-tick floors: windows valid
        // from tick 2 on (tick 1 measures the boot duty of 0)
        t.control.lifecycle.goal_duty = 8000;
    });
}

#[test]
fn ident_window_pins_aggregates() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    // window 1 (ticks 1-16) is boot-mixed; spend it
    for _ in 0..16 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    sh.table.with(|t| assert_eq!(t.telemetry.ident.agg_seq, 1));
    // window 2: 8 ticks at +100, 4 at +300, 4 at -60, all windows valid
    for _ in 0..8 {
        k.on_tick(frame(2000, BIAS + 100), &sh);
    }
    // seq holds mid-window: publish only at the /16 boundary
    sh.table.with(|t| assert_eq!(t.telemetry.ident.agg_seq, 1));
    for _ in 0..4 {
        k.on_tick(frame(2000, BIAS + 300), &sh);
    }
    for _ in 0..4 {
        k.on_tick(frame(2000, BIAS - 60), &sh);
    }
    sh.table.with(|t| {
        let d = &t.telemetry.ident;
        // (8*100 + 4*300 + 4*-60) >> 4 = 1760/16 = 110
        assert_eq!(d.i_mean_counts, 110);
        assert_eq!(d.i_min_counts, -60);
        assert_eq!(d.i_max_counts, 300);
        assert_eq!(d.vdiff_mean, 2960); // va 3000 - vb 40, every tick
        assert_eq!(d.duty_mean_q15, 8000);
        assert_eq!(d.agg_seq, 2);
    });
}

#[test]
fn ident_invalid_ticks_hold_last_valid() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    for _ in 0..32 {
        k.on_tick(frame(2000, BIAS + 200), &sh);
    }
    // disable: windows go invalid one tick later (tick 33 still measures
    // the period tick 32's command drove)
    sh.table
        .with_mut(|t| t.control.lifecycle.torque_enable = false);
    for _ in 0..16 {
        k.on_tick(frame(2000, BIAS + 200), &sh);
    }
    sh.table.with(|t| {
        let d = &t.telemetry.ident;
        // 15 invalid ticks held the last valid i/vdiff: means stay put
        assert_eq!(d.i_mean_counts, 200);
        assert_eq!(d.i_min_counts, 200);
        assert_eq!(d.i_max_counts, 200);
        assert_eq!(d.vdiff_mean, 2960);
        // duty is per-tick truth: 8000 on tick 33 only -> 8000>>4 = 500
        assert_eq!(d.duty_mean_q15, 500);
        assert_eq!(d.agg_seq, 3);
    });
}

#[test]
fn ident_accumulators_reset_between_windows() {
    let sh = Shared::new();
    ident_setup(&sh);
    let mut k = kernel();
    // windows 1 (boot-mixed) and 2 at a big current
    for _ in 0..32 {
        k.on_tick(frame(2000, BIAS + 1000), &sh);
    }
    sh.table
        .with(|t| assert_eq!(t.telemetry.ident.i_max_counts, 1000));
    // window 3: 15 ticks at 0, one at -5; window 2's 1000 must not leak
    for _ in 0..15 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    k.on_tick(frame(2000, BIAS - 5), &sh);
    sh.table.with(|t| {
        let d = &t.telemetry.ident;
        assert_eq!(d.i_max_counts, 0);
        assert_eq!(d.i_min_counts, -5);
        // sum -5 >> 4: arithmetic shift floors to -1
        assert_eq!(d.i_mean_counts, -1);
        assert_eq!(d.agg_seq, 3);
    });
}

// --- Closed-loop plant ----------------------------------------------------

/// Crude integer plant in the identification-model shape:
/// `omega[k+1] = a*omega[k] + b*duty - coulomb`, theta integrates, pot =
/// theta >> 16 clamped 0..4095, `i = (duty*vbus>>15 - ke*omega)/R`. Units
/// loose on purpose - qualitative closed-loop behavior, not fidelity.
struct Plant {
    theta_q16: i64,
    omega_cps: i32,
    vbus: u16,
    frozen: bool,
}

impl Plant {
    fn new(pos: u16) -> Self {
        Self {
            theta_q16: (pos as i64) << 16,
            omega_cps: 0,
            vbus: 3000,
            frozen: false,
        }
    }

    /// One FAST tick: a = 1 - 1/64, b*duty = duty*200>>15, coulomb 8 c/s
    /// toward zero; frozen = hard wall (theta pinned, omega 0).
    fn step(&mut self, duty: i16) -> SensorFrame {
        if self.frozen {
            self.omega_cps = 0;
        } else {
            self.omega_cps += ((duty as i32 * 200) >> 15) - (self.omega_cps >> 6);
            if self.omega_cps > 0 {
                self.omega_cps = (self.omega_cps - 8).max(0);
            } else {
                self.omega_cps = (self.omega_cps + 8).min(0);
            }
            self.theta_q16 += ((self.omega_cps as i64) << 16) / 20_000;
        }
        let pos = (self.theta_q16 >> 16).clamp(0, 4095) as u16;
        // ke = 1/16 vcounts per c/s, R = 2 vcounts/ccount
        let v = (duty as i32 * self.vbus as i32) >> 15;
        let i = (v - (self.omega_cps >> 4)) >> 1;
        let mag = if duty >= 0 { i } else { -i };
        let sample = (BIAS as i32 + mag).clamp(0, 4095) as u16;
        let (va, vb) = if duty >= 0 {
            (self.vbus, 40)
        } else {
            (40, self.vbus)
        };
        SensorFrame {
            pos,
            current: sample,
            current_trough: sample,
            vmotor_a: va,
            vmotor_a_trough: va,
            vmotor_b: vb,
            vmotor_b_trough: vb,
            vcal: 1200,
            tick: 0,
        }
    }

    fn pos(&self) -> i32 {
        (self.theta_q16 >> 16) as i32
    }
}

fn run_plant(k: &mut Kernel<FakeIo>, sh: &Shared, plant: &mut Plant, ticks: u32) {
    for _ in 0..ticks {
        let f = plant.step(k.duty_q15);
        k.on_tick(f, sh);
    }
}

#[test]
fn position_step_settles_without_limit_cycle() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3000;
        // the plant/gain pair is qualitative; keep the tracking screen out
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        // this plant accelerates far beyond what the (deliberately
        // degenerate) b_i predict explains, so a large l3 rails tau_d into
        // the collision trip on every ramp; keep it gentle here
        t.config.fusion.l3_q88 = 256;
        // tighter position loop so the post-profile residual closes inside
        // the run instead of creeping on a 1 s time constant
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    run_plant(&mut k, &sh, &mut plant, 40_000); // 2 s
    assert_eq!(k.faults.mask(), 0);
    assert!(
        (plant.pos() - 3000).abs() <= 16,
        "settled at {}",
        plant.pos()
    );
    assert_eq!(k.traj.theta_star_q16(), 3000 << 16, "profile landed");
    // trailing window: position parked, motor overwhelmingly coasting
    let (mut lo, mut hi, mut coast) = (i32::MAX, i32::MIN, 0u32);
    for _ in 0..2000 {
        let f = plant.step(k.duty_q15);
        k.on_tick(f, &sh);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
        if matches!(last_cmd(&k), MotorCmd::Coast) {
            coast += 1;
        }
    }
    assert!(hi - lo <= 2, "limit cycle: spread {}", hi - lo);
    assert!(coast >= 1500, "coast ticks {coast}");
}

#[test]
fn velocity_mode_tracks_a_ramp() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        // plenty of travel so the pot never rails
        t.config.pos_limits.pos_max_soft_counts = 1_000_000;
    });
    let mut k = kernel();
    let mut plant = Plant::new(100);
    for seg in 1..=4i32 {
        sh.table
            .with_mut(|t| t.control.lifecycle.goal_velocity = 500 * seg);
        run_plant(&mut k, &sh, &mut plant, 4000); // 200 ms per segment
    }
    assert_eq!(k.faults.mask(), 0);
    let omega_hat = k.fusion.omega_q16() >> 16;
    assert!((omega_hat - 2000).abs() <= 300, "omega_hat={omega_hat}");
    assert!(
        (plant.omega_cps - 2000).abs() <= 300,
        "plant omega={}",
        plant.omega_cps
    );
}

#[test]
fn hard_wall_stall_yields() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(500);
    plant.frozen = true;
    // The live observer first books the wall current as tau_d, so the
    // wind-up is integrator-paced (~230 ms to the full limit); the stall
    // verdict then folds i_lim to yield, the weaker drive lets tau_d
    // bleed below release, and the fold opens into another probe. That
    // probe-the-wall cycle (bench: re-trips through grip-strength dips)
    // makes any single-phase pin flaky - assert the cycle's invariants
    // across a run instead.
    let mut saw_full = false;
    let mut saw_fold = false;
    for _ in 0..24_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.i_ref_cc == 1200 {
            saw_full = true;
        }
        let mut lim = 0u16;
        sh.table.with(|t| lim = t.telemetry.estimates.i_lim_counts);
        if lim == 300 {
            saw_fold = true;
        }
    }
    assert!(saw_full, "wind-up reached i_lim");
    assert!(saw_fold, "stall verdict folded to yield");
    assert_eq!(k.faults.mask(), 0, "Yield never latches");
}

#[test]
fn hard_wall_stall_faults() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3500;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_response = StallResponse::Fault;
    });
    let mut k = kernel();
    let mut plant = Plant::new(500);
    plant.frozen = true;
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert_eq!(k.faults.mask(), faults::BIT_STALL);
    assert!(matches!(last_cmd(&k), MotorCmd::Disabled));
    sh.table
        .with(|t| assert_eq!(t.telemetry.mode.fault_code, faults::CODE_STALL));
}

#[test]
fn hold_parks_releases_and_reparks() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1990);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(matches!(last_cmd(&k), MotorCmd::Coast), "never parked");
    assert_eq!(k.faults.mask(), 0);
    // a goal write releases the park (omega_star != 0 drops hold); the whole
    // commanded move must run without a single Coast, then it re-parks once
    // the profile lands inside the deadband
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_position = 2100);
    let mut saw_move = false;
    let mut reparked = false;
    for _ in 0..20_000u32 {
        run_plant(&mut k, &sh, &mut plant, 1);
        let moving = k.traj.omega_star_q16() != 0;
        if moving {
            saw_move = true;
            assert!(!matches!(last_cmd(&k), MotorCmd::Coast), "coast mid-move");
        }
        if saw_move && !moving && matches!(last_cmd(&k), MotorCmd::Coast) {
            reparked = true;
        }
    }
    assert!(saw_move, "goal write never started the profile");
    assert!(reparked, "never re-parked after the move");
    assert_eq!(k.faults.mask(), 0);
    // an external push out of the deadband wakes the park, then it re-parks
    plant.theta_q16 -= 60i64 << 16;
    let mut woke = false;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        woke |= !matches!(last_cmd(&k), MotorCmd::Coast);
    }
    assert!(woke, "push never woke the hold");
    let mut coast = 0u32;
    for _ in 0..30_000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if matches!(last_cmd(&k), MotorCmd::Coast) {
            coast += 1;
        }
    }
    assert!(
        coast > 15_000,
        "never re-parked after the push: coast {coast}"
    );
    assert_eq!(k.faults.mask(), 0);
    assert!((plant.pos() - 2100).abs() <= 16, "rest {}", plant.pos());
}

#[test]
fn hold_stays_parked_under_pot_noise() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 2000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1990);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(matches!(last_cmd(&k), MotorCmd::Coast), "never parked");
    // parked hold under +-3 counts of pot noise: gating on position rest
    // (not the noisy omega_hat) keeps the park engaged - a wake per blip is
    // the shake fix2 removes
    let mut rng: u32 = 0xdead_beef;
    let (mut coast, mut lo, mut hi) = (0u32, i32::MAX, i32::MIN);
    for _ in 0..40_000 {
        let mut f = plant.step(k.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        let n = ((rng >> 24) as i32 % 7) - 3;
        f.pos = (f.pos as i32 + n).clamp(0, 4095) as u16;
        k.on_tick(f, &sh);
        if matches!(last_cmd(&k), MotorCmd::Coast) {
            coast += 1;
        }
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(hi - lo <= 4, "hold limit cycle: spread {}", hi - lo);
    assert!(coast >= 36_000, "coast ticks {coast} of 40000");
}

#[test]
fn current_mode_endstop_unwinds_duty_to_zero() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Current;
        t.control.lifecycle.goal_current = 300;
        t.config.pos_limits.pos_max_soft_counts = 3000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    // free flight into the soft wall with a wound-up loop
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert_eq!(k.faults.mask(), 0);
    assert!(
        plant.pos() >= 3000,
        "never reached the wall: {}",
        plant.pos()
    );
    // past the wall the endstop zeroes i_ref; the loop must unwind ALL the
    // way to zero, not freeze at the sub-floor duty the honest-zero feed
    // leaves behind (bench: 18% duty held grinding into the rail)
    assert_eq!(k.duty_q15, 0, "duty froze above zero at the wall");
    let mut zero = 0u32;
    for _ in 0..2000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        if k.duty_q15 == 0 {
            zero += 1;
        }
    }
    assert!(zero >= 1990, "duty kept firing at the wall: {zero} of 2000");
}

#[test]
fn velocity_mode_brakes_at_the_soft_wall() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::Velocity;
        t.control.lifecycle.goal_velocity = 2000;
        t.config.pos_limits.pos_max_soft_counts = 3000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.limits.stall_tau_trip_counts = u16::MAX;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    run_plant(&mut k, &sh, &mut plant, 30_000);
    assert_eq!(k.faults.mask(), 0);
    // the wall ramp shrinks the inward goal with distance: the profile
    // decelerates INTO the wall instead of leaving momentum to coast on an
    // open-loop current cut, and parks without a flip-flop limit cycle
    assert!(
        plant.pos() >= 2950 && plant.pos() <= 3020,
        "rest pos {}",
        plant.pos()
    );
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    let mut star_max = 0i32;
    for _ in 0..4000 {
        run_plant(&mut k, &sh, &mut plant, 1);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
        star_max = star_max.max(k.traj.omega_star_q16());
    }
    assert!(hi - lo <= 4, "limit cycle at the wall: spread {}", hi - lo);
    assert!(
        star_max <= 64 << 16,
        "profile still commands inward speed: {}",
        star_max >> 16
    );
    // the door back out stays open
    sh.table
        .with_mut(|t| t.control.lifecycle.goal_velocity = -500);
    run_plant(&mut k, &sh, &mut plant, 20_000);
    assert!(
        plant.pos() < 2900,
        "retreat from the wall failed: {}",
        plant.pos()
    );
}

#[test]
fn position_step_survives_tick_deletion() {
    let sh = Shared::new();
    seed(&sh);
    sh.table.with_mut(|t| {
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.goal_position = 3000;
        t.config.fault_cfg.pos_error_counts = u16::MAX;
        t.config.fusion.l3_q88 = 256;
        t.config.loop_position.p_kp_q88 = 1024;
    });
    let mut k = kernel();
    let mut plant = Plant::new(1000);
    // ~10% of ISR invocations vanish; the plant keeps running on the stale
    // duty (tick-indexed contract: dilation, never compensation)
    let mut rng: u32 = 0x1357_9bdf;
    for _ in 0..40_000 {
        let f = plant.step(k.duty_q15);
        rng = rng.wrapping_mul(1_664_525).wrapping_add(1_013_904_223);
        if !rng.is_multiple_of(10) {
            k.on_tick(f, &sh);
        }
    }
    assert_eq!(k.faults.mask(), 0);
    assert!(
        (plant.pos() - 3000).abs() <= 24,
        "settled at {}",
        plant.pos()
    );
    // still parks: trailing spread stays flat
    let (mut lo, mut hi) = (i32::MAX, i32::MIN);
    for _ in 0..2000 {
        let f = plant.step(k.duty_q15);
        k.on_tick(f, &sh);
        lo = lo.min(plant.pos());
        hi = hi.max(plant.pos());
    }
    assert!(hi - lo <= 4, "limit cycle: spread {}", hi - lo);
}

// --- TEL stream ------------------------------------------------------------

#[derive(Default)]
struct RecTel {
    cfg: Option<(bool, u16)>,
    samples: heapless::Vec<crate::tel::TelSample, 64>,
}
impl crate::tel::TelStream for RecTel {
    fn configure(&mut self, enabled: bool, mask: u16) {
        self.cfg = Some((enabled, mask));
    }
    fn on_tick(&mut self, sample: &crate::tel::TelSample) {
        let _ = self.samples.push(*sample);
    }
}

#[test]
fn tel_stream_gated_by_enable_and_mask() {
    let sh = Shared::new();
    seed(&sh);
    let mut k = Kernel::with_tel(
        FakeIo {
            sensors: FakeSensors,
            motor: FakeMotor { last: None },
        },
        RecTel::default(),
        TIMING,
    );
    // enable/mask both zero, then each alone: no emission
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| t.control.lifecycle.tel_enable = true);
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    sh.table.with_mut(|t| {
        t.control.lifecycle.tel_enable = false;
        t.control.lifecycle.tel_mask = crate::tel::MASK_ALL;
    });
    for _ in 0..10 {
        k.on_tick(frame(2000, BIAS), &sh);
    }
    assert!(k.tel.samples.is_empty());
    assert!(k.tel.cfg.is_none());

    // both set: one sample per tick, table mask forwarded
    sh.table.with_mut(|t| {
        t.control.lifecycle.tel_enable = true;
        t.control.lifecycle.torque_enable = true;
        t.control.lifecycle.mode = Mode::OpenLoop;
        t.control.lifecycle.goal_duty = 8000;
    });
    for _ in 0..20 {
        k.on_tick(frame(2100, BIAS + 40), &sh);
    }
    assert_eq!(k.tel.samples.len(), 20);
    assert_eq!(k.tel.cfg, Some((true, crate::tel::MASK_ALL)));
    let s = k.tel.samples.last().unwrap();
    assert_eq!(s.pos, 2100);
    assert_eq!(s.current_trough, BIAS + 40);
    // previous-tick alignment: the sampled duty is the command whose window
    // this frame measured
    assert_eq!(s.duty_q15, 8000);
    // duty 8000/32767 of ARR is far above the 100-tick window floors
    assert!(s.window_valid);
    assert_eq!(s.current, 40);
}
