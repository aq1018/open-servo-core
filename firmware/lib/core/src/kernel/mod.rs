//! The assembled kernel (spec "on_tick skeleton"): one `on_tick` per PWM
//! period carries all three rates - FAST every tick (sensor publish, window
//! select, OC trip, current PI, motor write), MEDIUM every DECIM_MED ticks
//! (fusion, trajectory, position/velocity, limits, vbus/bemf, detectors,
//! estimates publish), SLOW every DECIM_SLOW medium ticks (thermometer,
//! derate, undervolt/overtemp). Identification aggregates ride their own
//! /16 fast-tick window (`ident`), independent of DECIM_MED. Tick-indexed
//! by design: a missed tick dilates time, nothing compensates and nothing
//! reads a wall clock.

pub mod current;
pub mod faults;
pub mod ident;
pub mod limits;
pub mod position;
pub mod trajectory;
pub mod velocity;

pub use current::{CurrentGains, CurrentLoop};
pub use limits::{IBand, LimitCfg, LimitState};
pub use position::{PosOut, PositionCfg};
pub use trajectory::{TrajCfg, TrajGen};
pub use velocity::{VelocityGains, VelocityLoop};

use crate::estimator::{
    BemfObs, FusionGains, FusionObs, ThermAnchor, ThermGates, VbusEst, VcalLpf, WindingTherm, bemf,
    window,
};
use crate::math::{q_mul, q_mul_u};
use crate::regions::config::DecaySelect;
use crate::regions::control::Mode;
use crate::tel::{TelSample, TelStream};
use crate::traits::{ControlIo, DecayMode, Motor, MotorCmd};
use crate::{RegionStorageRaw, SensorFrame, Shared};
use osc_units::Effort;

/// FAST -> MEDIUM decimation: MED_HZ = tick_hz / DECIM_MED (2 kHz at 20 kHz).
pub const DECIM_MED: u8 = 10;
/// MEDIUM -> SLOW decimation: SLOW_HZ = MED_HZ / DECIM_SLOW (62.5 Hz).
pub const DECIM_SLOW: u8 = 32;

/// Finished timing constants the chip const-evals from `MOTOR_PWM_FREQ_HZ`
/// and the TIM1 ARR, so core never divides at runtime or install - every
/// field is a compile-time quotient on the chip side (`Precomputed`).
#[derive(Copy, Clone)]
pub struct KernelTiming {
    /// The TIM1 auto-reload the motor write programs; drive-window widths
    /// derive from it (`window::drive_ticks`).
    pub pwm_arr: u16,
    /// `(1 << bemf::RECIP_ARR_SHIFT) / pwm_arr`: duty-fraction reciprocal
    /// for the shared `v_mean` computation.
    pub recip_arr_q24: u32,
    /// FAST tick rate = `MOTOR_PWM_FREQ_HZ`, the same constant stamped into
    /// `CalibSense.tick_hz` at install; carried as the derivation anchor for
    /// the fields below.
    pub tick_hz: u16,
    /// `2^32 / MED_HZ` where `MED_HZ = tick_hz / DECIM_MED`: the medium
    /// integration step, `theta += q_mul(omega, dt_med_q32, 32)`.
    pub dt_med_q32: u32,
    /// `(MED_HZ << 16) / 1000`: ms -> medium ticks via `q_mul_u(ms, ., 16)`.
    pub med_ticks_per_ms_q16: u32,
}

/// Runs in the ADC DMA TC ISR (PFIC LOW); one `on_tick` per PWM period.
/// Single-writer contracts: the transport (PFIC HIGH) owns every
/// CONTROL/CONFIG/CALIB write and can preempt this ISR mid-read, so the
/// kernel only ever reads those regions - volatile via `region_ptr`, never
/// forming `&T`, cross-field tearing accepted (each field is independently
/// sane). The kernel is the sole writer of TELEMETRY sensors/estimates/mode
/// and the `fault_flags` byte.
pub struct Kernel<I: ControlIo, T: TelStream = ()> {
    pub io: I,
    tel: T,
    timing: KernelTiming,
    decim_med: u8,
    decim_slow: u8,
    vcal_lpf: VcalLpf,
    traj: TrajGen,
    fusion: FusionObs,
    cur: CurrentLoop,
    vel: VelocityLoop,
    limits: LimitState,
    i_band: IBand,
    vbus: VbusEst,
    thermal: WindingTherm,
    bemf: BemfObs,
    faults: faults::FaultLatch,
    det: faults::Detectors,
    booted: bool,
    te_prev: bool,
    run_prev: bool,
    mode_prev: Mode,
    /// MEDIUM's current command, consumed by the FAST current loop.
    i_ref_cc: i32,
    /// Position loop output, held for the velocity step (MEDIUM-internal).
    omega_ref_q16: i32,
    /// Anti-hunt hold from the position loop; FAST maps it to Coast.
    hold: bool,
    /// The duty actually commanded this tick, post-clamp post-gate: 0 while
    /// Coast/Disabled. Feeds next tick's window select AND the
    /// `duty_applied_q15` publish - `CurrentLoop::last_duty` is not used for
    /// telemetry because the gate can override the loop's output.
    duty_q15: i16,
    decay: DecayMode,
    /// Last window-valid measurement, for the `i_hat_counts` publish.
    i_meas_last: i16,
    /// Identification aggregator, own /16 fast-tick window.
    ident: ident::IdentAgg,
    /// Last v-valid drive-window differential (va - vb), for the ident
    /// accumulation - same hold-last-valid pattern as `i_meas_last`.
    vdiff_last: i16,
    /// ms -> medium-tick conversions, recomputed each SLOW pass; primed
    /// never-trip so no detector fires before the first pass computes them.
    stall_time_ticks: u32,
    pos_error_time_ticks: u32,
    /// Last `r0_q12` seeded into the thermometer; a calib rewrite re-seeds.
    therm_r0: u16,
}

impl<I: ControlIo> Kernel<I> {
    pub fn new(io: I, timing: KernelTiming) -> Self {
        Self::with_tel(io, (), timing)
    }
}

impl<I: ControlIo, T: TelStream> Kernel<I, T> {
    pub fn with_tel(io: I, tel: T, timing: KernelTiming) -> Self {
        Self {
            io,
            tel,
            timing,
            // primed so the FIRST tick runs the full medium+slow chain: the
            // ms->tick conversions and the vbus reciprocal exist before any
            // consumer sees them
            decim_med: DECIM_MED - 1,
            decim_slow: DECIM_SLOW - 1,
            vcal_lpf: VcalLpf::new(),
            traj: TrajGen::new(),
            fusion: FusionObs::new(),
            cur: CurrentLoop::new(),
            vel: VelocityLoop::new(),
            limits: LimitState::new(),
            i_band: IBand { lo: 0, hi: 0 },
            vbus: VbusEst::new(),
            thermal: WindingTherm::new(),
            bemf: BemfObs::new(),
            faults: faults::FaultLatch::new(),
            det: faults::Detectors::new(),
            booted: false,
            te_prev: false,
            run_prev: false,
            mode_prev: Mode::OpenLoop,
            i_ref_cc: 0,
            omega_ref_q16: 0,
            hold: false,
            duty_q15: 0,
            decay: DecayMode::Slow,
            i_meas_last: 0,
            ident: ident::IdentAgg::new(),
            vdiff_last: 0,
            stall_time_ticks: u32::MAX,
            pos_error_time_ticks: u32::MAX,
            therm_r0: 0,
        }
    }

    /// Must complete well inside the kernel period (~50 us at 20 kHz).
    pub fn on_tick(&mut self, frame: SensorFrame, shared: &Shared) {
        let p = shared.table.region_ptr();

        // SAFETY: reads of transport-owned regions - raw-pointer volatile
        // block copies, no `&T` formed, aligned repr(C) blocks inside the
        // static table (single-writer contract in the type doc).
        let (life, loop_cur, lim_cfg, therm_cfg, sense, motor_cal, bias) = unsafe {
            (
                (&raw const (*p).control.lifecycle).read_volatile(),
                (&raw const (*p).config.loop_current).read_volatile(),
                (&raw const (*p).config.limits).read_volatile(),
                (&raw const (*p).config.thermal).read_volatile(),
                (&raw const (*p).calib.sense).read_volatile(),
                (&raw const (*p).calib.motor).read_volatile(),
                (&raw const (*p).telemetry.sensors.current_bias_counts).read_volatile(),
            )
        };

        if !self.booted {
            self.booted = true;
            self.fusion.seed(frame.pos);
        }
        let vcal_lpf = self.vcal_lpf.update(frame.vcal);

        // SAFETY: ISR context is the region's sole writer (the `sample_tick`
        // contract); volatile per field so the stores survive optimization.
        unsafe {
            let s = &raw mut (*p).telemetry.sensors;
            (&raw mut (*s).pos).write_volatile(frame.pos);
            (&raw mut (*s).current).write_volatile(frame.current);
            (&raw mut (*s).vcal).write_volatile(frame.vcal);
            (&raw mut (*s).vcal_lpf).write_volatile(vcal_lpf);
            (&raw mut (*s).vmotor_a).write_volatile(frame.vmotor_a);
            (&raw mut (*s).vmotor_b).write_volatile(frame.vmotor_b);
            (&raw mut (*s).current_trough).write_volatile(frame.current_trough);
        }

        // torque_enable 0->1 is the fault ack: latch, detectors, and the
        // limits pend all clear; a still-present condition re-latches
        // through the normal detectors.
        if life.torque_enable && !self.te_prev {
            self.faults.clear();
            self.det.reset();
            self.limits.ack();
        }
        self.te_prev = life.torque_enable;

        // Window from the PREVIOUS tick's command: this frame's scan sampled
        // the period that command drove, so terminal and sign attribution
        // stay correct across sign flips (bang-bang bench test pins this).
        let ticks = window::drive_ticks(self.duty_q15, self.timing.pwm_arr);
        let fwd = self.duty_q15 >= 0;
        let sel = window::select(
            self.decay,
            ticks,
            sense.i_window_min_ticks,
            sense.v_window_min_ticks,
        );
        let i_meas = window::i_from_frame(&frame, sel, fwd, bias);
        if let Some(i) = i_meas {
            self.i_meas_last = i.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        }
        let oc_over = i_meas.map(|i| i.unsigned_abs() > lim_cfg.oc_trip_counts as u32);
        if self.det.oc_sample(oc_over, lim_cfg.oc_trip_ticks) {
            self.faults
                .raise(faults::BIT_OVER_CURRENT, faults::CODE_OVER_CURRENT);
        }

        // IDENT: per-tick sample aligned to the window the PREVIOUS command
        // drove - duty_q15 still holds that command here; i/vdiff hold
        // last-valid through invalid windows (ident module doc).
        if let Some((_, vdiff)) = window::vdrive_from_frame(&frame, sel, fwd) {
            self.vdiff_last = vdiff.clamp(i16::MIN as i32, i16::MAX as i32) as i16;
        }
        // TEL emits HERE, on the fast path before the medium/slow branches:
        // duty_q15 still holds the command whose window this frame's samples
        // measured (the same previous-tick alignment the ident aggregate
        // uses), and the send lands at a near-constant tick offset. Emitting
        // at on_tick's end loses the DMA drain margin every medium tick -
        // bench: >=9 B frames dropped at exactly the medium cadence.
        if life.tel_enable && life.tel_mask != 0 {
            let s = TelSample {
                pos: frame.pos,
                current: self.i_meas_last,
                current_trough: frame.current_trough,
                duty_q15: self.duty_q15,
                vdiff: self.vdiff_last,
                vbus: self.vbus.vbus_counts(),
                window_valid: i_meas.is_some(),
            };
            self.tel.configure(true, life.tel_mask);
            self.tel.on_tick(&s);
        }

        if let Some(agg) = self
            .ident
            .sample(self.i_meas_last, self.vdiff_last, self.duty_q15)
        {
            // SAFETY: sole-telemetry-writer contract (type doc); volatile
            // per-field stores at the ident window boundary.
            unsafe {
                let d = &raw mut (*p).telemetry.ident;
                (&raw mut (*d).i_mean_counts).write_volatile(agg.i_mean_counts);
                (&raw mut (*d).i_min_counts).write_volatile(agg.i_min_counts);
                (&raw mut (*d).i_max_counts).write_volatile(agg.i_max_counts);
                (&raw mut (*d).vdiff_mean).write_volatile(agg.vdiff_mean);
                (&raw mut (*d).duty_mean_q15).write_volatile(agg.duty_mean_q15);
                (&raw mut (*d).agg_seq).write_volatile(agg.agg_seq);
            }
        }

        let run = life.torque_enable && self.faults.mask() == 0;
        if run != self.run_prev {
            // both edges zero the loop chain; the enable edge additionally
            // reseeds fusion at the measurement (torque-off tau_d is built
            // from i_use = 0 fiction - a hand-moved shaft rails it and every
            // enable would re-latch STALL via the collision check) and the
            // profile at the fresh estimate - bumpless
            if run {
                self.fusion.seed(frame.pos);
                self.traj.reseed(self.fusion.theta_q16());
            }
            self.cur.reset();
            self.vel.reset();
            self.i_ref_cc = 0;
            self.omega_ref_q16 = 0;
            self.hold = false;
            self.run_prev = run;
        }
        if run && life.mode != self.mode_prev {
            // mode change mid-run: reference = estimate, at rest
            self.traj.reseed(self.fusion.theta_q16());
            self.hold = false;
        }
        self.mode_prev = life.mode;

        self.decim_med += 1;
        if self.decim_med >= DECIM_MED {
            self.decim_med = 0;

            // SAFETY: same volatile single-writer read contract as above.
            let (loop_vel, loop_pos, fus_cfg, fault_cfg, pos_lim, winding) = unsafe {
                (
                    (&raw const (*p).config.loop_velocity).read_volatile(),
                    (&raw const (*p).config.loop_position).read_volatile(),
                    (&raw const (*p).config.fusion).read_volatile(),
                    (&raw const (*p).config.fault_cfg).read_volatile(),
                    (&raw const (*p).config.pos_limits).read_volatile(),
                    (&raw const (*p).calib.winding).read_volatile(),
                )
            };

            // i_use: window-valid measurement, else the cached command - the
            // observer never sees the validity flag (fusion contract). While
            // disabled or in OpenLoop the cache is 0, so an invalid window
            // predicts torque-free.
            let i_use = i_meas.unwrap_or(self.i_ref_cc);
            let fg = FusionGains {
                b_i_q313: motor_cal.b_i_q313,
                l1_q016: fus_cfg.l1_q016,
                l2_q88: fus_cfg.l2_q88,
                l3_q88: fus_cfg.l3_q88,
                l_bemf_q016: fus_cfg.l_bemf_q016,
                fric_fc_counts: motor_cal.fric_fc_counts,
            };
            // previous medium tick's bemf estimate: one tick stale is fine
            // for a blend that defaults off
            let omega_bemf_q16 = self.bemf.omega_cps().clamp(-32767, 32767) << 16;
            self.fusion.step(
                i_use,
                frame.pos,
                Some(omega_bemf_q16),
                self.timing.dt_med_q32,
                &fg,
            );
            let theta_hat = self.fusion.theta_q16();
            let omega_hat = self.fusion.omega_q16();

            let tc = TrajCfg {
                vel_limit_cps: loop_pos.velocity_limit_cps,
                accel_limit_q88: loop_pos.accel_limit_q88,
                pos_min_soft_counts: pos_lim.pos_min_soft_counts,
                pos_max_soft_counts: pos_lim.pos_max_soft_counts,
                dt_med_q32: self.timing.dt_med_q32,
            };
            if run {
                match life.mode {
                    Mode::Position => {
                        self.traj.step_position(life.goal_position, &tc);
                        let pc = PositionCfg {
                            kp_q88: loop_pos.p_kp_q88,
                            pos_deadband_counts: loop_pos.pos_deadband_counts,
                            vel_limit_cps: loop_pos.velocity_limit_cps,
                        };
                        let out = position::step(
                            self.traj.theta_star_q16(),
                            self.traj.omega_star_q16(),
                            theta_hat,
                            &pc,
                        );
                        self.omega_ref_q16 = out.omega_ref_q16;
                        self.hold = out.hold;
                    }
                    Mode::Velocity => {
                        self.traj.step_velocity(life.goal_velocity, theta_hat, &tc);
                        self.omega_ref_q16 = self.traj.omega_star_q16();
                        self.hold = false;
                    }
                    Mode::Current | Mode::OpenLoop => self.hold = false,
                }
            }

            // limits fold; pinned = last command sat at a nonzero ceiling
            let prev_lim = self.limits.i_lim_counts();
            let pinned = prev_lim != 0 && self.i_ref_cc.unsigned_abs() >= prev_lim as u32;
            let lcfg = LimitCfg {
                current_limit_counts: lim_cfg.current_limit_counts,
                stall_response: lim_cfg.stall_response,
                drive_polarity: lim_cfg.drive_polarity,
                stall_omega_max_cps: lim_cfg.stall_omega_max_cps,
                stall_time_ticks: self.stall_time_ticks,
                stall_yield_counts: lim_cfg.stall_yield_counts,
                stall_release_counts: lim_cfg.stall_release_counts,
                stall_tau_trip_counts: lim_cfg.stall_tau_trip_counts,
                derate_start_cc: therm_cfg.derate_start_cc,
                cutoff_cc: therm_cfg.cutoff_cc,
                pos_min_soft_counts: pos_lim.pos_min_soft_counts,
                pos_max_soft_counts: pos_lim.pos_max_soft_counts,
            };
            let omega_abs_cps = omega_hat.unsigned_abs() >> 16;
            let band = self.limits.fold(
                pinned,
                omega_abs_cps,
                self.fusion.tau_d_counts().unsigned_abs(),
                theta_hat >> 16,
                &lcfg,
            );
            self.i_band = band;
            if run && self.limits.stall_fault_pending() {
                self.faults.raise(faults::BIT_STALL, faults::CODE_STALL);
            }

            if run {
                match life.mode {
                    // Parked: the drive coasts, so the velocity loop must stop
                    // too. omega_hat is pot-noise driven at rest (+-hundreds
                    // c/s); left running, the PI integrates that phantom error
                    // until i_ref pins at the current limit, and pinned + slow
                    // false-trips the stall detector (bench: CODE_STALL a few
                    // seconds into a clean hold). Zero the command and drain
                    // the integrator so it never winds and resumes bumplessly.
                    Mode::Position if self.hold => {
                        self.i_ref_cc = 0;
                        self.vel.reset();
                    }
                    Mode::Velocity | Mode::Position => {
                        let vg = VelocityGains {
                            kp_q88: loop_vel.v_kp_q88,
                            ki_q412: loop_vel.v_ki_q412,
                            kaw_q412: loop_vel.v_kaw_q412,
                            j_ff_q88: loop_vel.j_ff_q88,
                            fric_fc_counts: motor_cal.fric_fc_counts,
                            fric_fv_q016: motor_cal.fric_fv_q016,
                        };
                        self.i_ref_cc = self.vel.step(
                            self.omega_ref_q16,
                            omega_hat,
                            self.traj.alpha_star_q16(),
                            self.traj.omega_star_q16(),
                            band,
                            &vg,
                        );
                    }
                    // clamped fresh at the FAST rate below
                    Mode::Current => {}
                    Mode::OpenLoop => self.i_ref_cc = 0,
                }
            }

            // vbus plus the shared v_mean: computed ONCE here, consumed by
            // bemf now and the thermometer at SLOW (bemf RECIP_ARR contract)
            let vdrive = window::vdrive_from_frame(&frame, sel, fwd);
            self.vbus
                .step(vdrive.map(|(v, _)| v), therm_cfg.v_undervolt_counts);
            let v_mean = vdrive.map(|(_, vdiff)| {
                q_mul(
                    ticks as i32 * vdiff,
                    self.timing.recip_arr_q24 as i32,
                    bemf::RECIP_ARR_SHIFT,
                )
            });
            let omega_bemf = self
                .bemf
                .step(v_mean, i_meas, motor_cal.r_q12, motor_cal.recip_ke_q);

            // raw-pot sanity screen runs in every mode, torque-off included
            if self.det.sensor_sample(
                frame.pos,
                fault_cfg.sensor_delta_max,
                fault_cfg.sensor_bad_count,
            ) {
                self.faults.raise(faults::BIT_SENSOR, faults::CODE_SENSOR);
            }
            // tracking-error persistence: only meaningful with a live profile
            let pos_err_over = run
                && life.mode == Mode::Position
                && self
                    .traj
                    .theta_star_q16()
                    .saturating_sub(theta_hat)
                    .unsigned_abs()
                    > (fault_cfg.pos_error_counts as u32) << 16;
            if self
                .det
                .pos_err_sample(pos_err_over, self.pos_error_time_ticks)
            {
                self.faults
                    .raise(faults::BIT_POSITION_ERROR, faults::CODE_POSITION_ERROR);
            }

            self.decim_slow += 1;
            if self.decim_slow >= DECIM_SLOW {
                self.decim_slow = 0;
                // ms -> medium ticks each SLOW pass, straight off the live
                // table values: at 62.5 Hz two q16 multiplies are cheaper and
                // simpler than write hooks, and a rewrite lands within one
                // SLOW period (deliberate).
                self.stall_time_ticks = q_mul_u(
                    lim_cfg.stall_time_ms as u32,
                    self.timing.med_ticks_per_ms_q16,
                    16,
                );
                self.pos_error_time_ticks = q_mul_u(
                    fault_cfg.pos_error_time_ms as u32,
                    self.timing.med_ticks_per_ms_q16,
                    16,
                );

                // thermometer seed tracks the calib anchor: install writes
                // and host rewrites both land here
                if winding.r0_q12 != self.therm_r0 {
                    self.thermal.seed(winding.r0_q12);
                    self.therm_r0 = winding.r0_q12;
                }
                let gates = ThermGates {
                    i_min_counts: therm_cfg.rtherm_i_min_counts,
                    omega_max_cps: therm_cfg.rtherm_omega_max_cps,
                };
                let anchor = ThermAnchor {
                    r0_q12: winding.r0_q12,
                    t0_cc: winding.t0_cc,
                    k_r2t_q88: winding.k_r2t_q88,
                    mu_q016: winding.mu_q016,
                };
                // the LMS sample needs BOTH window paths valid
                let (vm, therm_i) = match (v_mean, i_meas) {
                    (Some(v), Some(i)) => (v, Some(i)),
                    _ => (0, None),
                };
                let t_cc = self
                    .thermal
                    .step(vm, therm_i, omega_abs_cps, &gates, &anchor);
                self.limits.update_derate(t_cc, &lcfg);
                if t_cc >= therm_cfg.cutoff_cc {
                    self.faults
                        .raise(faults::BIT_OVER_TEMP, faults::CODE_OVER_TEMP);
                }
                // undervolt needs FRESH evidence: a held estimate frozen by
                // the fault's own bridge-off (no drive -> no window) would
                // re-latch forever off a recovered rail. Recovery is a retry
                // probe: ack -> drive resumes -> fresh sample -> verdict.
                let vb = self.vbus.vbus_counts();
                if self.vbus.take_fresh() && vb < therm_cfg.v_undervolt_counts {
                    self.faults
                        .raise(faults::BIT_UNDER_VOLT, faults::CODE_UNDER_VOLT);
                }
            }

            // SAFETY: sole-telemetry-writer contract (type doc); volatile
            // per-field stores, medium-boundary publish.
            unsafe {
                let e = &raw mut (*p).telemetry.estimates;
                (&raw mut (*e).theta_hat_q16).write_volatile(theta_hat);
                (&raw mut (*e).omega_hat_cps).write_volatile(omega_hat);
                (&raw mut (*e).tau_d_counts).write_volatile(self.fusion.tau_d_counts());
                (&raw mut (*e).i_lim_counts).write_volatile(self.limits.i_lim_counts());
                (&raw mut (*e).t_winding_cc).write_volatile(self.thermal.t_cc());
                (&raw mut (*e).vbus_counts).write_volatile(self.vbus.vbus_counts());
                (&raw mut (*e).duty_applied_q15).write_volatile(self.duty_q15);
                (&raw mut (*e).omega_bemf_cps).write_volatile(omega_bemf);
                (&raw mut (*e).r_hat_q12).write_volatile(self.thermal.r_q12());
                (&raw mut (*e).i_hat_counts).write_volatile(self.i_meas_last);
                let m = &raw mut (*p).telemetry.mode;
                (&raw mut (*m).mode_active).write_volatile(life.mode as u8);
                (&raw mut (*m).fault_code).write_volatile(self.faults.code());
                (&raw mut (*p).telemetry.common.fault_flags).write_volatile(self.faults.mask());
            }
        }

        // Re-gate after the medium chain: a fault raised above disables THIS
        // tick's drive (the loop-reset bookkeeping catches up next tick).
        let run = run && self.faults.mask() == 0;
        let cmd = if !run {
            self.duty_q15 = 0;
            MotorCmd::Disabled
        } else {
            match life.mode {
                Mode::OpenLoop => {
                    // raw passthrough, duty_max-clamped; NO vbus comp -
                    // identification wants unconfounded actuation
                    let max = loop_cur.duty_max_q15.min(i16::MAX as u16) as i32;
                    let duty = (life.goal_duty as i32).clamp(-max, max) as i16;
                    let decay = match lim_cfg.openloop_decay {
                        DecaySelect::Slow => DecayMode::Slow,
                        DecaySelect::Fast => DecayMode::Fast,
                    };
                    self.duty_q15 = duty;
                    self.decay = decay;
                    MotorCmd::Drive {
                        duty: Effort(duty),
                        decay,
                    }
                }
                mode => {
                    if mode == Mode::Current {
                        // directional band: an endstop blocks only inward
                        // goals; retreat clamps against the composed limit
                        self.i_ref_cc = self.i_band.clamp(life.goal_current as i32);
                    }
                    if self.hold {
                        // anti-hunt: park instead of dithering on friction;
                        // the frozen loop resumes bumplessly on exit
                        self.duty_q15 = 0;
                        MotorCmd::Coast
                    } else if self.i_ref_cc == 0 && i_meas.is_none() {
                        // zero ref with no window: the honest-zero feed
                        // below makes e = 0, freezing the PI at whatever
                        // sub-floor duty it unwound to - stalled at an
                        // endstop that grinds the gears forever (bench:
                        // 18% duty held into the rail). Zero duty is the
                        // honest actuation; slow decay shorts the winding,
                        // passively braking whatever momentum remains.
                        self.cur.reset();
                        self.duty_q15 = 0;
                        self.decay = DecayMode::Slow;
                        MotorCmd::Drive {
                            duty: Effort(0),
                            decay: DecayMode::Slow,
                        }
                    } else {
                        let gains = CurrentGains {
                            kp_q88: loop_cur.i_kp_q88,
                            ki_q412: loop_cur.i_ki_q412,
                            kaw_q412: loop_cur.i_kaw_q412,
                            ke_q412: motor_cal.ke_vpc_q,
                            duty_max_q15: loop_cur.duty_max_q15,
                        };
                        // undervolt-floored vbus: the same floor `vbus.step`
                        // applied to the reciprocal, so (vbus, recip) stay
                        // the contract pair even before the first seed
                        let vbus_eff = self.vbus.vbus_counts().max(therm_cfg.v_undervolt_counts);
                        // An invalid window only happens below the sampling
                        // floor, where the duty is too small to push real
                        // current: 0 is the honest estimate THERE, and it
                        // lets the PI lift off zero duty - a strict freeze
                        // would deadlock (no duty -> no window -> e = 0 ->
                        // no duty). OC and the estimators keep the strict
                        // validity view.
                        let i_loop = Some(i_meas.unwrap_or(0));
                        let duty = self.cur.step(
                            self.i_ref_cc,
                            i_loop,
                            self.fusion.omega_q16(),
                            vbus_eff,
                            self.vbus.recip_q15(),
                            &gains,
                        );
                        self.duty_q15 = duty;
                        // closed-loop decay is fixed Slow (spec: config enum
                        // exists for OpenLoop identification only)
                        self.decay = DecayMode::Slow;
                        MotorCmd::Drive {
                            duty: Effort(duty),
                            decay: DecayMode::Slow,
                        }
                    }
                }
            }
        };
        let (_sensors, motor) = self.io.parts();
        motor.write(cmd);
    }
}

#[cfg(test)]
mod tests;
