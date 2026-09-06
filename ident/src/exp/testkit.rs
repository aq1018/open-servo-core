//! Test-only fake servo + driver pump: a scripted plant that answers the
//! engine's commands the way the rig would, so experiments run end to end
//! in-process. Electrical model: stalled i = v/R; free-running i = fc; the
//! first windows after a duty change are inflated to imitate the L
//! transient the settle discard exists for.

use super::{Cmd, Experiment};
use crate::frame::{TelFrame, TelemetrySnapshot};
use crate::regs::{ALL, Reg, control};

pub struct FakeServo {
    pub r: f64,
    pub vbus: f64,
    pub ke: f64,
    pub fc: f64,
    /// Viscous friction, ccounts per c/s (physical model only).
    pub fv: f64,
    pub free_speed: f64,
    /// Steady omega from the motor equation instead of the free_speed
    /// shortcut: omega = (|v| - R*fc) / (Ke + R*fv), signed by duty.
    pub physical_motion: bool,
    /// First-order dynamics for the inertia transient: omega integrates
    /// alpha = b * f_med * (i - fc*sgn - fv*omega) with i = (v - Ke*w)/R,
    /// so the planted `b` is exactly what the estimators must recover.
    /// Steady state matches `physical_motion` by construction.
    pub dynamic: bool,
    /// B, (c/s per medium tick) per ccount (dynamic model).
    pub b: f64,
    /// Medium rate, tick_hz / 10.
    pub f_med: f64,
    /// Static friction: no motion below this |duty| (0 = none).
    pub breakaway_q15: i16,
    /// Reported pos gains +80 counts inside this zone (slip artifact).
    pub glitch_zone: Option<(f64, f64)>,
    pub pos: f64,
    pub ends: (f64, f64),
    /// Wiring convention: false inverts duty's effect on motion, so the
    /// endstop experiment must infer the flipped drive_polarity.
    pub drive_polarity: bool,
    pub pos_noise: f64,
    pub fault_at_ms: Option<f64>,
    pub torque: bool,
    pub duty: i16,
    pub tel_enable: bool,
    pub tel_mask: u16,
    tel_seq: u8,
    omega_dyn: f64,
    pub t_ms: f64,
    t_duty_change: f64,
    pub transient_windows: f64,
    pub transient_gain: f64,
    lcg: u64,
}

impl FakeServo {
    pub fn new(r: f64) -> Self {
        Self {
            r,
            vbus: 1731.0,
            ke: 0.1731,
            fc: 20.0,
            fv: 0.0,
            free_speed: 10_000.0,
            physical_motion: false,
            dynamic: false,
            b: 0.1,
            f_med: 2010.0,
            breakaway_q15: 0,
            glitch_zone: None,
            pos: 2400.0,
            ends: (200.0, 4000.0),
            drive_polarity: true,
            pos_noise: 0.0,
            fault_at_ms: None,
            torque: false,
            duty: 0,
            tel_enable: false,
            tel_mask: 0,
            tel_seq: 0,
            omega_dyn: 0.0,
            t_ms: 0.0,
            t_duty_change: -1e9,
            transient_windows: 3.0,
            transient_gain: 1.5,
            lcg: 0x9E3779B97F4A7C15,
        }
    }

    fn noise(&mut self) -> f64 {
        self.lcg = self
            .lcg
            .wrapping_mul(6364136223846793005)
            .wrapping_add(1442695040888963407);
        // uniform in [-0.5, 0.5) scaled by pos_noise
        ((self.lcg >> 11) as f64 / (1u64 << 53) as f64 - 0.5) * self.pos_noise
    }

    fn omega(&self) -> f64 {
        if self.dynamic {
            return self.omega_dyn;
        }
        if !self.torque || self.duty == 0 || self.duty.unsigned_abs() < self.breakaway_q15 as u16 {
            return 0.0;
        }
        let vsign = self.duty.signum() as f64 * if self.drive_polarity { 1.0 } else { -1.0 };
        let stalled =
            (self.pos <= self.ends.0 && vsign < 0.0) || (self.pos >= self.ends.1 && vsign > 0.0);
        if stalled {
            return 0.0;
        }
        if self.physical_motion {
            let v = self.duty.unsigned_abs() as f64 / 32767.0 * self.vbus;
            let mag = ((v - self.r * self.fc) / (self.ke + self.r * self.fv)).max(0.0);
            mag * vsign
        } else {
            self.duty.unsigned_abs() as f64 / 32767.0 * self.free_speed * vsign
        }
    }

    /// The winding current the dynamic model carries right now: ohmic on
    /// the applied volts minus bemf. Friction is mechanical - it consumes
    /// torque, not extra current - so nothing else is added.
    fn i_dyn(&self) -> f64 {
        if !self.torque || self.duty == 0 {
            return 0.0;
        }
        let v = self.duty as f64 / 32767.0 * self.vbus;
        (v - self.ke * self.omega_dyn) / self.r
    }

    pub fn write(&mut self, reg: Reg, value: i32) {
        if reg == control::TORQUE_ENABLE {
            self.torque = value != 0;
        } else if reg == control::GOAL_DUTY {
            self.duty = value as i16;
            self.t_duty_change = self.t_ms;
        } else if reg == control::TEL_ENABLE {
            self.tel_enable = value != 0;
        } else if reg == control::TEL_MASK {
            self.tel_mask = value as u16;
        }
    }

    /// One dynamic-model integration substep.
    fn substep(&mut self, dt: f64) {
        let i = self.i_dyn();
        let w = self.omega_dyn;
        let fric = if w != 0.0 {
            self.fc * w.signum() + self.fv * w
        } else if i.abs() > self.fc {
            self.fc * i.signum()
        } else {
            i // no net torque below stiction: alpha = 0
        };
        let alpha = self.b * self.f_med * (i - fric);
        let w2 = w + alpha * dt;
        // coasting friction never reverses the spin through zero
        self.omega_dyn = if !self.torque || self.duty == 0 {
            if w != 0.0 && w.signum() != w2.signum() {
                0.0
            } else {
                w2
            }
        } else {
            w2
        };
        self.pos = (self.pos + self.omega_dyn * dt).clamp(self.ends.0, self.ends.1);
        if (self.pos <= self.ends.0 && self.omega_dyn < 0.0)
            || (self.pos >= self.ends.1 && self.omega_dyn > 0.0)
        {
            self.omega_dyn = 0.0;
        }
    }

    pub fn advance(&mut self, ms: u32) {
        if self.dynamic {
            // tick-sized substeps keep the ~tens-of-ms tau integration exact
            let dt = 1.0 / (self.f_med * 10.0);
            let n = (ms as f64 / 1000.0 / dt).round() as u64;
            for _ in 0..n {
                self.substep(dt);
            }
        } else {
            let dt = ms as f64 / 1000.0;
            self.pos = (self.pos + self.omega() * dt).clamp(self.ends.0, self.ends.1);
        }
        self.t_ms += ms as f64;
    }

    /// Like [`advance`], emitting one TEL frame per fast tick while the
    /// table has the stream armed (ladder mask assumed: pos, current,
    /// duty, vdiff). No L-transient inflation here - the electrical
    /// transient is about one tick long on the real rig.
    pub fn advance_tel(&mut self, ms: u32, sink: &mut Vec<TelFrame>) {
        let dt = 1.0 / (self.f_med * 10.0);
        let n = (ms as f64 / 1000.0 / dt).round() as u64;
        for _ in 0..n {
            if self.dynamic {
                self.substep(dt);
            } else {
                self.pos = (self.pos + self.omega() * dt).clamp(self.ends.0, self.ends.1);
            }
            if self.tel_enable && self.tel_mask != 0 {
                let noise = self.noise();
                let driving = self.torque && self.duty != 0;
                sink.push(TelFrame {
                    seq: self.tel_seq,
                    window_valid: driving,
                    pos: Some((self.pos + noise).round().clamp(0.0, 4095.0) as u16),
                    current: Some(self.i_dyn().round() as i16),
                    current_trough: None,
                    duty_q15: Some(if driving { self.duty } else { 0 }),
                    vdiff: Some(if driving {
                        (self.vbus * self.duty.signum() as f64) as i16
                    } else {
                        0
                    }),
                    vbus: None,
                });
                self.tel_seq = self.tel_seq.wrapping_add(1);
            }
        }
        self.t_ms += ms as f64;
    }

    pub fn read(&mut self) -> TelemetrySnapshot {
        let driving = self.torque && self.duty != 0;
        let (i, vdiff) = if driving {
            let v = self.duty as f64 / 32767.0 * self.vbus;
            let omega = self.omega();
            // friction current only while moving: stalled current is ohmic.
            // The physical and dynamic models need no extra term - their
            // (v - ke*omega)/r IS the winding current at every instant.
            let fric = if omega != 0.0 && !self.physical_motion && !self.dynamic {
                self.fc * self.duty.signum() as f64
            } else {
                0.0
            };
            let mut i = (v - self.ke * omega) / self.r + fric;
            if (self.t_ms - self.t_duty_change) / 0.8 < self.transient_windows {
                i *= self.transient_gain;
            }
            (i, self.vbus * self.duty.signum() as f64)
        } else {
            (0.0, 0.0)
        };
        let fault = matches!(self.fault_at_ms, Some(at) if self.t_ms >= at);
        let glitch = match self.glitch_zone {
            Some((lo, hi)) if (lo..=hi).contains(&self.pos) => 80.0,
            _ => 0.0,
        };
        let pos = (self.pos + glitch + self.noise())
            .round()
            .clamp(0.0, 4095.0) as u16;
        TelemetrySnapshot {
            fault_flags: if fault { 32 } else { 0 },
            fault_code: if fault { 6 } else { 0 },
            pos,
            current: (512.0 + i).round() as u16,
            current_bias_counts: 512,
            vbus_counts: self.vbus as u16,
            i_mean_counts: i.round() as i16,
            vdiff_mean: vdiff.round() as i16,
            duty_mean_q15: if driving { self.duty } else { 0 },
            agg_seq: (self.t_ms / 0.8) as u64 as u16,
            ..Default::default()
        }
    }
}

fn reg_name(reg: Reg) -> &'static str {
    ALL.iter()
        .find(|(_, r)| *r == reg)
        .map(|(n, _)| *n)
        .unwrap_or("?")
}

/// Drive an experiment against the fake servo; returns the command log
/// ("write <field> <value>" entries plus a trailing marker on overrun).
pub fn pump<E: Experiment>(exp: &mut E, servo: &mut FakeServo, max_steps: u32) -> Vec<String> {
    let mut log = Vec::new();
    let mut pending: Option<TelemetrySnapshot> = None;
    for _ in 0..max_steps {
        match exp.step(pending.take().as_ref()) {
            Cmd::Write { reg, value } => {
                servo.write(reg, value);
                log.push(format!("write {} {}", reg_name(reg), value));
            }
            Cmd::Read => pending = Some(servo.read()),
            Cmd::Pause { ms } => servo.advance(ms),
            Cmd::Done => return log,
        }
    }
    log.push("OVERRUN".into());
    log
}

/// [`pump`] with the TEL side channel live: every Pause synthesizes the
/// tick-rate frames the wire would carry and hands them to the inertia
/// experiment, the way the CLI pumps deframer output between commands.
pub fn pump_tel(
    exp: &mut super::inertia::Inertia,
    servo: &mut FakeServo,
    max_steps: u32,
) -> Vec<String> {
    let mut log = Vec::new();
    let mut pending: Option<TelemetrySnapshot> = None;
    let mut frames = Vec::new();
    for _ in 0..max_steps {
        match exp.step(pending.take().as_ref()) {
            Cmd::Write { reg, value } => {
                servo.write(reg, value);
                log.push(format!("write {} {}", reg_name(reg), value));
            }
            Cmd::Read => pending = Some(servo.read()),
            Cmd::Pause { ms } => {
                frames.clear();
                servo.advance_tel(ms, &mut frames);
                exp.push_tel(&frames);
            }
            Cmd::Done => return log,
        }
    }
    log.push("OVERRUN".into());
    log
}
