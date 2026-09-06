//! Steady-state duty ladder: constant-duty full-travel sweeps, both
//! directions, for the Ke fit and the kinetic friction line. Each rung
//! seeks the start end, rests, then sweeps across the travel collecting
//! ident windows paired with pot position; the steady segment (settle
//! windows dropped, ends trimmed, slip zone masked) gives omega as the
//! pos-slope LS, plus the segment's mean current and winding volts. Fits
//! live in [`crate::fits`]; rungs whose segment is too short or that stall
//! mid-sweep are dropped with a warning, not silently.
//!
//! Rungs alternate +duty then -duty at each level so the pot ends near the
//! next sweep's start and seeks stay short.

use super::{Cmd, Experiment, RigParams, WindowSample, WindowStream};
use crate::fitmath::{linear_ls, mean};
use crate::fits::{FrictionFit, KeFit, RungPoint, friction_line, ke_fit};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct LadderCfg {
    /// Rung duties, q15, run as +d then -d each (26/33/40/47/55/64%).
    pub rungs_q15: Vec<i16>,
    /// Seek drive toward the start end, q15.
    pub seek_duty_q15: i16,
    /// Sweep start/stop distance from the guard edges, counts. OpenLoop
    /// cannot brake: a duty-0 at speed coasts several hundred counts, and
    /// this absorbs it (bench: the 64% sweep-end coast on a healthy gear
    /// ran ~970 counts through the rest pause, breaching a 900 margin).
    pub stop_margin: u16,
    pub poll_ms: u32,
    pub seek_poll_ms: u32,
    /// Rest after every seek and every sweep.
    pub rest_ms: u32,
    /// Still detect: pos delta <= eps across this many consecutive polls.
    pub stall_eps: u16,
    pub stall_polls: u32,
    /// Seek gives up (with a warning) after this many polls.
    pub seek_cap_polls: u32,
    /// Fraction trimmed off each end of a sweep's accepted windows.
    pub trim_frac: f64,
    /// Minimum steady windows for a rung to enter the fits.
    pub min_steady: usize,
}

impl Default for LadderCfg {
    fn default() -> Self {
        Self {
            // 26/33/40/47/55/64% of 32767
            rungs_q15: vec![8520, 10813, 13107, 15400, 18022, 20971],
            seek_duty_q15: 8520,
            stop_margin: 1300,
            poll_ms: 2,
            seek_poll_ms: 30,
            rest_ms: 300,
            stall_eps: 3,
            stall_polls: 10,
            seek_cap_polls: 400,
            trim_frac: 0.15,
            min_steady: 12,
        }
    }
}

/// One accepted sweep window with the pot position it was read with.
#[derive(Copy, Clone, Debug)]
struct SweepSample {
    w: WindowSample,
    pos: u16,
}

/// One rung reduced; `used` = false rungs carry their reason in `note`.
#[derive(Clone, Debug)]
pub struct RungSummary {
    pub duty_q15: i16,
    pub omega: f64,
    pub omega_r2: f64,
    pub i: f64,
    pub v: f64,
    pub windows: usize,
    pub used: bool,
    pub note: Option<String>,
}

#[derive(Clone, Debug)]
pub struct LadderResult {
    pub ke: KeFit,
    pub fric_fwd: Option<FrictionFit>,
    pub fric_rev: Option<FrictionFit>,
    pub rungs: Vec<RungSummary>,
    pub warnings: Vec<String>,
}

enum Phase {
    ModeWrite,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SeekRest,
    RungSet,
    RungRead,
    RungEval,
    RungOff,
    RungRest,
    FinishTorque,
    Finished,
}

pub struct Ladder {
    cfg: LadderCfg,
    params: RigParams,
    band: (u16, u16),
    phase: Phase,
    /// Sweep index: rung level = sweep / 2, direction = +1 then -1.
    sweep: usize,
    last_pos: Option<u16>,
    still: u32,
    polls: u32,
    windows: WindowStream,
    sweep_samples: Vec<SweepSample>,
    stalled_note: bool,
    rungs: Vec<RungSummary>,
    warnings: Vec<String>,
}

impl Ladder {
    pub fn new(cfg: LadderCfg, params: &RigParams) -> Self {
        // sweeps need a band to run between even when the envelope guard
        // is off; fall back to the default guard
        let band = params.pos_guard.unwrap_or((150, 3950));
        Self {
            cfg,
            params: *params,
            band,
            phase: Phase::ModeWrite,
            sweep: 0,
            last_pos: None,
            still: 0,
            polls: 0,
            windows: WindowStream::new(params),
            sweep_samples: Vec::new(),
            stalled_note: false,
            rungs: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn dir(&self) -> i8 {
        if self.sweep.is_multiple_of(2) { 1 } else { -1 }
    }

    fn duty(&self) -> i16 {
        let d = self.cfg.rungs_q15[self.sweep / 2];
        if self.dir() > 0 { d } else { -d }
    }

    /// Sweep start is near the band edge the drive moves away from.
    fn start_stop(&self) -> (u16, u16) {
        let lo = self.band.0 + self.cfg.stop_margin;
        let hi = self.band.1 - self.cfg.stop_margin;
        if self.dir() > 0 { (lo, hi) } else { (hi, lo) }
    }

    fn seek_done(&self, pos: u16) -> bool {
        let (start, _) = self.start_stop();
        if self.dir() > 0 {
            pos <= start
        } else {
            pos >= start
        }
    }

    fn sweep_done(&self, pos: u16) -> bool {
        let (_, stop) = self.start_stop();
        if self.dir() > 0 {
            pos >= stop
        } else {
            pos <= stop
        }
    }

    fn reset_motion_track(&mut self) {
        self.last_pos = None;
        self.still = 0;
        self.polls = 0;
    }

    fn track_still(&mut self, pos: u16) {
        if let Some(last) = self.last_pos
            && pos.abs_diff(last) <= self.cfg.stall_eps
        {
            self.still += 1;
        } else {
            self.still = 0;
        }
        self.last_pos = Some(pos);
    }

    /// Reduce the finished sweep into a rung summary.
    fn close_rung(&mut self) {
        let duty = self.duty();
        let n_raw = self.sweep_samples.len();
        let trim = (n_raw as f64 * self.cfg.trim_frac) as usize;
        let steady: Vec<&SweepSample> = self.sweep_samples[trim..n_raw.saturating_sub(trim)]
            .iter()
            .filter(|s| !self.params.in_slip(s.pos))
            .collect();
        let mut note = None;
        if self.stalled_note {
            note = Some("stalled mid-sweep".into());
        } else if steady.len() < self.cfg.min_steady {
            note = Some(format!("steady segment too short ({})", steady.len()));
        }
        let used = note.is_none();
        let mut rung = RungSummary {
            duty_q15: duty,
            omega: 0.0,
            omega_r2: 0.0,
            i: 0.0,
            v: 0.0,
            windows: steady.len(),
            used,
            note,
        };
        if used {
            let pos_t: Vec<(f64, f64)> = steady
                .iter()
                .map(|s| (s.w.t_ms / 1000.0, s.pos as f64))
                .collect();
            let iv: Vec<f64> = steady.iter().map(|s| s.w.i).collect();
            // duty * vdiff / 32767 is |v|; re-sign by the drive direction
            let vv: Vec<f64> = steady
                .iter()
                .map(|s| s.w.duty_q15 * s.w.vdiff / 32767.0 * self.dir() as f64)
                .collect();
            match (linear_ls(&pos_t), mean(&iv), mean(&vv)) {
                (Some(slope), Some(i), Some(v)) => {
                    rung.omega = slope.b;
                    rung.omega_r2 = slope.r2;
                    rung.i = i;
                    rung.v = v;
                }
                _ => {
                    rung.used = false;
                    rung.note = Some("degenerate steady segment".into());
                }
            }
        }
        if let Some(n) = &rung.note {
            self.warnings.push(format!("rung {duty}: {n}"));
        }
        self.rungs.push(rung);
        self.sweep_samples.clear();
        self.stalled_note = false;
    }

    /// Ke + friction line over the used rungs; R comes from the resistance
    /// run. None until at least two usable rungs exist.
    pub fn fit(&self, r_vpc: f64) -> Option<LadderResult> {
        let pts: Vec<RungPoint> = self
            .rungs
            .iter()
            .filter(|r| r.used)
            .map(|r| RungPoint {
                omega: r.omega,
                i: r.i,
                v: r.v,
            })
            .collect();
        let ke = ke_fit(&pts, r_vpc)?;
        Some(LadderResult {
            ke,
            fric_fwd: friction_line(&pts, 1),
            fric_rev: friction_line(&pts, -1),
            rungs: self.rungs.clone(),
            warnings: self.warnings.clone(),
        })
    }
}

impl Experiment for Ladder {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::TorqueOn => {
                self.phase = Phase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::SeekSet => {
                self.reset_motion_track();
                self.windows.mark_transition();
                self.phase = Phase::SeekRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: -(self.dir() as i32) * self.cfg.seek_duty_q15 as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                let mut done = false;
                if let Some(o) = obs {
                    self.track_still(o.pos);
                    // still = the physical end sits inside the band
                    done = self.seek_done(o.pos) || self.still >= self.cfg.stall_polls;
                }
                self.polls += 1;
                if !done && self.polls >= self.cfg.seek_cap_polls {
                    self.warnings
                        .push(format!("sweep {}: seek gave up, starting here", self.sweep));
                    done = true;
                }
                if done {
                    self.phase = Phase::SeekOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            Phase::SeekOff => {
                self.windows.mark_transition();
                self.phase = Phase::SeekRest;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::SeekRest => {
                self.phase = Phase::RungSet;
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::RungSet => {
                self.reset_motion_track();
                self.windows.mark_transition();
                self.phase = Phase::RungRead;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.duty() as i32,
                }
            }
            Phase::RungRead => {
                self.phase = Phase::RungEval;
                Cmd::Read
            }
            Phase::RungEval => {
                let mut done = false;
                if let Some(o) = obs {
                    if let Some(w) = self.windows.push(o) {
                        self.sweep_samples.push(SweepSample { w, pos: o.pos });
                    }
                    self.track_still(o.pos);
                    if self.still >= self.cfg.stall_polls {
                        self.stalled_note = true;
                        done = true;
                    }
                    done = done || self.sweep_done(o.pos);
                }
                if done {
                    self.phase = Phase::RungOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = Phase::RungRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            Phase::RungOff => {
                self.close_rung();
                self.windows.mark_transition();
                self.phase = Phase::RungRest;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::RungRest => {
                self.sweep += 1;
                self.phase = if self.sweep < 2 * self.cfg.rungs_q15.len() {
                    Phase::SeekSet
                } else {
                    Phase::FinishTorque
                };
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::FinishTorque => {
                self.phase = Phase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            Phase::Finished => Cmd::Done,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::super::testkit::{FakeServo, pump};
    use super::super::{Guarded, RigParams};
    use super::*;

    fn physical_servo() -> FakeServo {
        let mut s = FakeServo::new(3.37);
        s.physical_motion = true;
        s.fv = 0.006;
        s
    }

    fn run_cfg(servo: &mut FakeServo, cfg: LadderCfg, params: RigParams) -> (Ladder, Vec<String>) {
        let mut exp = Guarded::new(Ladder::new(cfg, &params), params);
        let log = pump(&mut exp, servo, 2_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    fn run_e3(servo: &mut FakeServo, params: RigParams) -> (Ladder, Vec<String>) {
        run_cfg(servo, LadderCfg::default(), params)
    }

    #[test]
    fn recovers_planted_ke_and_friction_line() {
        let mut servo = physical_servo();
        let (exp, log) = run_e3(&mut servo, RigParams::default());
        assert!(!log.contains(&"OVERRUN".to_string()));
        let fit = exp.fit(3.37).expect("usable rungs");
        assert_eq!(fit.rungs.iter().filter(|r| r.used).count(), 12);
        assert!(
            (fit.ke.ke_vpc - 0.1731).abs() / 0.1731 < 0.01,
            "ke {}",
            fit.ke.ke_vpc
        );
        assert!(fit.ke.r2 > 0.999, "r2 {}", fit.ke.r2);
        for fr in [fit.fric_fwd.unwrap(), fit.fric_rev.unwrap()] {
            assert!((fr.fc - 20.0).abs() < 2.0, "fc {}", fr.fc);
            assert!((fr.fv - 0.006).abs() / 0.006 < 0.1, "fv {}", fr.fv);
        }
    }

    #[test]
    fn slip_zone_samples_are_masked() {
        let mut clean = physical_servo();
        let (exp_clean, _) = run_e3(&mut clean, RigParams::default());
        let mut glitched = physical_servo();
        // +80-count pot artifact strictly inside the masked slip zone:
        // above the sweep-start threshold (band lo + stop_margin) so the
        // seek geometry matches the clean run, and low enough that the
        // +80 readings also stay inside the mask
        glitched.glitch_zone = Some((1460.0, 1560.0));
        let (exp_glitch, _) = run_e3(&mut glitched, RigParams::default());
        let a = exp_clean.fit(3.37).unwrap();
        let b = exp_glitch.fit(3.37).unwrap();
        assert!(
            (a.ke.ke_vpc - b.ke.ke_vpc).abs() < 1e-9,
            "masked glitch must not move the fit: {} vs {}",
            a.ke.ke_vpc,
            b.ke.ke_vpc
        );
    }

    #[test]
    fn short_travel_rung_drops_with_warning() {
        let mut servo = physical_servo();
        // 400-count sweep span: the fast rungs cross it in a few dozen
        // windows, under the raised min_steady; the slow rungs clear it
        servo.pos = 2050.0;
        servo.ends = (1600.0, 2500.0);
        let params = RigParams {
            pos_guard: Some((1600, 2500)),
            slip: (0, 0),
            ..RigParams::default()
        };
        let cfg = LadderCfg {
            min_steady: 40,
            // frictionless fake never coasts; keep the scaled-down band's
            // sweep span alive
            stop_margin: 250,
            ..LadderCfg::default()
        };
        let (exp, _) = run_cfg(&mut servo, cfg, params);
        let fit = exp.fit(3.37).expect("slow rungs still usable");
        assert!(!fit.warnings.is_empty(), "expected short-segment warnings");
        assert!(
            fit.rungs.iter().any(|r| !r.used),
            "fastest rungs should drop"
        );
        assert!(fit.rungs.iter().filter(|r| r.used).count() >= 2);
    }
}
