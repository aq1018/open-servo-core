//! E5/E6 closed-loop verification, run AFTER the fitted gains are written:
//! Current-mode steps against an end-stop (the loop must settle onto the
//! goal) and Velocity-mode travel legs (the pot slope must track the
//! goal). Both report per-step/leg errors and an overall pass verdict -
//! the check that the synthesized bandwidths hold on the real plant.

use super::{Cmd, Experiment, RigParams, WindowSample, WindowStream};
use crate::fitmath::linear_ls;
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

/// E5: goal_current steps into a stall. Amplitudes must clear the current
/// window floor (i valid needs |duty| >= ~20%, so goal >= ~floor/R * vbus
/// terms - on the rig >= ~110 counts) and stay under the table's
/// current_limit clamp.
pub struct VerifyCurrentCfg {
    /// Seek drive toward each end, q15, OpenLoop.
    pub seek_duty_q15: i16,
    /// Step amplitudes, ccounts, applied with the stall direction's sign.
    pub steps_counts: Vec<i16>,
    pub dwell_polls: u32,
    pub poll_ms: u32,
    pub rest_ms: u32,
    pub stall_eps: u16,
    pub stall_polls: u32,
    pub seek_poll_ms: u32,
    /// Settle band around the goal, fraction.
    pub tol: f64,
}

impl Default for VerifyCurrentCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 8520,
            steps_counts: vec![110, 140],
            dwell_polls: 250,
            poll_ms: 2,
            rest_ms: 300,
            stall_eps: 3,
            stall_polls: 8,
            seek_poll_ms: 30,
            tol: 0.10,
        }
    }
}

#[derive(Clone, Debug)]
pub struct CurrentStep {
    /// Signed goal, ccounts.
    pub goal: i16,
    /// Mean window current over the dwell tail, ccounts.
    pub mean_i: f64,
    pub err_pct: f64,
    /// First in-band window relative to the step, ms; None = never settled.
    pub settle_ms: Option<f64>,
    pub windows: usize,
    pub pass: bool,
}

#[derive(Clone, Debug)]
pub struct VerifyCurrentResult {
    pub steps: Vec<CurrentStep>,
    pub pass: bool,
    pub warnings: Vec<String>,
}

enum CPhase {
    ModeOpen,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SwitchTorqueOff,
    SwitchMode,
    SwitchTorqueOn,
    StepSet,
    StepRead,
    StepEval,
    StepOff,
    StepRest,
    DirTorqueOff,
    DirModeOpen,
    FinishGoal,
    FinishTorque,
    Finished,
}

/// One step's raw windows; settle time comes from the unfiltered stream,
/// the mean from the tail past the settle discard.
struct StepCapture {
    goal: i16,
    t0: Option<f64>,
    windows: Vec<WindowSample>,
}

pub struct VerifyCurrent {
    cfg: VerifyCurrentCfg,
    settle_windows: u32,
    phase: CPhase,
    dir_idx: u8,
    step_idx: usize,
    polls_left: u32,
    last_pos: Option<u16>,
    still: u32,
    /// settle_windows = 0: the capture keeps every window so settle time is
    /// measurable; the mean discards the head manually.
    windows: WindowStream,
    cur: Option<StepCapture>,
    captures: Vec<StepCapture>,
    warnings: Vec<String>,
}

impl VerifyCurrent {
    pub fn new(cfg: VerifyCurrentCfg, params: &RigParams) -> Self {
        let mut raw = *params;
        raw.settle_windows = 0;
        Self {
            cfg,
            settle_windows: params.settle_windows,
            phase: CPhase::ModeOpen,
            dir_idx: 0,
            step_idx: 0,
            polls_left: 0,
            last_pos: None,
            still: 0,
            windows: WindowStream::new(&raw),
            cur: None,
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    fn dir(&self) -> i8 {
        if self.dir_idx == 0 { 1 } else { -1 }
    }

    pub fn result(&self) -> VerifyCurrentResult {
        let mut steps = Vec::new();
        for c in &self.captures {
            let goal = c.goal as f64;
            let tail: Vec<&WindowSample> = c
                .windows
                .iter()
                .skip(self.settle_windows as usize)
                .collect();
            if tail.is_empty() {
                steps.push(CurrentStep {
                    goal: c.goal,
                    mean_i: 0.0,
                    err_pct: 100.0,
                    settle_ms: None,
                    windows: 0,
                    pass: false,
                });
                continue;
            }
            let mean_i = tail.iter().map(|w| w.i).sum::<f64>() / tail.len() as f64;
            let err_pct = (mean_i - goal).abs() / goal.abs() * 100.0;
            let settle_ms = c.t0.and_then(|t0| {
                c.windows
                    .iter()
                    .find(|w| (w.i - goal).abs() <= self.cfg.tol * goal.abs())
                    .map(|w| w.t_ms - t0)
            });
            steps.push(CurrentStep {
                goal: c.goal,
                mean_i,
                err_pct,
                settle_ms,
                windows: tail.len(),
                pass: err_pct <= self.cfg.tol * 100.0,
            });
        }
        let pass = !steps.is_empty() && steps.iter().all(|s| s.pass);
        VerifyCurrentResult {
            steps,
            pass,
            warnings: self.warnings.clone(),
        }
    }
}

impl Experiment for VerifyCurrent {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            CPhase::ModeOpen => {
                self.phase = CPhase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            CPhase::TorqueOn => {
                self.phase = CPhase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            CPhase::SeekSet => {
                self.phase = CPhase::SeekRead;
                self.last_pos = None;
                self.still = 0;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.dir() as i32 * self.cfg.seek_duty_q15 as i32,
                }
            }
            CPhase::SeekRead => {
                self.phase = CPhase::SeekEval;
                Cmd::Read
            }
            CPhase::SeekEval => {
                if let Some(o) = obs {
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= self.cfg.stall_eps
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                }
                if self.still >= self.cfg.stall_polls {
                    self.phase = CPhase::SeekOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = CPhase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            CPhase::SeekOff => {
                self.phase = CPhase::SwitchTorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            // mode changes ride a torque-off gap: the enable edge reseeds
            // the loop chain cleanly (kernel run-edge contract)
            CPhase::SwitchTorqueOff => {
                self.phase = CPhase::SwitchMode;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            CPhase::SwitchMode => {
                self.phase = CPhase::SwitchTorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 1,
                }
            }
            CPhase::SwitchTorqueOn => {
                self.step_idx = 0;
                self.phase = CPhase::StepSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            CPhase::StepSet => {
                let goal = self.dir() as i32 * self.cfg.steps_counts[self.step_idx] as i32;
                self.polls_left = self.cfg.dwell_polls;
                self.windows.mark_transition();
                self.cur = Some(StepCapture {
                    goal: goal as i16,
                    t0: None,
                    windows: Vec::new(),
                });
                self.phase = CPhase::StepRead;
                Cmd::Write {
                    reg: control::GOAL_CURRENT,
                    value: goal,
                }
            }
            CPhase::StepRead => {
                self.phase = CPhase::StepEval;
                Cmd::Read
            }
            CPhase::StepEval => {
                if let Some(o) = obs
                    && let Some(w) = self.windows.push(o)
                    && let Some(c) = self.cur.as_mut()
                {
                    c.t0.get_or_insert(w.t_ms);
                    c.windows.push(w);
                }
                self.polls_left = self.polls_left.saturating_sub(1);
                if self.polls_left == 0 {
                    self.phase = CPhase::StepOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = CPhase::StepRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            CPhase::StepOff => {
                if let Some(c) = self.cur.take() {
                    self.captures.push(c);
                }
                self.phase = CPhase::StepRest;
                Cmd::Write {
                    reg: control::GOAL_CURRENT,
                    value: 0,
                }
            }
            CPhase::StepRest => {
                self.step_idx += 1;
                if self.step_idx < self.cfg.steps_counts.len() {
                    self.phase = CPhase::StepSet;
                } else if self.dir_idx == 0 {
                    self.dir_idx = 1;
                    self.phase = CPhase::DirTorqueOff;
                } else {
                    self.phase = CPhase::FinishGoal;
                }
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            CPhase::DirTorqueOff => {
                self.phase = CPhase::DirModeOpen;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            CPhase::DirModeOpen => {
                self.phase = CPhase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            CPhase::FinishGoal => {
                self.phase = CPhase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_CURRENT,
                    value: 0,
                }
            }
            CPhase::FinishTorque => {
                self.phase = CPhase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            CPhase::Finished => Cmd::Done,
        }
    }
}

/// E6: Velocity-mode legs across the travel; the pot slope vs goal is the
/// tracking check. Legs alternate direction so each ends where the next
/// starts.
pub struct VerifyVelocityCfg {
    pub seek_duty_q15: i16,
    /// Leg speeds, c/s; keep under the table velocity limit.
    pub legs_cps: Vec<i32>,
    pub poll_ms: u32,
    pub rest_ms: u32,
    pub seek_poll_ms: u32,
    /// Travel margin off each guard edge where a leg turns around. Wide
    /// enough that goal-0 braking on unproven gains stays inside the
    /// guard through the rest pause (no envelope read until the next leg).
    pub margin: u16,
    /// Park distance off the low guard edge for the open-loop seek.
    /// Separate from `margin`: the seek cannot brake, and the coast after
    /// seek-off runs through the mode-switch writes with no envelope read
    /// in between (bench: a 26% seek parked at margin 350 coasted into
    /// the low rail and the first leg read aborted at pos 8).
    pub seek_margin: u16,
    /// Head trim before the slope fit (trajectory accel ramp), ms.
    pub accel_trim_ms: f64,
    pub tol: f64,
    /// Give-up cap per leg (stall protection), polls.
    pub leg_cap_polls: u32,
}

impl Default for VerifyVelocityCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 8520,
            legs_cps: vec![600, 1200],
            poll_ms: 4,
            rest_ms: 300,
            seek_poll_ms: 30,
            margin: 550,
            seek_margin: 1300,
            accel_trim_ms: 250.0,
            tol: 0.15,
            leg_cap_polls: 4000,
        }
    }
}

#[derive(Clone, Debug)]
pub struct VelocityLeg {
    pub goal_cps: i32,
    pub meas_cps: f64,
    pub err_pct: f64,
    pub r2: f64,
    pub n: usize,
    pub pass: bool,
}

#[derive(Clone, Debug)]
pub struct VerifyVelocityResult {
    pub legs: Vec<VelocityLeg>,
    pub pass: bool,
    pub warnings: Vec<String>,
}

enum VPhase {
    ModeOpen,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    SeekOff,
    SwitchTorqueOff,
    SwitchMode,
    SwitchTorqueOn,
    LegSet,
    LegRead,
    LegEval,
    LegOff,
    LegRest,
    FinishGoal,
    FinishTorque,
    Finished,
}

struct LegCapture {
    goal_cps: i32,
    /// (t_s from sample_tick, pos) pairs, slip-zone samples excluded.
    pts: Vec<(f64, f64)>,
}

pub struct VerifyVelocity {
    cfg: VerifyVelocityCfg,
    params: RigParams,
    band: (u16, u16),
    tick_hz: f64,
    phase: VPhase,
    leg_idx: usize,
    polls: u32,
    last_pos: Option<u16>,
    still: u32,
    tick0: Option<u32>,
    cur: Option<LegCapture>,
    captures: Vec<LegCapture>,
    warnings: Vec<String>,
}

impl VerifyVelocity {
    pub fn new(cfg: VerifyVelocityCfg, params: &RigParams, tick_hz: f64) -> Self {
        let band = params.pos_guard.unwrap_or((150, 3950));
        Self {
            cfg,
            params: *params,
            band,
            tick_hz,
            phase: VPhase::ModeOpen,
            leg_idx: 0,
            polls: 0,
            last_pos: None,
            still: 0,
            tick0: None,
            cur: None,
            captures: Vec::new(),
            warnings: Vec::new(),
        }
    }

    /// Leg list: each speed runs + then -, back and forth across the band.
    fn goal(&self) -> i32 {
        let speed = self.cfg.legs_cps[self.leg_idx / 2];
        if self.leg_idx.is_multiple_of(2) {
            speed
        } else {
            -speed
        }
    }

    fn leg_done(&self, pos: u16) -> bool {
        if self.goal() > 0 {
            pos >= self.band.1 - self.cfg.margin
        } else {
            pos <= self.band.0 + self.cfg.margin
        }
    }

    pub fn result(&self) -> VerifyVelocityResult {
        let mut legs = Vec::new();
        for c in &self.captures {
            let t_start = c.pts.first().map(|p| p.0).unwrap_or(0.0);
            let pts: Vec<(f64, f64)> = c
                .pts
                .iter()
                .filter(|p| (p.0 - t_start) * 1000.0 >= self.cfg.accel_trim_ms)
                .copied()
                .collect();
            let Some(f) = linear_ls(&pts) else {
                legs.push(VelocityLeg {
                    goal_cps: c.goal_cps,
                    meas_cps: 0.0,
                    err_pct: 100.0,
                    r2: 0.0,
                    n: pts.len(),
                    pass: false,
                });
                continue;
            };
            let goal = c.goal_cps as f64;
            let err_pct = (f.b - goal).abs() / goal.abs() * 100.0;
            legs.push(VelocityLeg {
                goal_cps: c.goal_cps,
                meas_cps: f.b,
                err_pct,
                r2: f.r2,
                n: f.n,
                pass: err_pct <= self.cfg.tol * 100.0,
            });
        }
        let pass = !legs.is_empty() && legs.iter().all(|l| l.pass);
        VerifyVelocityResult {
            legs,
            pass,
            warnings: self.warnings.clone(),
        }
    }
}

impl Experiment for VerifyVelocity {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            VPhase::ModeOpen => {
                self.phase = VPhase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            VPhase::TorqueOn => {
                self.phase = VPhase::SeekSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            // park at the low edge so the first (+) leg has the full band
            VPhase::SeekSet => {
                self.phase = VPhase::SeekRead;
                self.last_pos = None;
                self.still = 0;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: -(self.cfg.seek_duty_q15 as i32),
                }
            }
            VPhase::SeekRead => {
                self.phase = VPhase::SeekEval;
                Cmd::Read
            }
            VPhase::SeekEval => {
                let mut parked = false;
                if let Some(o) = obs {
                    parked = o.pos <= self.band.0 + self.cfg.seek_margin;
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= 3
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                }
                if parked || self.still >= 8 {
                    self.phase = VPhase::SeekOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = VPhase::SeekRead;
                    Cmd::Pause {
                        ms: self.cfg.seek_poll_ms,
                    }
                }
            }
            VPhase::SeekOff => {
                self.phase = VPhase::SwitchTorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            VPhase::SwitchTorqueOff => {
                self.phase = VPhase::SwitchMode;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            VPhase::SwitchMode => {
                self.phase = VPhase::SwitchTorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 2,
                }
            }
            VPhase::SwitchTorqueOn => {
                self.phase = VPhase::LegSet;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            VPhase::LegSet => {
                self.polls = 0;
                self.tick0 = None;
                self.cur = Some(LegCapture {
                    goal_cps: self.goal(),
                    pts: Vec::new(),
                });
                self.phase = VPhase::LegRead;
                Cmd::Write {
                    reg: control::GOAL_VELOCITY,
                    value: self.goal(),
                }
            }
            VPhase::LegRead => {
                self.phase = VPhase::LegEval;
                Cmd::Read
            }
            VPhase::LegEval => {
                let mut done = false;
                if let Some(o) = obs {
                    let t0 = *self.tick0.get_or_insert(o.sample_tick);
                    if let Some(c) = self.cur.as_mut()
                        && !self.params.in_slip(o.pos)
                    {
                        let t = o.sample_tick.wrapping_sub(t0) as f64 / self.tick_hz;
                        c.pts.push((t, o.pos as f64));
                    }
                    done = self.leg_done(o.pos);
                }
                self.polls += 1;
                if self.polls >= self.cfg.leg_cap_polls {
                    self.warnings
                        .push(format!("leg {}: capped before the far edge", self.goal()));
                    done = true;
                }
                if done {
                    self.phase = VPhase::LegOff;
                    Cmd::Pause { ms: 0 }
                } else {
                    self.phase = VPhase::LegRead;
                    Cmd::Pause {
                        ms: self.cfg.poll_ms,
                    }
                }
            }
            VPhase::LegOff => {
                if let Some(c) = self.cur.take() {
                    self.captures.push(c);
                }
                self.phase = VPhase::LegRest;
                Cmd::Write {
                    reg: control::GOAL_VELOCITY,
                    value: 0,
                }
            }
            VPhase::LegRest => {
                self.leg_idx += 1;
                if self.leg_idx < self.cfg.legs_cps.len() * 2 {
                    self.phase = VPhase::LegSet;
                } else {
                    self.phase = VPhase::FinishGoal;
                }
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            VPhase::FinishGoal => {
                self.phase = VPhase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_VELOCITY,
                    value: 0,
                }
            }
            VPhase::FinishTorque => {
                self.phase = VPhase::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            VPhase::Finished => Cmd::Done,
        }
    }
}

/// Combined E5+E6 verdict the CLI assembles.
#[derive(Clone, Debug)]
pub struct VerifyResult {
    pub current: Option<VerifyCurrentResult>,
    pub velocity: Option<VerifyVelocityResult>,
    pub pass: bool,
}

impl VerifyResult {
    pub fn assemble(
        current: Option<VerifyCurrentResult>,
        velocity: Option<VerifyVelocityResult>,
    ) -> Self {
        let pass = current.as_ref().is_none_or(|c| c.pass)
            && velocity.as_ref().is_none_or(|v| v.pass)
            && (current.is_some() || velocity.is_some());
        Self {
            current,
            velocity,
            pass,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::regs::control;

    /// Scripted E5: the "servo" holds pos still during seeks and echoes the
    /// last goal_current into i_mean with a fixed -5% bias.
    #[test]
    fn current_verify_measures_settle_and_error() {
        let mut exp = VerifyCurrent::new(VerifyCurrentCfg::default(), &RigParams::default());
        let goal = std::cell::Cell::new(0i16);
        let seq = std::cell::Cell::new(0u16);
        let mut log = Vec::new();
        let mut pending: Option<TelemetrySnapshot> = None;
        for _ in 0..2_000_000 {
            match exp.step(pending.take().as_ref()) {
                Cmd::Write { reg, value } => {
                    if reg == control::GOAL_CURRENT {
                        goal.set(value as i16);
                    }
                    log.push((reg, value));
                }
                Cmd::Read => {
                    seq.set(seq.get().wrapping_add(1));
                    pending = Some(TelemetrySnapshot {
                        pos: 2000,
                        agg_seq: seq.get(),
                        i_mean_counts: (goal.get() as f64 * 0.95) as i16,
                        duty_mean_q15: if goal.get() != 0 { 6000 } else { 0 },
                        ..Default::default()
                    });
                }
                Cmd::Pause { .. } => {}
                Cmd::Done => break,
            }
        }
        let r = exp.result();
        assert_eq!(r.steps.len(), 4, "2 amplitudes x 2 directions");
        for s in &r.steps {
            assert!(s.pass, "step {} err {:.1}%", s.goal, s.err_pct);
            assert!((s.err_pct - 5.0).abs() < 1.5, "err {:.2}", s.err_pct);
            assert!(s.settle_ms.is_some());
        }
        assert!(r.pass);
        // mode switch rides a torque-off gap, both directions
        let mode_writes: Vec<i32> = log
            .iter()
            .filter(|(r, _)| *r == control::MODE)
            .map(|(_, v)| *v)
            .collect();
        assert_eq!(mode_writes, [0, 1, 0, 1], "open, current, per direction");
    }

    /// Scripted E6: pos advances at 97% of the goal each 4 ms poll.
    #[test]
    fn velocity_verify_measures_tracking() {
        let mut exp = VerifyVelocity::new(
            VerifyVelocityCfg::default(),
            &RigParams::default(),
            20_100.0,
        );
        let goal = std::cell::Cell::new(0i32);
        let pos = std::cell::Cell::new(2000.0f64);
        let tick = std::cell::Cell::new(0u32);
        let mut pending: Option<TelemetrySnapshot> = None;
        for _ in 0..4_000_000 {
            match exp.step(pending.take().as_ref()) {
                Cmd::Write { reg, value } => {
                    if reg == control::GOAL_VELOCITY {
                        goal.set(value);
                    }
                    if reg == control::GOAL_DUTY && value < 0 {
                        pos.set(400.0); // seek lands at the low edge
                    }
                }
                Cmd::Read => {
                    // 4 ms of motion at 97% tracking
                    pos.set((pos.get() + goal.get() as f64 * 0.97 * 0.004).clamp(160.0, 3940.0));
                    tick.set(tick.get().wrapping_add(80)); // 4 ms of 20.1k ticks
                    pending = Some(TelemetrySnapshot {
                        pos: pos.get() as u16,
                        sample_tick: tick.get(),
                        agg_seq: (tick.get() / 16) as u16,
                        ..Default::default()
                    });
                }
                Cmd::Pause { .. } => {}
                Cmd::Done => break,
            }
        }
        let r = exp.result();
        assert_eq!(r.legs.len(), 4, "2 speeds x 2 directions");
        for l in &r.legs {
            assert!(l.pass, "leg {} err {:.1}%", l.goal_cps, l.err_pct);
            assert!((l.err_pct - 3.0).abs() < 1.0, "err {:.2}", l.err_pct);
        }
        assert!(r.pass);
    }

    #[test]
    fn assemble_requires_at_least_one_section() {
        assert!(!VerifyResult::assemble(None, None).pass);
    }
}
