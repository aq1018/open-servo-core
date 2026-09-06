//! End-stop finder + drive-direction check: open-loop, seek each mechanical
//! hard-stop at modest duty, record the settled physical pot count at each
//! end, and infer drive polarity from which rail a positive duty reached.
//!
//! Run with the pos guard disabled ([`super::RigParams::without_pos_guard`]):
//! driving into the physical ends is the method. The current abort stays
//! live so a stalled winding is not cooked; the seek duty mirrors
//! resistance.rs's stall duty so the stall current stays well under it.

use super::{Cmd, Experiment, RigParams};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct EndstopCfg {
    /// Seek drive toward each end, q15 (mirrors resistance's 26% seek).
    pub seek_duty_q15: i16,
    pub seek_poll_ms: u32,
    /// Cool-down between the two seeks, duty 0.
    pub rest_ms: u32,
}

impl Default for EndstopCfg {
    fn default() -> Self {
        Self {
            seek_duty_q15: 8520,
            seek_poll_ms: 30,
            rest_ms: 300,
        }
    }
}

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct EndstopResult {
    pub pos_min_phys: i32,
    pub pos_max_phys: i32,
    /// true = positive duty increased position counts (matches the firmware
    /// `ConfigLimits::drive_polarity` semantics).
    pub drive_polarity: bool,
    /// Larger of the two stall currents, counts - confirms the motor
    /// actually loaded against a hard stop rather than settling idle.
    pub i_stall_counts: i16,
}

enum Phase {
    ModeWrite,
    TorqueOn,
    SeekSet,
    SeekRead,
    SeekEval,
    RestOff,
    RestPause,
    FinishDuty,
    FinishTorque,
    Finished,
}

pub struct Endstop {
    cfg: EndstopCfg,
    phase: Phase,
    stall_eps: u16,
    stall_polls: u32,
    dir_idx: u8,
    last_pos: Option<u16>,
    still: u32,
    /// Settled pos + stall current recorded per seek direction: index 0 is
    /// the positive-duty rail, index 1 the negative-duty rail.
    rail: [Option<(u16, i16)>; 2],
}

impl Endstop {
    pub fn new(cfg: EndstopCfg, params: &RigParams) -> Self {
        Self {
            cfg,
            phase: Phase::ModeWrite,
            stall_eps: params.stall_eps,
            stall_polls: params.stall_polls,
            dir_idx: 0,
            last_pos: None,
            still: 0,
            rail: [None; 2],
        }
    }

    fn dir(&self) -> i8 {
        if self.dir_idx == 0 { 1 } else { -1 }
    }

    /// `None` until both rails have been reached (a guard abort leaves one
    /// side unrecorded).
    pub fn result(&self) -> Option<EndstopResult> {
        let (p_pos, i_pos) = self.rail[0]?;
        let (p_neg, i_neg) = self.rail[1]?;
        let (p_pos, p_neg) = (p_pos as i32, p_neg as i32);
        Some(EndstopResult {
            pos_min_phys: p_pos.min(p_neg),
            pos_max_phys: p_pos.max(p_neg),
            drive_polarity: p_pos > p_neg,
            i_stall_counts: i_pos.abs().max(i_neg.abs()),
        })
    }
}

impl Experiment for Endstop {
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
                self.phase = Phase::SeekRead;
                self.last_pos = None;
                self.still = 0;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.dir() as i32 * self.cfg.seek_duty_q15 as i32,
                }
            }
            Phase::SeekRead => {
                self.phase = Phase::SeekEval;
                Cmd::Read
            }
            Phase::SeekEval => {
                if let Some(o) = obs {
                    if let Some(last) = self.last_pos
                        && o.pos.abs_diff(last) <= self.stall_eps
                    {
                        self.still += 1;
                    } else {
                        self.still = 0;
                    }
                    self.last_pos = Some(o.pos);
                    if self.still >= self.stall_polls {
                        self.rail[self.dir_idx as usize] = Some((o.pos, o.i_mean_counts));
                        self.phase = Phase::RestOff;
                        return Cmd::Pause { ms: 0 };
                    }
                }
                self.phase = Phase::SeekRead;
                Cmd::Pause {
                    ms: self.cfg.seek_poll_ms,
                }
            }
            Phase::RestOff => {
                self.phase = Phase::RestPause;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::RestPause => {
                if self.dir_idx == 0 {
                    self.dir_idx = 1;
                    self.phase = Phase::SeekSet;
                } else {
                    self.phase = Phase::FinishDuty;
                }
                Cmd::Pause {
                    ms: self.cfg.rest_ms,
                }
            }
            Phase::FinishDuty => {
                self.phase = Phase::FinishTorque;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
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

    fn run(servo: &mut FakeServo) -> (Endstop, Vec<String>) {
        let params = RigParams::default().without_pos_guard();
        let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
        let log = pump(&mut exp, servo, 2_000_000);
        assert!(exp.abort().is_none(), "abort: {:?}", exp.abort());
        (exp.into_inner(), log)
    }

    #[test]
    fn recovers_rails_and_polarity() {
        let mut servo = FakeServo::new(3.37);
        servo.ends = (321.0, 3702.0);
        servo.pos = 2000.0;
        let (exp, log) = run(&mut servo);
        assert!(!log.contains(&"OVERRUN".to_string()));
        let r = exp.result().expect("both rails found");
        assert!((r.pos_min_phys - 321).abs() <= 3, "min {}", r.pos_min_phys);
        assert!((r.pos_max_phys - 3702).abs() <= 3, "max {}", r.pos_max_phys);
        assert!(r.drive_polarity, "positive duty should raise counts");
        assert!(r.i_stall_counts > 0, "stall current {}", r.i_stall_counts);
    }

    #[test]
    fn polarity_inference_flips_when_wiring_reversed() {
        let mut servo = FakeServo::new(3.37);
        servo.ends = (321.0, 3702.0);
        servo.pos = 2000.0;
        servo.drive_polarity = false;
        let (exp, _) = run(&mut servo);
        let r = exp.result().expect("both rails found");
        assert!(!r.drive_polarity, "reversed wiring must flip inference");
        // rails ordered regardless of which duty sign reached which end
        assert!(r.pos_min_phys < r.pos_max_phys);
        assert!((r.pos_min_phys - 321).abs() <= 3, "min {}", r.pos_min_phys);
        assert!((r.pos_max_phys - 3702).abs() <= 3, "max {}", r.pos_max_phys);
    }

    #[test]
    fn command_choreography_is_safe() {
        let mut servo = FakeServo::new(3.37);
        let (_, log) = run(&mut servo);
        // torque on before any nonzero duty
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .expect("torque on");
        let first_duty = log
            .iter()
            .position(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .expect("a drive command");
        assert!(torque_on < first_duty);
        // rest between the two seeks
        let zeros = log.iter().filter(|l| *l == "write goal_duty 0").count();
        assert!(zeros >= 2, "one rest plus final safety, got {zeros}");
        // final safety pair
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }
}
