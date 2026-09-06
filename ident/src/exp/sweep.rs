//! Dedicated constant-duty ripple capture: one clean constant-duty traverse
//! whose per-tick TEL current carries an uninterrupted commutation-ripple
//! signal for the tachometer and pot LUT.
//!
//! Why a separate motion and not the end-stop seek: the seek must poll pos to
//! detect each stall, and every poll is a bus read the pump cannot drain TEL
//! through - so the seek capture is shredded into sub-millisecond seq-
//! contiguous fragments, too short for the autocorr window, and it spends
//! most of its time dwelling stalled at a rail (flat current, no ripple).
//! This experiment instead drives one fixed duty and does nothing but Pause,
//! so the pump drains TEL continuously and the whole traverse lands as a
//! single long contiguous run.
//!
//! No positioning or speed probe here: the caller positions to the capture
//! start rail (with TEL off, where polling is free) and sizes capture_ms from
//! a measured speed so the traverse stays off BOTH mechanical rails - the
//! clone can jam at either end, so the capture must never reach a stop.
//!
//! No safety Reads during the traverse (they would re-fragment the capture):
//! the time-bounded, speed-sized duration is the backstop, and the firmware
//! current limit + fault protection guard the winding. The caller parks duty 0
//! + torque off on exit.

use super::{Cmd, Experiment};
use crate::frame::TelemetrySnapshot;
use crate::regs::control;

pub struct SweepCfg {
    /// Constant capture drive magnitude, q15 (mirrors the end-stop seek duty so
    /// the winding load is the same proven-safe operating point).
    pub duty_q15: i16,
    /// Constant-duty capture duration. The caller sizes this from a measured
    /// speed so the motion covers the count-inset span without reaching a rail.
    pub capture_ms: u32,
}

impl Default for SweepCfg {
    fn default() -> Self {
        Self {
            duty_q15: 8520,
            capture_ms: 800,
        }
    }
}

enum Phase {
    ModeWrite,
    TorqueOn,
    CaptureDuty,
    CapturePause,
    ZeroDuty,
    TorqueOff,
    Finished,
}

pub struct Sweep {
    cfg: SweepCfg,
    phase: Phase,
    /// Duty sign that drives pos toward the capture end (increasing pos); the
    /// caller derives it from the measured drive polarity.
    sweep_sign: i8,
}

impl Sweep {
    pub fn new(cfg: SweepCfg, sweep_sign: i8) -> Self {
        Self {
            cfg,
            phase: Phase::ModeWrite,
            sweep_sign: if sweep_sign < 0 { -1 } else { 1 },
        }
    }

    fn capture_duty(&self) -> i32 {
        self.sweep_sign as i32 * self.cfg.duty_q15 as i32
    }
}

impl Experiment for Sweep {
    fn step(&mut self, _obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            Phase::ModeWrite => {
                self.phase = Phase::TorqueOn;
                Cmd::Write {
                    reg: control::MODE,
                    value: 0,
                }
            }
            Phase::TorqueOn => {
                self.phase = Phase::CaptureDuty;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 1,
                }
            }
            Phase::CaptureDuty => {
                self.phase = Phase::CapturePause;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: self.capture_duty(),
                }
            }
            Phase::CapturePause => {
                self.phase = Phase::ZeroDuty;
                Cmd::Pause {
                    ms: self.cfg.capture_ms,
                }
            }
            Phase::ZeroDuty => {
                self.phase = Phase::TorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            Phase::TorqueOff => {
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
    use super::*;

    fn run(sweep_sign: i8) -> Vec<String> {
        let mut servo = FakeServo::new(3.37);
        servo.ends = (321.0, 3702.0);
        servo.pos = 2000.0;
        let mut exp = Sweep::new(SweepCfg::default(), sweep_sign);
        let log = pump(&mut exp, &mut servo, 2_000);
        assert!(!log.contains(&"OVERRUN".to_string()));
        log
    }

    #[test]
    fn choreography_is_safe() {
        let log = run(1);
        // mode set before anything drives
        assert_eq!(log[0], "write mode 0");
        // torque on before any nonzero duty
        let torque_on = log
            .iter()
            .position(|l| l == "write torque_enable 1")
            .unwrap();
        let first_duty = log
            .iter()
            .position(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .unwrap();
        assert!(torque_on < first_duty);
        // single nonzero capture duty (no prime): exactly one drive write
        let duties: Vec<&String> = log
            .iter()
            .filter(|l| l.starts_with("write goal_duty"))
            .collect();
        assert_eq!(duties[0], "write goal_duty 8520", "capture toward far rail");
        // ends parked: duty 0 then torque off
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }

    #[test]
    fn negative_sign_flips_capture_duty() {
        let pos = run(1);
        let neg = run(-1);
        let pos_duty = pos
            .iter()
            .find(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .unwrap();
        let neg_duty = neg
            .iter()
            .find(|l| l.starts_with("write goal_duty") && !l.ends_with(" 0"))
            .unwrap();
        assert_eq!(pos_duty, "write goal_duty 8520");
        assert_eq!(neg_duty, "write goal_duty -8520");
    }
}
