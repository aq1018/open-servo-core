//! Sans-io experiment engine. An [`Experiment`] is a state machine: each
//! `step` consumes at most one observation (the reply to a previous
//! [`Cmd::Read`]) and emits the next command. The driver - CLI over USB
//! today, wasm GUI over Web Serial later - owns all IO and time:
//!
//! ```ignore
//! let mut pending = None;
//! loop {
//!     match exp.step(pending.take().as_ref()) {
//!         Cmd::Write { reg, value } => client.write(reg, value),
//!         Cmd::Read => pending = Some(read_telemetry_region()),
//!         Cmd::Pause { ms } => sleep_ms(ms),
//!         Cmd::Done => break,
//!     }
//! }
//! ```
//!
//! [`Guarded`] wraps any experiment with the safety envelope; rig limits
//! live in [`RigParams`], the one home for bench constants.

pub mod bias;
pub mod breakaway;
pub mod endstop;
pub mod inertia;
pub mod ladder;
pub mod resistance;
pub mod sweep;
pub mod verify;

use crate::frame::{SeqUnwrap, TelemetrySnapshot};
use crate::regs::{Reg, control};

/// One driver action. `Write` is a single-field wire write (the value is
/// truncated to the reg width by the driver); `Read` is one gread of the
/// telemetry region whose parsed snapshot feeds the NEXT `step`.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Cmd {
    Write { reg: Reg, value: i32 },
    Read,
    Pause { ms: u32 },
    Done,
}

/// A pumpable experiment. `step` with `Some` only in reply to [`Cmd::Read`].
pub trait Experiment {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd;
}

/// Rig constants with bench defaults - the single home. Experiments take
/// what they need; the CLI overrides via flags.
#[derive(Copy, Clone, Debug)]
pub struct RigParams {
    /// Soft travel guard; `None` disables (end-stop experiments stall at
    /// the physical ends on purpose - see [`RigParams::without_pos_guard`]).
    pub pos_guard: Option<(u16, u16)>,
    /// Abort threshold on the ident-window current mean, counts. Checked
    /// only while duty_mean is nonzero: at torque-off the ident block
    /// holds its last driven value and would trip forever.
    pub i_abort: i16,
    /// Stripped-gear slip zone, masked from motion fits and avoided as a
    /// dwell region (consumed by the ladder/inertia experiments).
    pub slip: (u16, u16),
    /// Ident windows discarded after every duty change (L transient +
    /// window-boundary smear).
    pub settle_windows: u32,
    /// One ident aggregate window in ms: 16 fast ticks at ~20 kHz.
    pub agg_period_ms: f64,
    /// End-stop stall detect: a seek read whose pos moved <= `stall_eps`
    /// counts from the prior one counts as still; `stall_polls` consecutive
    /// still reads declare the mechanical rail.
    pub stall_eps: u16,
    pub stall_polls: u32,
}

impl Default for RigParams {
    fn default() -> Self {
        Self {
            pos_guard: Some((150, 3950)),
            i_abort: 1100,
            slip: (1250, 1650),
            settle_windows: 5,
            agg_period_ms: 0.8,
            stall_eps: 3,
            stall_polls: 8,
        }
    }
}

impl RigParams {
    pub fn without_pos_guard(self) -> Self {
        Self {
            pos_guard: None,
            ..self
        }
    }

    pub fn in_slip(&self, pos: u16) -> bool {
        (self.slip.0..=self.slip.1).contains(&pos)
    }
}

/// Why the envelope stopped an experiment.
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum AbortReason {
    /// The servo latched a fault (flags + code as read).
    Fault { flags: u8, code: u8 },
    /// Position left the soft guard band.
    PosGuard { pos: u16 },
    /// Ident-window current mean exceeded the abort threshold mid-drive.
    Overcurrent { i_mean: i16 },
}

enum GuardState {
    Run,
    DutyOff,
    TorqueOff,
    Finished,
}

/// Safety envelope: checks every observation, and on a violation preempts
/// the inner experiment with duty-0 + torque-off before reporting Done.
/// The inner experiment is left unstepped from that point on.
pub struct Guarded<E> {
    exp: E,
    params: RigParams,
    state: GuardState,
    abort: Option<AbortReason>,
}

impl<E: Experiment> Guarded<E> {
    pub fn new(exp: E, params: RigParams) -> Self {
        Self {
            exp,
            params,
            state: GuardState::Run,
            abort: None,
        }
    }

    /// `Some` once the envelope has tripped; the run's samples are partial.
    pub fn abort(&self) -> Option<AbortReason> {
        self.abort
    }

    pub fn into_inner(self) -> E {
        self.exp
    }

    /// Mid-run access for side channels (the driver hands TEL frames to a
    /// guarded [`inertia::Inertia`] between commands).
    pub fn inner_mut(&mut self) -> &mut E {
        &mut self.exp
    }

    fn violation(&self, o: &TelemetrySnapshot) -> Option<AbortReason> {
        if o.fault_flags != 0 {
            return Some(AbortReason::Fault {
                flags: o.fault_flags,
                code: o.fault_code,
            });
        }
        if let Some((lo, hi)) = self.params.pos_guard
            && !(lo..=hi).contains(&o.pos)
        {
            return Some(AbortReason::PosGuard { pos: o.pos });
        }
        if o.duty_mean_q15 != 0
            && o.i_mean_counts.unsigned_abs() > self.params.i_abort.unsigned_abs()
        {
            return Some(AbortReason::Overcurrent {
                i_mean: o.i_mean_counts,
            });
        }
        None
    }
}

impl<E: Experiment> Experiment for Guarded<E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        if matches!(self.state, GuardState::Run)
            && let Some(o) = obs
            && let Some(reason) = self.violation(o)
        {
            self.abort = Some(reason);
            self.state = GuardState::DutyOff;
        }
        match self.state {
            GuardState::Run => self.exp.step(obs),
            GuardState::DutyOff => {
                self.state = GuardState::TorqueOff;
                Cmd::Write {
                    reg: control::GOAL_DUTY,
                    value: 0,
                }
            }
            GuardState::TorqueOff => {
                self.state = GuardState::Finished;
                Cmd::Write {
                    reg: control::TORQUE_ENABLE,
                    value: 0,
                }
            }
            GuardState::Finished => Cmd::Done,
        }
    }
}

/// One accepted ident aggregate window, timebased on the unwrapped
/// `agg_seq` (x 0.8 ms) - poll jitter does not touch the fit clock.
#[derive(Copy, Clone, Debug, PartialEq)]
pub struct WindowSample {
    pub t_ms: f64,
    /// Signed bias-subtracted window current mean, counts.
    pub i: f64,
    /// Drive-window va - vb mean, vcounts.
    pub vdiff: f64,
    /// Commanded duty mean over the window, q15.
    pub duty_q15: f64,
}

/// Turns polled snapshots into deduplicated window samples: a sample is
/// accepted only when `agg_seq` advanced (torn or repeated reads yield
/// None), and the first `settle` windows after every duty transition are
/// discarded. The driver's re-read/agg_seq-match dance guards torn
/// aggregates; this stream guards duplicates and settling.
pub struct WindowStream {
    unwrap: SeqUnwrap,
    last: Option<u64>,
    settle: u32,
    settle_windows: u32,
    agg_period_ms: f64,
}

impl WindowStream {
    pub fn new(params: &RigParams) -> Self {
        Self {
            unwrap: SeqUnwrap::default(),
            last: None,
            settle: 0,
            settle_windows: params.settle_windows,
            agg_period_ms: params.agg_period_ms,
        }
    }

    /// Call on every duty change; the next `settle_windows` accepted
    /// windows are dropped.
    pub fn mark_transition(&mut self) {
        self.settle = self.settle_windows;
    }

    pub fn push(&mut self, o: &TelemetrySnapshot) -> Option<WindowSample> {
        let seq = self.unwrap.push(o.agg_seq);
        if self.last == Some(seq) {
            return None;
        }
        self.last = Some(seq);
        if self.settle > 0 {
            self.settle -= 1;
            return None;
        }
        Some(WindowSample {
            t_ms: seq as f64 * self.agg_period_ms,
            i: o.i_mean_counts as f64,
            vdiff: o.vdiff_mean as f64,
            duty_q15: o.duty_mean_q15 as f64,
        })
    }
}

#[cfg(test)]
pub(crate) mod testkit;

#[cfg(test)]
mod tests {
    use super::testkit::{FakeServo, pump};
    use super::*;

    #[test]
    fn torn_agg_seq_no_duplicate_sample() {
        let params = RigParams::default();
        let mut ws = WindowStream::new(&params);
        let mut o = TelemetrySnapshot {
            agg_seq: 7,
            i_mean_counts: 42,
            ..Default::default()
        };
        assert!(ws.push(&o).is_some());
        assert!(ws.push(&o).is_none(), "repeated agg_seq must not resample");
        o.agg_seq = 8;
        assert!(ws.push(&o).is_some());
    }

    #[test]
    fn settle_windows_discarded_after_transition() {
        let params = RigParams::default();
        let mut ws = WindowStream::new(&params);
        ws.mark_transition();
        let mut accepted = 0;
        for seq in 0..8u16 {
            let o = TelemetrySnapshot {
                agg_seq: seq,
                ..Default::default()
            };
            if ws.push(&o).is_some() {
                accepted += 1;
            }
        }
        assert_eq!(accepted, 3, "5 settle windows dropped from 8");
    }

    #[test]
    fn abort_on_fault_emits_safety_commands() {
        // an experiment that would poll forever
        struct Forever(bool);
        impl Experiment for Forever {
            fn step(&mut self, _: Option<&TelemetrySnapshot>) -> Cmd {
                self.0 = !self.0;
                if self.0 {
                    Cmd::Read
                } else {
                    Cmd::Pause { ms: 5 }
                }
            }
        }
        let mut servo = FakeServo::new(3.37);
        let mut exp = Guarded::new(Forever(false), RigParams::default());
        servo.fault_at_ms = Some(50.0);
        let log = pump(&mut exp, &mut servo, 10_000);
        assert!(matches!(exp.abort(), Some(AbortReason::Fault { .. })));
        let tail: Vec<&String> = log.iter().rev().take(2).collect();
        assert_eq!(*tail[1], "write goal_duty 0");
        assert_eq!(*tail[0], "write torque_enable 0");
    }

    #[test]
    fn abort_on_pos_guard_breach() {
        struct Drive(u8);
        impl Experiment for Drive {
            fn step(&mut self, _: Option<&TelemetrySnapshot>) -> Cmd {
                self.0 += 1;
                match self.0 {
                    1 => Cmd::Write {
                        reg: control::TORQUE_ENABLE,
                        value: 1,
                    },
                    2 => Cmd::Write {
                        reg: control::GOAL_DUTY,
                        value: 12000,
                    },
                    _ => {
                        if self.0 % 2 == 1 {
                            Cmd::Read
                        } else {
                            Cmd::Pause { ms: 30 }
                        }
                    }
                }
            }
        }
        let mut servo = FakeServo::new(3.37);
        servo.pos = 3800.0;
        let mut exp = Guarded::new(Drive(0), RigParams::default());
        let log = pump(&mut exp, &mut servo, 10_000);
        assert!(matches!(exp.abort(), Some(AbortReason::PosGuard { pos } ) if pos > 3950));
        assert_eq!(*log.last().unwrap(), "write torque_enable 0");
    }
}
