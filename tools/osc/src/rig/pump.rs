//! The four-arm driver loop from osc-ident's exp module doc: Write ->
//! wire write, Read -> telemetry gread + parse, Pause -> sleep in slices
//! that drain TEL and honor ctrl-c, Done -> break.

use std::sync::atomic::{AtomicBool, Ordering};
use std::time::{Duration, Instant};

use anyhow::{Context, Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::{Cmd, Experiment};
use osc_ident::frame::{TelFrame, TelemetrySnapshot};
use osc_ident::regs::{Reg, control, telemetry};

use super::csvio::SnapshotLog;
use super::tel::TelSink;

pub(crate) static STOP: AtomicBool = AtomicBool::new(false);

pub(crate) fn install_ctrlc() {
    let _ = ctrlc::set_handler(|| STOP.store(true, Ordering::SeqCst));
}

/// Write one register, value LE-truncated to the field width (negative
/// i32 -> correct two's complement for 2/4-byte fields).
pub(crate) fn write_reg(c: &mut Client<NusbPipe>, id: Id, reg: Reg, value: i32) -> Result<()> {
    let bytes = value.to_le_bytes();
    c.write(id, reg.addr, &bytes[..reg.width as usize])
        .with_context(|| format!("write addr {:#06x}", reg.addr))?;
    Ok(())
}

const TEL_BASE: u16 = telemetry::FAULT_FLAGS.addr;
const TEL_LEN: u16 = telemetry::AGG_SEQ.addr + 2 - TEL_BASE;
const IDENT_BASE: u16 = telemetry::I_MEAN_COUNTS.addr;
const IDENT_LEN: u16 = telemetry::AGG_SEQ.addr + 2 - IDENT_BASE;

/// One telemetry snapshot with the torn-ident-window guard: the full read
/// is paired with a 12 B re-read of the ident block, and only a pair whose
/// agg_seq agrees is returned (the block is written mid-tick; agg_seq lands
/// last). Bounded retries - a stubbornly torn read returns the last full
/// snapshot, which the engine's WindowStream then dedups by seq anyway.
pub(crate) fn read_snapshot(c: &mut Client<NusbPipe>, id: Id) -> Result<TelemetrySnapshot> {
    let mut last = None;
    for _ in 0..3 {
        let raw = c.read(id, TEL_BASE, TEL_LEN).context("telemetry read")?;
        let snap =
            TelemetrySnapshot::parse(TEL_BASE, &raw).context("telemetry parse (short read?)")?;
        let ib = c.read(id, IDENT_BASE, IDENT_LEN).context("ident re-read")?;
        let re_seq = u16::from_le_bytes([ib[10], ib[11]]);
        if re_seq == snap.agg_seq {
            return Ok(snap);
        }
        last = Some(snap);
    }
    Ok(last.expect("loop ran"))
}

pub(crate) struct Pump<'a> {
    pub(crate) client: &'a mut Client<NusbPipe>,
    pub(crate) id: Id,
    /// TEL device path; the sink opens on the experiment's tel_enable=1
    /// write and closes (frames flushed) on tel_enable=0.
    pub(crate) tel_port: Option<String>,
    pub(crate) tel_mask: u16,
    pub(crate) log: Option<&'a mut SnapshotLog>,
    /// Lossless raw-byte capture of the TEL stream, in addition to the live
    /// deframing; None skips the file (e.g. endstop has no ripple sweep).
    pub(crate) tel_raw_path: Option<std::path::PathBuf>,
}

impl Pump<'_> {
    /// Run one experiment to completion. `on_tel` receives decoded TEL
    /// frames as they drain (inertia's push_tel; pass |_| {} otherwise).
    pub(crate) fn run(
        &mut self,
        exp: &mut dyn Experiment,
        mut on_tel: impl FnMut(&[TelFrame]),
    ) -> Result<()> {
        let mut pending: Option<TelemetrySnapshot> = None;
        let mut sink: Option<TelSink> = None;
        let t0 = Instant::now();
        loop {
            if STOP.load(Ordering::SeqCst) {
                bail!("interrupted");
            }
            let cmd = exp.step(pending.take().as_ref());
            if let Some(s) = sink.as_mut() {
                s.drain();
                let frames = s.take_frames();
                if !frames.is_empty() {
                    on_tel(&frames);
                }
            }
            match cmd {
                Cmd::Write { reg, value } => {
                    if reg == control::TEL_ENABLE && self.tel_port.is_some() {
                        if value != 0 && sink.is_none() {
                            sink = Some(TelSink::open(
                                self.tel_port.as_deref().expect("checked"),
                                self.tel_mask,
                                self.tel_raw_path.as_deref(),
                            )?);
                        } else if value == 0
                            && let Some(mut s) = sink.take()
                        {
                            s.drain();
                            let frames = s.take_frames();
                            if !frames.is_empty() {
                                on_tel(&frames);
                            }
                            let st = s.stats();
                            eprintln!(
                                "tel: {} bytes, {} frames, {} seq gaps, {} realigns",
                                s.bytes_read(),
                                st.frames,
                                st.seq_gaps,
                                st.realigns
                            );
                        }
                    }
                    write_reg(self.client, self.id, reg, value)?;
                }
                Cmd::Read => {
                    let snap = read_snapshot(self.client, self.id)?;
                    if let Some(log) = self.log.as_mut() {
                        log.push(t0.elapsed().as_secs_f64() * 1000.0, &snap)?;
                    }
                    pending = Some(snap);
                }
                Cmd::Pause { ms } => {
                    // 5 ms slices keep the CDC drained and ctrl-c prompt
                    let mut left = ms;
                    loop {
                        if STOP.load(Ordering::SeqCst) {
                            bail!("interrupted");
                        }
                        if let Some(s) = sink.as_mut() {
                            s.drain();
                            let frames = s.take_frames();
                            if !frames.is_empty() {
                                on_tel(&frames);
                            }
                        }
                        if left == 0 {
                            break;
                        }
                        let slice = left.min(5);
                        std::thread::sleep(Duration::from_millis(slice as u64));
                        left -= slice;
                    }
                }
                Cmd::Done => return Ok(()),
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn write_arm_truncates_by_width() {
        // the exact bytes the Write arm sends per width
        let cases: [(i32, u8, &[u8]); 4] = [
            (1, 1, &[0x01]),
            (-9000, 2, &(-9000i16).to_le_bytes()),
            (0x1B, 2, &[0x1B, 0x00]),
            (-1500, 4, &(-1500i32).to_le_bytes()),
        ];
        for (value, width, want) in cases {
            let bytes = value.to_le_bytes();
            assert_eq!(
                &bytes[..width as usize],
                want,
                "value {value} width {width}"
            );
        }
    }

    #[test]
    fn telemetry_span_covers_the_ident_block() {
        assert_eq!(TEL_BASE, 0x200);
        assert_eq!(TEL_LEN, 0x60);
        assert_eq!(IDENT_BASE, 0x254);
        assert_eq!(IDENT_LEN, 12);
    }
}
