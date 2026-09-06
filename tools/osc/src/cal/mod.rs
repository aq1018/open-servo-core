//! `osc cal` -- the calibration orchestrator: find the mechanical end-stops,
//! confirm the real-world angle range with the operator, print the
//! count->unit report, then write the limits + drive polarity + angle
//! endpoints + gear + pot LUT and persist with MGMT SAVE. Interactive by
//! default; flags make it headless. With `--tel-port` the rail-to-rail seek
//! streams a TEL current+pos sweep: its commutation ripple gives a MEASURED
//! gear ratio (the gear prompt's default) and fills the pot linearization
//! LUT. Both are gear-2-dependent and degrade gracefully (identity LUT /
//! operator-input gear) when ripple SNR is low. The endstop state machine and
//! the kinematics/units/lut math live in osc-ident; this wrapper owns USB,
//! the TEL port, prompts, and files.

pub mod replay;

use std::path::Path;
use std::sync::atomic::Ordering;
use std::time::Duration;

use anyhow::{Context, Result, bail};
use dialoguer::{Confirm, Input};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::endstop::{Endstop, EndstopCfg, EndstopResult};
use osc_ident::exp::sweep::{Sweep, SweepCfg};
use osc_ident::exp::{Cmd, Experiment, Guarded, RigParams};
use osc_ident::frame::{TelFrame, TelemetrySnapshot};
use osc_ident::kinematics::{self, KinematicsResult, angle_endpoints};
use osc_ident::lut::{self, PotLut, build_multi, stitched_motor_revs};
use osc_ident::regs::{calib, config, control};
use osc_ident::slip;
use osc_ident::units::{self, SenseParams};

use crate::rig::csvio::{OutDir, SnapshotLog};
use crate::rig::pump::{self, Pump, read_snapshot, write_reg};
use crate::rig::snapshot::{self, read_u16};

/// Commutation events per rotor rev for the brushed 3-slot motor: the ripple
/// tach and the LUT angle clock both count 6 per revolution.
const RIPPLE_PER_REV: f64 = 6.0;

/// Ripple coverage (fraction of sweep windows that found ripple) below this
/// flags the gear estimate as low-confidence in the printout - still used, as
/// expected on a stripped gear where slip corrupts the pot displacement.
const RIPPLE_CONF_MIN: f64 = 0.7;

/// Assumed total rated travel when the operator gives no phys angle: a 0..180
/// deg span. Only a default; every real servo overrides it.
const DEFAULT_TRAVEL_DEG: f64 = 180.0;

/// Default soft-limit span, centered in the phys range (or the full phys
/// range when it is narrower).
const DEFAULT_SOFT_TRAVEL_DEG: f64 = 180.0;

/// `osc cal` args: output dir plus the headless overrides. `--baud`/`--id`
/// come from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// Output dir for the rollback snapshot + endstop log (default ./cal-out).
    #[arg(long)]
    out: Option<std::path::PathBuf>,
    /// Real angle (deg) at the min-count rail.
    #[arg(long)]
    phys_angle_min: Option<f64>,
    /// Real angle (deg) at the max-count rail.
    #[arg(long)]
    phys_angle_max: Option<f64>,
    /// App-facing working-range min (deg); defaults to the low edge of a
    /// 180 deg window centered in the phys range.
    #[arg(long)]
    soft_angle_min: Option<f64>,
    /// App-facing working-range max (deg); defaults to the high edge of a
    /// 180 deg window centered in the phys range.
    #[arg(long)]
    soft_angle_max: Option<f64>,
    /// Known gear ratio (motor rev per output rev); unset leaves it 0.
    #[arg(long)]
    gear_ratio: Option<f64>,
    /// TEL stream serial device (the LinkE CDC); empty disables the ripple
    /// sweep (no measured gear, LUT left untouched).
    #[arg(long, default_value = "")]
    tel_port: String,
    /// Assume yes: never prompt. Required values must come from flags.
    #[arg(long)]
    yes: bool,
}

/// Entry from the top-level `osc cal` dispatch.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    pump::install_ctrlc();
    let mut c = crate::rig::connect(&baud)?;
    let id = Id::new(id);

    let sense = read_sense(&mut c, id)?;
    // TEL frames arrive one per fast tick, so tick_hz is the sweep sample rate.
    let fs = sense.tick_hz as f64;
    let ke_vpc_q = read_u16(&mut c, id, calib::KE_VPC_Q)?;
    if ke_vpc_q == 0 {
        eprintln!("warning: ke_vpc_q is 0 - run `osc ident` first for the torque report");
    }

    // Safety gate before any motion: both mechanical ends get driven.
    if !args.yes {
        println!("cal drives the servo into BOTH mechanical end-stops.");
        if !confirm("proceed", false)? {
            println!("declined, no writes");
            return Ok(());
        }
    }

    let tel_port = (!args.tel_port.is_empty()).then(|| args.tel_port.clone());
    let out = OutDir::create(args.out.as_deref().unwrap_or(Path::new("./cal-out")))?;
    let r = run_endstop(&mut c, id, &out)?;
    let EndstopResult {
        pos_min_phys,
        pos_max_phys,
        drive_polarity,
        i_stall_counts,
    } = r;
    let span = pos_max_phys - pos_min_phys;
    println!(
        "rails: min {pos_min_phys} max {pos_max_phys} span {span} counts, polarity {}, stall {i_stall_counts} counts",
        if drive_polarity { "normal" } else { "reversed" },
    );

    // Dedicated constant-duty traverse = the ripple/LUT source. A real capture
    // fragments into several clean chunks (poll seams, brief dropouts), so the
    // LUT + anchor stitch ALL chunks (build_sweep_chunks) over the shared pos
    // axis; the single longest run (build_sweep) is kept only for the slip
    // health-check and the moving-run print. Skipped without --tel-port.
    let (sweep, chunks) = match &tel_port {
        Some(port) => {
            let tel = run_sweep(
                &mut c,
                id,
                port,
                pos_min_phys,
                pos_max_phys,
                drive_polarity,
                &out,
            )?;
            let s = build_sweep(&tel);
            let chunks = build_sweep_chunks(&tel);
            match &s {
                Some((pos, _)) => println!(
                    "[sweep] {} tel frames, moving run {} (fs {fs:.0} Hz from tick_hz)",
                    tel.len(),
                    pos.len()
                ),
                None => println!("[sweep] {} tel frames, no usable moving run", tel.len()),
            }
            (s, chunks)
        }
        None => (None, Vec::new()),
    };

    recenter(&mut c, id, pos_min_phys, pos_max_phys, drive_polarity)?;

    // Full-traverse motor revs from the ripple sweep: anchor-free geometry,
    // stitched over all chunks and extrapolated from the covered phase span to
    // the whole rail count span. Paired with a gear ratio it yields travel;
    // paired with a travel angle it yields the gear. Both anchors consume this
    // same ripple half. Empty chunks (no --tel-port) -> None.
    let motor_revs_full = stitched_motor_revs(
        &chunks,
        fs,
        RIPPLE_PER_REV,
        pos_min_phys as u16,
        pos_max_phys as u16,
    );
    let mrf = motor_revs_full.map(|(m, _)| m);
    let coverage = motor_revs_full.map(|(_, c)| c);
    if tel_port.is_some() {
        println!(
            "[sweep] {} clean chunks, stitched coverage {:.0}%",
            chunks.len(),
            coverage.unwrap_or(0.0) * 100.0
        );
    }

    // Resolve the anchor: gear is preferred, travel is the fallback. --yes stays
    // headless through the pure resolve_anchor; interactively the operator
    // anchors on the gear first (enter 0 to switch to a travel angle instead).
    let (phys_min, phys_max, gear_ratio_centi) = if args.yes {
        let (lo, hi, gear) = resolve_anchor(
            mrf,
            args.gear_ratio,
            args.phys_angle_min,
            args.phys_angle_max,
        );
        report_yes_anchor(args, mrf, coverage, lo, hi, gear);
        (lo, hi, gear)
    } else {
        resolve_anchor_interactive(args, mrf, coverage)?
    };

    let (soft_min_deg, soft_max_deg) = soft_angles(args, phys_min, phys_max)?;
    let (angle_min_cdeg, angle_max_cdeg) = angle_endpoints(phys_min, phys_max);
    let dpc = kinematics::deg_per_count(angle_min_cdeg, angle_max_cdeg, pos_min_phys, pos_max_phys);

    // pot LUT stitched from all chunks; identity when the stitched coverage is
    // too low (or the ripple SNR is too low to clock true angle) - build_multi
    // decides internally and returns identity in either case. No chunks at all
    // (no --tel-port) -> None, LUT left untouched below.
    let lut = (!chunks.is_empty()).then(|| {
        let l = build_multi(
            &chunks,
            fs,
            RIPPLE_PER_REV,
            pos_min_phys as u16,
            pos_max_phys as u16,
        );
        let populated = l.corr.iter().any(|&c| c != 0);
        (l, populated)
    });
    match &lut {
        Some((l, true)) => {
            let maxc = l.corr.iter().map(|&c| c.unsigned_abs()).max().unwrap_or(0);
            println!("pot LUT: populated, max |corr| {maxc} counts");
        }
        Some((_, false)) => println!(
            "pot LUT: identity (stitched coverage {:.0}% of travel)",
            coverage.unwrap_or(0.0) * 100.0
        ),
        None => {}
    }

    // gear-mesh slip check on the SAME sweep: pot must advance in fixed
    // proportion to motor rotation; a slipping/stripped tooth spikes or zeros
    // a segment. Advisory - one sweep can miss a localized slip zone.
    if let Some((pos, current)) = &sweep
        && let Some((phase, _)) = lut::cumulative_phase(current, fs, RIPPLE_PER_REV)
        && let Some(rep) = slip::slip_metrics(pos, &phase, 10, 1)
    {
        println!(
            "gear mesh: slip CoV {:.2} ({} core segments)",
            rep.slip_cov, rep.core_segments
        );
        if rep.flagged {
            eprintln!(
                "warning: possible worn/stripped gear teeth - gear-slip signature (slip CoV {:.2}, {} stuck segment(s)); a single sweep is not conclusive, re-run cal to confirm",
                rep.slip_cov, rep.stuck_count
            );
        }
    }

    // soft angle -> phys count via the linear phys map, clamped to the rails
    let soft_min_count =
        soft_to_count(soft_min_deg, phys_min, phys_max, pos_min_phys, pos_max_phys);
    let soft_max_count =
        soft_to_count(soft_max_deg, phys_min, phys_max, pos_min_phys, pos_max_phys);

    let kin = KinematicsResult {
        angle_min_cdeg,
        angle_max_cdeg,
        gear_ratio_centi,
    };
    let factors = units::derive(&sense, &kin, pos_min_phys, pos_max_phys, ke_vpc_q);
    print!("{}", units::render(&factors));

    println!("deg/count {dpc:.6}");
    println!(
        "phys angle {phys_min:.2}..{phys_max:.2} deg, soft angle {soft_min_deg:.2}..{soft_max_deg:.2} deg"
    );
    println!("soft counts {soft_min_count}..{soft_max_count}");

    if !args.yes && !confirm("write these values and MGMT SAVE", false)? {
        println!("declined before write");
        return Ok(());
    }

    // snapshot the calib + gain block before any write (rollback safety)
    snapshot::take_snapshot(&mut c, id, &out.0.join("snapshot.json"))?;

    write_reg(&mut c, id, config::POS_MIN_PHYS_COUNTS, pos_min_phys)?;
    write_reg(&mut c, id, config::POS_MAX_PHYS_COUNTS, pos_max_phys)?;
    write_reg(&mut c, id, config::POS_MIN_SOFT_COUNTS, soft_min_count)?;
    write_reg(&mut c, id, config::POS_MAX_SOFT_COUNTS, soft_max_count)?;
    write_reg(&mut c, id, config::DRIVE_POLARITY, drive_polarity as i32)?;
    write_reg(&mut c, id, calib::ANGLE_MIN_CDEG, angle_min_cdeg as i32)?;
    write_reg(&mut c, id, calib::ANGLE_MAX_CDEG, angle_max_cdeg as i32)?;
    write_reg(&mut c, id, calib::GEAR_RATIO_CENTI, gear_ratio_centi as i32)?;

    if let Some((l, ..)) = &lut {
        write_pot_lut(&mut c, id, l)?;
    }

    // SAVE needs torque off (protocol sec 9.4); park was already torque-off.
    write_reg(&mut c, id, control::TORQUE_ENABLE, 0)?;
    c.save(id).context("MGMT SAVE")?;
    println!("saved");
    Ok(())
}

/// A plausible pot reading: the ADC is 12-bit, so a valid pos is 0..=4095.
/// Anything larger is bit-corrupted.
fn pos_plausible(pos: u16) -> bool {
    pos <= 4095
}

/// Time-aligned (pos, current) sweep from the captured frames: the longest
/// seq-contiguous run. Splitting on seq gaps is the ONLY selection - it keeps
/// the sample spacing uniform, which the ripple autocorr assumes. Pos is NOT
/// required to be monotonic: the commutation ripple lives on the winding
/// current, so a slipping stripped gear (pos jittering while the motor spins)
/// still carries a clean signal, and lut::build sorts by pos regardless. A
/// flat stall segment self-rejects downstream (no ripple -> None). None when
/// nothing was captured.
fn build_sweep(tel: &[TelFrame]) -> Option<(Vec<u16>, Vec<f64>)> {
    // Drop value-domain corruption: pos > 4095 is impossible from a 12-bit ADC,
    // so a set bit above b11 is a bit error. Dropping the frame removes its seq,
    // which splits the contiguous run at that point and isolates the corruption.
    // Catches pos corruption only; a current(ripple)-domain bit-error is not
    // caught here - that needs a firmware frame CRC (deferred).
    let s: Vec<(u8, u16, f64)> = tel
        .iter()
        .filter_map(|f| {
            let pos = f.pos?;
            pos_plausible(pos).then_some((f.seq, pos, f.current? as f64))
        })
        .collect();
    let (lo, hi) = longest_contiguous_run(&s)?;
    // trim fused rail dwell here too: the slip check reads this run, and a
    // pinned-pot buzz segment reads as a stuck (slipping) gear segment.
    let run = trim_edge_dwell(&s[lo..hi]);
    Some((
        run.iter().map(|x| x.1).collect(),
        run.iter().map(|x| x.2).collect(),
    ))
}

/// A chunk shorter than this is too short to carry usable ripple and is almost
/// certainly a corruption fragment, so it is dropped from the stitch.
const MIN_CHUNK_SAMPLES: usize = 200;

/// A chunk whose pos span is below this is a STALL, not a sweep segment: the
/// open-loop constant-duty traverse can overshoot into a rail and buzz there
/// with pos frozen while commutation ripple keeps accruing revs. Such a chunk
/// has many samples but ~0 pos travel, so its revs/count is nonsense (spikes
/// the stitched anchor and dumps a bogus rail correction); drop it by span.
const MIN_CHUNK_SPAN: u16 = 50;

/// A moving run can also carry a rail dwell FUSED to its ends (too much travel
/// for the span gate above to catch): the traverse overshoots into an end-stop
/// and buzzes there seq-contiguously with the sweep, pot pinned while the motor
/// keeps turning. Those samples accrue ripple revs with ~zero pot travel and
/// poison the per-count density (bench: 270 ms of max-rail buzz fused to one
/// chunk deposited 71 garbage revs into the top bins - travel read 340 deg on a
/// ~225 deg servo, and the pinned-pot segment faked a gear-slip warning). Trim
/// each end at the sweep's first crossing out of / into the band this many
/// counts from that end's extreme pos.
const EDGE_DWELL_COUNTS: u16 = 8;

/// Trim rail dwell fused to a run's ends (see EDGE_DWELL_COUNTS): keep from the
/// first sample past the head-extreme band to the first sample reaching the
/// tail-extreme band. First-crossing on both ends is robust to buzz wiggle and
/// mid-run jitter; on a clean rail-to-rail sweep it costs only the outermost
/// EDGE_DWELL_COUNTS counts of each end. Runs narrower than the two bands are
/// returned unchanged.
fn trim_edge_dwell(run: &[(u8, u16, f64)]) -> &[(u8, u16, f64)] {
    let (mut lo, mut hi) = (run[0].1, run[0].1);
    for x in run {
        lo = lo.min(x.1);
        hi = hi.max(x.1);
    }
    if hi - lo <= 2 * EDGE_DWELL_COUNTS {
        return run;
    }
    let up = run[run.len() - 1].1 >= run[0].1;
    let past_head = |p: u16| {
        if up {
            p > lo + EDGE_DWELL_COUNTS
        } else {
            p < hi - EDGE_DWELL_COUNTS
        }
    };
    let at_tail = |p: u16| {
        if up {
            p >= hi - EDGE_DWELL_COUNTS
        } else {
            p <= lo + EDGE_DWELL_COUNTS
        }
    };
    let start = run.iter().position(|x| past_head(x.1)).unwrap_or(0);
    let end = run
        .iter()
        .position(|x| at_tail(x.1))
        .map(|i| i + 1)
        .unwrap_or(run.len());
    &run[start..end.max(start + 1)]
}

/// ALL clean sweep chunks (not just the longest): each maximal run of
/// seq-contiguous, in-range (pos<=4095) frames, as (pos, current). Chunks
/// shorter than MIN_CHUNK_SAMPLES are dropped as corruption fragments. The
/// stitch (lut::build_multi) reassembles them over the shared pos axis.
pub(super) fn build_sweep_chunks(tel: &[TelFrame]) -> Vec<(Vec<u16>, Vec<f64>)> {
    // Same filter as build_sweep: a dropped (corrupt/absent) frame removes its
    // seq, so the run splits there just as a seq gap would.
    let s: Vec<(u8, u16, f64)> = tel
        .iter()
        .filter_map(|f| {
            let pos = f.pos?;
            pos_plausible(pos).then_some((f.seq, pos, f.current? as f64))
        })
        .collect();
    let mut chunks = Vec::new();
    if s.is_empty() {
        return chunks;
    }
    let mut lo = 0usize;
    for i in 1..s.len() {
        if s[i].0 != s[i - 1].0.wrapping_add(1) {
            emit_chunk(&s[lo..i], &mut chunks);
            lo = i;
        }
    }
    emit_chunk(&s[lo..], &mut chunks);
    chunks
}

/// Push one maximal seq-contiguous run as a (pos, current) chunk, dropping it
/// when shorter than MIN_CHUNK_SAMPLES or when its pos span is below
/// MIN_CHUNK_SPAN (a rail stall, not a sweep segment). Rail dwell fused to the
/// ends is trimmed first; the length gate re-applies to the trimmed run.
fn emit_chunk(run: &[(u8, u16, f64)], chunks: &mut Vec<(Vec<u16>, Vec<f64>)>) {
    if run.len() < MIN_CHUNK_SAMPLES {
        return;
    }
    let (mut lo, mut hi) = (run[0].1, run[0].1);
    for x in run {
        lo = lo.min(x.1);
        hi = hi.max(x.1);
    }
    if hi - lo < MIN_CHUNK_SPAN {
        return;
    }
    let run = trim_edge_dwell(run);
    if run.len() >= MIN_CHUNK_SAMPLES {
        chunks.push((
            run.iter().map(|x| x.1).collect(),
            run.iter().map(|x| x.2).collect(),
        ));
    }
}

/// Longest half-open index range `[lo, hi)` of samples whose u8 seq increments
/// by exactly one each step (no dropped frames). None when nothing has >= 2
/// contiguous samples.
fn longest_contiguous_run(s: &[(u8, u16, f64)]) -> Option<(usize, usize)> {
    if s.len() < 2 {
        return None;
    }
    let (mut best_lo, mut best_hi) = (0usize, 0usize);
    let mut lo = 0usize;
    for i in 1..s.len() {
        if s[i].0 != s[i - 1].0.wrapping_add(1) {
            if i - lo > best_hi - best_lo {
                (best_lo, best_hi) = (lo, i);
            }
            lo = i;
        }
    }
    if s.len() - lo > best_hi - best_lo {
        (best_lo, best_hi) = (lo, s.len());
    }
    (best_hi - best_lo >= 2).then_some((best_lo, best_hi))
}

/// Write the PotLutBlock: raw_min/raw_max as scalars, then the 55 corr knots
/// as one 110-byte blob (lut_corr is a single Bytes field, so a bulk write
/// beats 55 round-trips and matches the on-servo layout exactly).
fn write_pot_lut(c: &mut Client<NusbPipe>, id: Id, lut: &PotLut) -> Result<()> {
    write_reg(c, id, calib::POT_LUT_RAW_MIN, lut.raw_min as i32)?;
    write_reg(c, id, calib::POT_LUT_RAW_MAX, lut.raw_max as i32)?;
    let mut bytes = [0u8; 110];
    for (i, &k) in lut.corr.iter().enumerate() {
        bytes[2 * i..2 * i + 2].copy_from_slice(&k.to_le_bytes());
    }
    c.write(id, calib::POT_LUT_CORR.addr, &bytes)
        .context("write pot lut corr")?;
    Ok(())
}

fn read_sense(c: &mut Client<NusbPipe>, id: Id) -> Result<SenseParams> {
    Ok(SenseParams {
        shunt_r_mohm: read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        vdd_mv: read_u16(c, id, calib::VDD_MV)?,
        tick_hz: read_u16(c, id, calib::TICK_HZ)?,
    })
}

fn run_endstop(c: &mut Client<NusbPipe>, id: Id, out: &OutDir) -> Result<EndstopResult> {
    println!("[endstop] seeking both rails (pos guard off)");
    // pos guard off: driving into the physical ends IS the method
    let params = RigParams::default().without_pos_guard();
    let mut log = SnapshotLog::create(out, "endstop_snapshots.csv")?;
    let mut exp = Guarded::new(Endstop::new(EndstopCfg::default(), &params), params);
    let ran = Pump {
        client: c,
        id,
        tel_port: None,
        tel_mask: 0,
        log: Some(&mut log),
        tel_raw_path: None,
    }
    .run(&mut exp, |_| {});
    // park safe whether the run finished, errored, or was ctrl-c'd
    let _ = write_reg(c, id, control::GOAL_DUTY, 0);
    let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    ran?;
    if let Some(reason) = exp.abort() {
        bail!("endstop aborted by the safety envelope: {reason:?}");
    }
    exp.into_inner()
        .result()
        .context("endstop did not reach both rails - no writes")
}

/// One dedicated constant-duty ripple capture, TEL captured throughout. The
/// motion stays strictly between count-insets from both rails, so the clone's
/// end-jam is never reached: positioning + a speed probe run first with TEL off
/// (polling is free there), then the capture is TIME-bounded (no polls), sized
/// from the probed speed so the pump drains TEL as a single long seq-contiguous
/// run for build_sweep. The ~3% inset clears the rail jam yet leaves ~94% of
/// travel captured (still passes the LUT coverage gate).
fn run_sweep(
    c: &mut Client<NusbPipe>,
    id: Id,
    port: &str,
    pos_min_phys: i32,
    pos_max_phys: i32,
    drive_polarity: bool,
    out: &OutDir,
) -> Result<Vec<TelFrame>> {
    const DUTY: i16 = 8520;
    let span = (pos_max_phys - pos_min_phys) as f64;
    let inset = (span * 0.03).max(80.0) as i32;
    let mut start = pos_min_phys + inset;
    let mut end = pos_max_phys - inset;
    if end <= start {
        // travel too short for an inset capture: fall back to the rails rather
        // than crossing over (capture_ms still clamps the duration).
        start = pos_min_phys;
        end = pos_max_phys;
    }
    // capture direction = increasing pos; polarity maps duty sign to direction
    let sweep_sign = if drive_polarity { 1 } else { -1 };

    // speed probe, TEL off: drive AWAY from the nearer rail (toward the
    // interior) for a short window so the probe itself never reaches a stop.
    let mid = (pos_min_phys + pos_max_phys) / 2;
    let here = read_snapshot(c, id)?.pos as i32;
    let probe_sign = if here < mid { sweep_sign } else { -sweep_sign };
    let speed = probe_speed(c, id, DUTY, probe_sign, 150)?;

    // position to the capture start rail-inset (still TEL off, polling free)
    drive_to(c, id, start, drive_polarity, DUTY)?;

    let ms = capture_ms(end as f64 - start as f64, speed, 0.95);
    println!("[sweep] capture {ms} ms over counts {start}..{end} (speed {speed:.0} cps)");

    let mut exp = Sweep::new(
        SweepCfg {
            duty_q15: DUTY,
            capture_ms: ms,
        },
        sweep_sign,
    );
    let mut tel: Vec<TelFrame> = Vec::new();
    // 0x1B = pos|current|duty|vdiff (same TEL mask as ident inertia)
    let tel_mask: u16 = 0x1B;
    let mut wrap = TelWrap {
        inner: &mut exp,
        phase: TelInject::MaskOn,
        mask: tel_mask,
    };
    let ran = Pump {
        client: c,
        id,
        tel_port: Some(port.to_string()),
        tel_mask,
        log: None,
        tel_raw_path: Some(out.0.join("tel-raw.bin")),
    }
    .run(&mut wrap, |frames| tel.extend_from_slice(frames));
    // park safe whether the run finished, errored, or was ctrl-c'd
    let _ = write_reg(c, id, control::GOAL_DUTY, 0);
    let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    ran?;
    println!(
        "[sweep] raw bytes -> {}",
        out.0.join("tel-raw.bin").display()
    );
    Ok(tel)
}

/// Injects the TEL enable/mask writes around an inner experiment so the pump
/// opens the side-channel for a run that would not enable TEL on its own
/// (Endstop). Prepends mask+enable, delegates the body, appends enable=0 +
/// mask=0 when the inner run reports Done.
enum TelInject {
    MaskOn,
    EnaOn,
    Body,
    EnaOff,
    MaskOff,
    Done,
}

struct TelWrap<'a, E> {
    inner: &'a mut E,
    phase: TelInject,
    mask: u16,
}

impl<E: Experiment> Experiment for TelWrap<'_, E> {
    fn step(&mut self, obs: Option<&TelemetrySnapshot>) -> Cmd {
        match self.phase {
            TelInject::MaskOn => {
                self.phase = TelInject::EnaOn;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: self.mask as i32,
                }
            }
            TelInject::EnaOn => {
                self.phase = TelInject::Body;
                Cmd::Write {
                    reg: control::TEL_ENABLE,
                    value: 1,
                }
            }
            TelInject::Body => {
                let cmd = self.inner.step(obs);
                if matches!(cmd, Cmd::Done) {
                    self.phase = TelInject::EnaOff;
                    Cmd::Write {
                        reg: control::TEL_ENABLE,
                        value: 0,
                    }
                } else {
                    cmd
                }
            }
            TelInject::EnaOff => {
                self.phase = TelInject::MaskOff;
                Cmd::Write {
                    reg: control::TEL_MASK,
                    value: 0,
                }
            }
            TelInject::MaskOff => {
                self.phase = TelInject::Done;
                Cmd::Done
            }
            TelInject::Done => Cmd::Done,
        }
    }
}

/// Drive to the rail midpoint so the servo does not rest on a hard stop.
/// Best-effort: uses the measured polarity to pick the duty sign toward the
/// midpoint, parks torque-off on arrival, and gives up quietly after the loop.
fn recenter(
    c: &mut Client<NusbPipe>,
    id: Id,
    pos_min: i32,
    pos_max: i32,
    drive_polarity: bool,
) -> Result<()> {
    const DUTY: i32 = 9000;
    let mid = (pos_min + pos_max) / 2;
    let margin = ((pos_max - pos_min) / 10).max(1);
    let lo = (mid - margin).clamp(0, u16::MAX as i32) as u16;
    let hi = (mid + margin).clamp(0, u16::MAX as i32) as u16;
    let park = |c: &mut Client<NusbPipe>| {
        let _ = write_reg(c, id, control::GOAL_DUTY, 0);
        let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    };
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    for _ in 0..200 {
        if pump::STOP.load(Ordering::SeqCst) {
            park(c);
            bail!("interrupted");
        }
        let pos = read_snapshot(c, id)?.pos;
        if (lo..=hi).contains(&pos) {
            park(c);
            return Ok(());
        }
        // duty sign toward the midpoint: polarity maps count direction to sign
        let toward_higher = (pos as i32) < mid;
        let duty = if toward_higher == drive_polarity {
            DUTY
        } else {
            -DUTY
        };
        write_reg(c, id, control::GOAL_DUTY, duty)?;
        std::thread::sleep(Duration::from_millis(25));
    }
    park(c);
    println!("[recenter] did not reach mid-travel (gear slip?), left parked");
    Ok(())
}

/// Measure traverse speed (counts/s) at the capture duty over a short window,
/// TEL off so pos polling is free. Drives one fixed duty for `ms` and divides
/// the pos delta by the elapsed time. Leaves duty 0 + torque ON (the caller
/// positions next); ctrl-c parks duty 0 + torque off and bails.
fn probe_speed(c: &mut Client<NusbPipe>, id: Id, duty_q15: i16, sign: i8, ms: u32) -> Result<f64> {
    let park = |c: &mut Client<NusbPipe>| {
        let _ = write_reg(c, id, control::GOAL_DUTY, 0);
        let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    };
    if pump::STOP.load(Ordering::SeqCst) {
        park(c);
        bail!("interrupted");
    }
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    let p0 = read_snapshot(c, id)?.pos as f64;
    write_reg(c, id, control::GOAL_DUTY, sign as i32 * duty_q15 as i32)?;
    std::thread::sleep(Duration::from_millis(ms as u64));
    if pump::STOP.load(Ordering::SeqCst) {
        park(c);
        bail!("interrupted");
    }
    let p1 = read_snapshot(c, id)?.pos as f64;
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    Ok((p1 - p0).abs() / (ms as f64 / 1000.0))
}

/// Closed-loop drive toward a target count, TEL off. Polls pos, picks the duty
/// sign toward target via the measured polarity, and stops within a small band.
/// Leaves duty 0 + torque ON (holds position for the capture that follows);
/// ctrl-c parks duty 0 + torque off and bails.
fn drive_to(
    c: &mut Client<NusbPipe>,
    id: Id,
    target: i32,
    drive_polarity: bool,
    duty_q15: i16,
) -> Result<()> {
    // fixed band, well inside the >=80 count rail inset so we never settle on
    // (or overshoot into) a stop.
    const BAND: i32 = 40;
    let duty = duty_q15 as i32;
    let park = |c: &mut Client<NusbPipe>| {
        let _ = write_reg(c, id, control::GOAL_DUTY, 0);
        let _ = write_reg(c, id, control::TORQUE_ENABLE, 0);
    };
    write_reg(c, id, control::MODE, 0)?;
    write_reg(c, id, control::TORQUE_ENABLE, 1)?;
    for _ in 0..200 {
        if pump::STOP.load(Ordering::SeqCst) {
            park(c);
            bail!("interrupted");
        }
        let pos = read_snapshot(c, id)?.pos as i32;
        if (pos - target).abs() <= BAND {
            write_reg(c, id, control::GOAL_DUTY, 0)?;
            return Ok(());
        }
        // duty sign toward target: polarity maps count direction to sign
        let toward_higher = pos < target;
        let d = if toward_higher == drive_polarity {
            duty
        } else {
            -duty
        };
        write_reg(c, id, control::GOAL_DUTY, d)?;
        std::thread::sleep(Duration::from_millis(25));
    }
    write_reg(c, id, control::GOAL_DUTY, 0)?;
    println!("[sweep] drive_to did not reach start inset (gear slip?), capturing from here");
    Ok(())
}

/// Capture duration (ms) to traverse `span_counts` at `speed_cps`, scaled by a
/// `safety` factor and clamped to [200, 4000] ms so a bad speed reading can
/// never drive for minutes. A non-positive or non-finite speed returns the
/// 4000 ms max (defensive - the clamp still bounds it).
fn capture_ms(span_counts: f64, speed_cps: f64, safety: f64) -> u32 {
    if !speed_cps.is_finite() || speed_cps <= 0.0 {
        return 4000;
    }
    let ms = span_counts / speed_cps * 1000.0 * safety;
    if !ms.is_finite() {
        return 4000;
    }
    ms.clamp(200.0, 4000.0).round() as u32
}

/// Phys angle at each rail: from flags under `--yes` (both required), else
/// prompted with the flags or 0..DEFAULT_TRAVEL_DEG as defaults.
fn phys_angles(args: &Args) -> Result<(f64, f64)> {
    if args.yes {
        let min = args
            .phys_angle_min
            .context("--yes needs --phys-angle-min")?;
        let max = args
            .phys_angle_max
            .context("--yes needs --phys-angle-max")?;
        return Ok((min, max));
    }
    let min = input_f64(
        "phys angle at min rail (deg)",
        args.phys_angle_min.unwrap_or(0.0),
    )?;
    let max = input_f64(
        "phys angle at max rail (deg)",
        args.phys_angle_max.unwrap_or(DEFAULT_TRAVEL_DEG),
    )?;
    Ok((min, max))
}

/// Soft (working-range) angle: flags under `--yes` (defaulting to phys), else
/// prompted with the flags or the phys angles as defaults.
fn soft_angles(args: &Args, phys_min: f64, phys_max: f64) -> Result<(f64, f64)> {
    // default = DEFAULT_SOFT_TRAVEL_DEG centered in the phys range: soft
    // limits set to the rails hand every mode's endstop zero coast margin
    let span = (phys_max - phys_min).min(DEFAULT_SOFT_TRAVEL_DEG);
    let mid = (phys_min + phys_max) / 2.0;
    let (dmin, dmax) = (mid - span / 2.0, mid + span / 2.0);
    if args.yes {
        return Ok((
            args.soft_angle_min.unwrap_or(dmin),
            args.soft_angle_max.unwrap_or(dmax),
        ));
    }
    let min = input_f64("soft angle min (deg)", args.soft_angle_min.unwrap_or(dmin))?;
    let max = input_f64("soft angle max (deg)", args.soft_angle_max.unwrap_or(dmax))?;
    Ok((min, max))
}

/// Resolve the headless (`--yes`) anchor to (angle_at_min, angle_at_max,
/// gear_centi). GEAR is the preferred anchor: given a `--gear-ratio` flag and a
/// ripple-derived `motor_revs_full`, the physical travel is derived from the
/// gear (pure geometry, protocol-free) and the counted gear stored as-is;
/// angle_at_min comes from `--phys-angle-min` (0 default), angle_at_max is that
/// plus the derived travel. Falling back: explicit `--phys-angle-min/max` flags
/// anchor on travel and the gear is derived from ripple (the inverse) - though a
/// counted `--gear-ratio` flag, when present, still wins for storage (matching
/// the pre-inverse precedence). Neither anchor -> the 0/0/0 unset sentinels.
fn resolve_anchor(
    motor_revs_full: Option<f64>,
    gear_flag: Option<f64>,
    phys_min_flag: Option<f64>,
    phys_max_flag: Option<f64>,
) -> (f64, f64, u16) {
    if let (Some(g), Some(m)) = (gear_flag, motor_revs_full)
        && g > 0.0
    {
        let travel = kinematics::travel_deg_from_gear(m, g);
        let angle_min = phys_min_flag.unwrap_or(0.0);
        return (angle_min, angle_min + travel, ratio_to_centi(Some(g)));
    }
    if let (Some(lo), Some(hi)) = (phys_min_flag, phys_max_flag) {
        let ripple_centi = motor_revs_full
            .map(|m| ratio_to_centi(travel_to_gear(m, hi - lo)))
            .unwrap_or(0);
        let gear_centi = match gear_flag {
            Some(g) if g > 0.0 => ratio_to_centi(Some(g)),
            _ => ripple_centi,
        };
        return (lo, hi, gear_centi);
    }
    (0.0, 0.0, 0)
}

/// Print the headless anchor outcome. Gear anchor -> the derived travel (and,
/// when an explicit travel is also given, a divergence warning; gear still wins
/// for storage). Travel anchor with a ripple-derived gear -> the ripple info
/// line. Divergence flags a wrong travel angle, gear slip, or wrong ripple-per-rev.
fn report_yes_anchor(
    args: &Args,
    mrf: Option<f64>,
    coverage: Option<f64>,
    angle_min: f64,
    angle_max: f64,
    gear_centi: u16,
) {
    let gear_flag = args.gear_ratio.filter(|&g| g > 0.0);
    if let (Some(_), Some(m)) = (gear_flag, mrf) {
        println!(
            "derived travel {:.1} deg from gear {:.2}",
            angle_max - angle_min,
            gear_centi as f64 / 100.0,
        );
        if let (Some(lo), Some(hi)) = (args.phys_angle_min, args.phys_angle_max) {
            let ripple_centi = ratio_to_centi(travel_to_gear(m, hi - lo));
            if gear_divergent(gear_centi, ripple_centi, 0.1) {
                eprintln!(
                    "warning: entered gear ratio {:.2} diverges from ripple-measured {:.2} by >10% - can indicate a wrong travel angle, gear slip, or wrong ripple-per-rev",
                    gear_centi as f64 / 100.0,
                    ripple_centi as f64 / 100.0,
                );
            }
        }
    } else if gear_flag.is_none() && gear_centi != 0 {
        print_ripple_gear(gear_centi, coverage.unwrap_or(0.0));
    }
}

/// Interactive anchor resolution, nudged toward the gear. Prompts the gear ratio
/// first: a positive gear with a ripple-derived `motor_revs_full` derives travel
/// (gear anchor); entering 0, or having no ripple, falls back to the travel-angle
/// prompts and derives the gear from ripple (the inverse). With no ripple at all,
/// an entered gear still stands so a known ratio can be stored without a sweep.
fn resolve_anchor_interactive(
    args: &Args,
    mrf: Option<f64>,
    coverage: Option<f64>,
) -> Result<(f64, f64, u16)> {
    let gear = input_f64(
        "gear ratio (motor rev per output rev) - most accurate if you counted the teeth; 0 to enter travel angle instead",
        args.gear_ratio.unwrap_or(1.0),
    )?;
    if gear > 0.0
        && let Some(m) = mrf
    {
        let travel = kinematics::travel_deg_from_gear(m, gear);
        let angle_min = input_f64(
            "phys angle at min rail (deg)",
            args.phys_angle_min.unwrap_or(0.0),
        )?;
        println!("derived travel {travel:.1} deg from gear {gear:.2}");
        return Ok((angle_min, angle_min + travel, ratio_to_centi(Some(gear))));
    }
    let (lo, hi) = phys_angles(args)?;
    let gear_centi = match mrf {
        Some(m) => {
            let centi = ratio_to_centi(travel_to_gear(m, hi - lo));
            if centi != 0 {
                print_ripple_gear(centi, coverage.unwrap_or(0.0));
            }
            centi
        }
        None => ratio_to_centi((gear > 0.0).then_some(gear)),
    };
    Ok((lo, hi, gear_centi))
}

/// Ripple gear ratio (motor rev per output rev) implied by the full-traverse
/// motor revs and an operator travel angle - the inverse of
/// kinematics::travel_deg_from_gear. None on a zero/non-finite travel, so
/// ratio_to_centi then stores the 0 sentinel.
fn travel_to_gear(motor_revs_full: f64, travel_deg: f64) -> Option<f64> {
    (travel_deg.is_finite() && travel_deg > 0.0).then_some(motor_revs_full * 360.0 / travel_deg)
}

/// The ripple-derived gear info line (the historical "measured gear ratio ..."
/// print), shown when the travel anchor is used.
fn print_ripple_gear(gear_centi: u16, coverage: f64) {
    let conf = if coverage < RIPPLE_CONF_MIN {
        " (low confidence)"
    } else {
        ""
    };
    println!(
        "measured gear ratio {:.2}, ripple-derived ({:.0}% coverage){conf}",
        gear_centi as f64 / 100.0,
        coverage * 100.0,
    );
}

/// True when the ripple-`measured_centi` gear ratio diverges from the operator-
/// entered `entered_centi` by more than `frac` (relative). entered_centi==0
/// (flag unset or invalid) can never diverge.
fn gear_divergent(entered_centi: u16, measured_centi: u16, frac: f64) -> bool {
    if entered_centi == 0 {
        return false;
    }
    ((measured_centi as f64 / entered_centi as f64) - 1.0).abs() > frac
}

fn ratio_to_centi(ratio: Option<f64>) -> u16 {
    match ratio {
        Some(r) if r.is_finite() && r > 0.0 => {
            (r * 100.0).round().clamp(0.0, u16::MAX as f64) as u16
        }
        _ => 0,
    }
}

fn soft_to_count(angle: f64, phys_min: f64, phys_max: f64, pos_min: i32, pos_max: i32) -> i32 {
    let denom = phys_max - phys_min;
    let frac = if denom == 0.0 {
        0.0
    } else {
        (angle - phys_min) / denom
    };
    let count = pos_min as f64 + frac * (pos_max - pos_min) as f64;
    let (lo, hi) = (pos_min.min(pos_max), pos_min.max(pos_max));
    (count.round() as i32).clamp(lo, hi)
}

fn confirm(prompt: &str, default: bool) -> Result<bool> {
    Ok(Confirm::new()
        .with_prompt(prompt)
        .default(default)
        .interact()?)
}

fn input_f64(prompt: &str, default: f64) -> Result<f64> {
    Ok(Input::<f64>::new()
        .with_prompt(prompt)
        .default(default)
        .interact_text()?)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn soft_maps_and_clamps_within_rails() {
        // 0..180 deg over counts 100..4000
        assert_eq!(soft_to_count(0.0, 0.0, 180.0, 100, 4000), 100);
        assert_eq!(soft_to_count(180.0, 0.0, 180.0, 100, 4000), 4000);
        assert_eq!(soft_to_count(90.0, 0.0, 180.0, 100, 4000), 2050);
        // tighter-than-phys stays in band; out-of-range clamps to a rail
        assert_eq!(soft_to_count(-10.0, 0.0, 180.0, 100, 4000), 100);
        assert_eq!(soft_to_count(200.0, 0.0, 180.0, 100, 4000), 4000);
    }

    #[test]
    fn soft_degenerate_phys_span_is_min_rail() {
        assert_eq!(soft_to_count(50.0, 90.0, 90.0, 100, 4000), 100);
    }

    /// Contiguous-seq samples from a pos list (current = pos for simplicity).
    fn seqd(pos: &[u16]) -> Vec<(u8, u16, f64)> {
        pos.iter()
            .enumerate()
            .map(|(i, &p)| (i as u8, p, p as f64))
            .collect()
    }

    #[test]
    fn contiguous_run_splits_on_seq_gap_and_picks_longest() {
        // 30 samples, seq gap at index 20: the longer half [0,20) wins, and
        // pos need not be monotonic (slip is fine - current carries ripple).
        let mut s = seqd(&(0..30).map(|k| 1000 + (k % 3) * 10).collect::<Vec<_>>());
        for e in s.iter_mut().skip(20) {
            e.0 = e.0.wrapping_add(7); // punch a seq discontinuity at 20
        }
        assert_eq!(longest_contiguous_run(&s), Some((0, 20)));
    }

    #[test]
    fn contiguous_run_seq_wraps_cleanly() {
        // u8 seq wrapping 255->0 mid-run stays contiguous.
        let mut s = seqd(&[10u16; 6]);
        for (i, e) in s.iter_mut().enumerate() {
            e.0 = (253u8).wrapping_add(i as u8); // 253,254,255,0,1,2
        }
        assert_eq!(longest_contiguous_run(&s), Some((0, 6)));
    }

    #[test]
    fn contiguous_run_none_when_too_short() {
        assert!(longest_contiguous_run(&[]).is_none());
        assert!(longest_contiguous_run(&seqd(&[7])).is_none());
    }

    #[test]
    fn build_sweep_none_without_frames() {
        assert!(build_sweep(&[]).is_none());
    }

    #[test]
    fn pos_plausible_rejects_above_12bit() {
        assert!(pos_plausible(0));
        assert!(pos_plausible(4095));
        assert!(!pos_plausible(4096));
        assert!(!pos_plausible(9999));
    }

    #[test]
    fn build_sweep_drops_out_of_range_pos() {
        // seqs 0..10, all pos in-range except seq 5 which is bit-corrupt (>4095).
        // Dropping it removes its seq, so the contiguous run splits there: the
        // longer half is seqs 0..=4 (5 samples) and the corrupt pos is excluded.
        let frames: Vec<TelFrame> = (0..10u8)
            .map(|k| TelFrame {
                seq: k,
                pos: Some(if k == 5 { 9999 } else { 1000 + k as u16 }),
                current: Some(50),
                ..Default::default()
            })
            .collect();
        let (pos, current) = build_sweep(&frames).expect("a run survives");
        assert!(
            pos.iter().all(|&p| p <= 4095),
            "corrupt pos leaked: {pos:?}"
        );
        assert!(!pos.contains(&9999));
        assert_eq!(pos, vec![1000, 1001, 1002, 1003, 1004]);
        assert_eq!(current.len(), pos.len());
    }

    #[test]
    fn build_sweep_chunks_keeps_big_runs_drops_fragments_and_stalls() {
        // seq = index (u8, wraps cleanly). Moving runs (pos sweeps a real span)
        // split by out-of-range pos, plus a trailing short fragment, a long
        // STALL run (pos frozen at a rail), and a moving run with a rail buzz
        // FUSED to its tail. Kept: the moving runs, edge-trimmed by
        // EDGE_DWELL_COUNTS; the fused buzz tail is trimmed off entirely.
        let mut frames: Vec<TelFrame> = Vec::new();
        let mut push = |base: u16, count: u32, moving: bool| {
            for j in 0..count {
                let seq = frames.len() as u8;
                let pos = if moving { base + j as u16 } else { base };
                frames.push(TelFrame {
                    seq,
                    pos: Some(pos),
                    current: Some(50),
                    ..Default::default()
                });
            }
        };
        push(1000, 250, true); // run A: span 249, kept (trimmed)
        push(9999, 1, true); // split (out-of-range)
        push(2000, 300, true); // run B: span 299, kept (trimmed)
        push(9999, 1, true); // split
        push(3000, 50, true); // fragment: < MIN_CHUNK_SAMPLES, dropped
        push(9999, 1, true); // split
        push(4095, 250, false); // stall: >= samples but span 0, dropped
        push(9999, 1, true); // split
        push(3600, 300, true); // run C sweeps 3600..3899 ...
        push(3899, 400, false); // ... then buzzes at the rail seq-contiguously

        let chunks = build_sweep_chunks(&frames);
        assert_eq!(chunks.len(), 3, "the three moving runs survive");
        // each end trimmed at the first crossing out of the extreme band:
        // head loses EDGE_DWELL_COUNTS+1 samples, tail keeps through the first
        // sample inside the band (1 count/sample here)
        let trimmed = |n: usize| n - (2 * EDGE_DWELL_COUNTS as usize + 1);
        assert_eq!(chunks[0].0.len(), trimmed(250));
        assert_eq!(chunks[1].0.len(), trimmed(300));
        assert_eq!(chunks[0].1.len(), trimmed(250));
        // the stall run (span 0) is gone despite having >= MIN_CHUNK_SAMPLES
        assert!(chunks.iter().all(|(p, _)| p.iter().max() != Some(&4095)));
        // run C: the 400-sample fused buzz is trimmed off with the tail band
        assert_eq!(chunks[2].0.len(), trimmed(300));
        assert!(chunks[2].0.iter().max() < Some(&3899));
    }

    #[test]
    fn build_sweep_chunks_empty_without_frames() {
        assert!(build_sweep_chunks(&[]).is_empty());
    }

    #[test]
    fn gear_divergent_relative_threshold_and_zero_safe() {
        // 234.73 counted vs a >10% off ripple value -> divergent
        assert!(gear_divergent(23473, 30000, 0.1));
        // within 10% -> not divergent
        assert!(!gear_divergent(23473, 24000, 0.1));
        assert!(!gear_divergent(10000, 10500, 0.1));
        assert!(gear_divergent(10000, 11500, 0.1));
        // entered unset -> never diverges (no divide by zero)
        assert!(!gear_divergent(0, 30000, 0.1));
    }

    #[test]
    fn resolve_anchor_gear_flag_derives_travel() {
        // gear 150 + motor_revs_full 79.1667 -> travel ~190 deg; endpoints 0..190
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), Some(150.0), None, None);
        assert_eq!(lo, 0.0);
        assert!((hi - 190.0).abs() < 0.01, "hi {hi}");
        assert_eq!(gear, 15000);
    }

    #[test]
    fn resolve_anchor_gear_flag_honors_min_offset() {
        // angle_at_min from --phys-angle-min, angle_at_max = min + derived travel
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), Some(150.0), Some(-95.0), None);
        assert_eq!(lo, -95.0);
        assert!((hi - 95.0).abs() < 0.01, "hi {hi}");
        assert_eq!(gear, 15000);
    }

    #[test]
    fn resolve_anchor_travel_flags_derive_gear() {
        // travel 0..190 deg + motor_revs_full 79.1667 -> ripple gear ~150
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), None, Some(0.0), Some(190.0));
        assert_eq!((lo, hi), (0.0, 190.0));
        assert!((gear as i32 - 15000).abs() <= 2, "gear {gear}");
    }

    #[test]
    fn resolve_anchor_gear_flag_wins_when_both_given() {
        // both a gear flag and travel flags: gear anchor wins, travel derived
        // from the gear (max ignores --phys-angle-max as an endpoint)
        let (lo, hi, gear) = resolve_anchor(Some(79.166_667), Some(150.0), Some(0.0), Some(999.0));
        assert_eq!(lo, 0.0);
        assert!((hi - 190.0).abs() < 0.01, "hi {hi}");
        assert_eq!(gear, 15000);
    }

    #[test]
    fn resolve_anchor_flag_stored_without_ripple() {
        // travel flags but no ripple: endpoints kept, counted gear flag stored
        assert_eq!(
            resolve_anchor(None, Some(234.73), Some(0.0), Some(190.0)),
            (0.0, 190.0, 23473)
        );
    }

    #[test]
    fn resolve_anchor_unset_sentinels() {
        // neither anchor -> all-zero sentinel
        assert_eq!(resolve_anchor(Some(79.0), None, None, None), (0.0, 0.0, 0));
        // travel flags, no ripple, no gear flag -> gear unset
        assert_eq!(
            resolve_anchor(None, None, Some(0.0), Some(190.0)),
            (0.0, 190.0, 0)
        );
        // gear flag but no ripple and no travel -> nothing to anchor on
        assert_eq!(resolve_anchor(None, Some(150.0), None, None), (0.0, 0.0, 0));
    }

    #[test]
    fn capture_ms_scales_clamps_and_defends() {
        // 3400 counts at 3400 cps * 0.95 safety = 950 ms
        assert_eq!(capture_ms(3400.0, 3400.0, 0.95), 950);
        // fast motor -> below the 200 ms floor -> clamped up
        assert_eq!(capture_ms(100.0, 20000.0, 0.95), 200);
        // slow motor -> above the 4000 ms ceiling -> clamped down
        assert_eq!(capture_ms(4000.0, 200.0, 0.95), 4000);
        // bad speed readings -> defensive 4000 ms max
        assert_eq!(capture_ms(3400.0, 0.0, 0.95), 4000);
        assert_eq!(capture_ms(3400.0, -50.0, 0.95), 4000);
        assert_eq!(capture_ms(3400.0, f64::NAN, 0.95), 4000);
        assert_eq!(capture_ms(3400.0, f64::INFINITY, 0.95), 4000);
    }

    #[test]
    fn ratio_centi_encoding_and_sentinel() {
        assert_eq!(ratio_to_centi(Some(150.0)), 15000);
        assert_eq!(ratio_to_centi(Some(1.5)), 150);
        assert_eq!(ratio_to_centi(None), 0);
        assert_eq!(ratio_to_centi(Some(0.0)), 0);
        assert_eq!(ratio_to_centi(Some(-3.0)), 0);
        assert_eq!(ratio_to_centi(Some(f64::NAN)), 0);
        assert_eq!(ratio_to_centi(Some(1e9)), u16::MAX);
    }
}
