//! `osc cal-replay` -- offline replay + corruption diagnostic for a saved
//! `tel-raw.bin`. `osc cal --tel-port` streams the ripple sweep and writes the
//! raw side-channel bytes; this command deframes those bytes without a servo,
//! classifies which corruption mode (if any) the stream carries, and re-runs
//! the same pot-LUT / motor-rev pipeline so the analysis can be iterated
//! offline. It NEVER connects to the servo (no baud/id).
//!
//! Two corruption modes are distinguished: whole-frame CDC drops (the values
//! that survive are clean, seq continuity has holes) versus in-frame bit-errors
//! (frame framing holds but field contents are corrupted - pos out of the
//! 12-bit range, or a within-range pos jump no motor could make in one sample).

use anyhow::{Context, Result, bail};
use osc_ident::frame::{TelDeframer, TelFrame};
use osc_ident::lut::{self, build_multi, stitched_motor_revs};

use super::{RIPPLE_PER_REV, build_sweep, build_sweep_chunks, pos_plausible};

/// A pos delta larger than this across two truly-consecutive samples is
/// physically impossible at the sweep sample rate (the capture traverses at a
/// few thousand counts/s, i.e. well under one count per 20 kHz sample), so it
/// marks a within-range bit-flip rather than real motion.
const MAX_STEP_COUNTS: u32 = 200;

/// `osc cal-replay` args. Offline: only the file and decode parameters, no bus.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// The tel-raw.bin captured during `osc cal --tel-port`.
    path: std::path::PathBuf,
    /// TEL mask used at capture (hex ok, e.g. 0x1B); must match the capture.
    #[arg(long, default_value = "0x1B")]
    mask: String,
    /// Ripple sample rate (Hz); the cal capture uses the servo tick_hz.
    #[arg(long, default_value_t = 20000.0)]
    fs: f64,
    /// Pot count at the min rail; default = min plausible pos observed.
    #[arg(long)]
    raw_min: Option<u16>,
    /// Pot count at the max rail; default = max plausible pos observed.
    #[arg(long)]
    raw_max: Option<u16>,
}

/// Parse a mask given as decimal-free hex, with or without a `0x` prefix.
fn parse_mask(s: &str) -> Result<u16> {
    let t = s.trim();
    let t = t
        .strip_prefix("0x")
        .or_else(|| t.strip_prefix("0X"))
        .unwrap_or(t);
    u16::from_str_radix(t, 16).with_context(|| format!("invalid --mask hex {s:?}"))
}

/// Corruption tallies from a decoded frame list.
struct Classification {
    /// Frames with pos > 4095 (12-bit ADC overflow = definite bit corruption).
    out_of_range: usize,
    /// Truly-consecutive (seq delta 1) within-range pos pairs whose |delta|
    /// exceeds MAX_STEP_COUNTS - a within-range bit-flip.
    implausible_jumps: usize,
    /// Largest |pos delta| seen across the same within-range consecutive pairs.
    max_consec_delta: u32,
}

fn classify(frames: &[TelFrame]) -> Classification {
    let out_of_range = frames
        .iter()
        .filter(|f| matches!(f.pos, Some(p) if !pos_plausible(p)))
        .count();
    let mut implausible_jumps = 0usize;
    let mut max_consec_delta = 0u32;
    for w in frames.windows(2) {
        let (a, b) = (&w[0], &w[1]);
        // only truly consecutive frames (no dropped seq in between)
        if b.seq.wrapping_sub(a.seq) != 1 {
            continue;
        }
        // both ends within range: an out-of-range end is already counted above,
        // and this keeps the jump metric a distinct within-range signal.
        if let (Some(pa), Some(pb)) = (a.pos, b.pos)
            && pos_plausible(pa)
            && pos_plausible(pb)
        {
            let delta = (pa as i32 - pb as i32).unsigned_abs();
            if delta > MAX_STEP_COUNTS {
                implausible_jumps += 1;
            }
            max_consec_delta = max_consec_delta.max(delta);
        }
    }
    Classification {
        out_of_range,
        implausible_jumps,
        max_consec_delta,
    }
}

pub fn run(args: &Args) -> Result<()> {
    let mask = parse_mask(&args.mask)?;
    let bytes =
        std::fs::read(&args.path).with_context(|| format!("read {}", args.path.display()))?;
    println!("file: {} ({} bytes)", args.path.display(), bytes.len());

    let mut d = match TelDeframer::new(mask) {
        Some(d) => d,
        None => bail!("invalid TEL mask {mask:#06x}: reserved bits set or empty"),
    };
    let mut frames: Vec<TelFrame> = Vec::new();
    d.push(&bytes, &mut frames);
    let stats = d.stats();
    let cls = classify(&frames);

    // resolve rails: explicit flags win; otherwise fall back to the observed
    // plausible-pos bounds and say so.
    let observed_used = args.raw_min.is_none() || args.raw_max.is_none();
    let obs_min = frames
        .iter()
        .filter_map(|f| f.pos)
        .filter(|&p| pos_plausible(p))
        .min();
    let obs_max = frames
        .iter()
        .filter_map(|f| f.pos)
        .filter(|&p| pos_plausible(p))
        .max();
    let raw_min = args.raw_min.or(obs_min).unwrap_or(0);
    let raw_max = args.raw_max.or(obs_max).unwrap_or(0);

    let sweep = build_sweep(&frames);
    let chunks = build_sweep_chunks(&frames);
    let chunk_samples: usize = chunks.iter().map(|(pos, _)| pos.len()).sum();
    let run_len = sweep.as_ref().map(|(pos, _)| pos.len()).unwrap_or(0);
    let run_cov = sweep
        .as_ref()
        .map(|(pos, _)| lut::span_coverage(pos, raw_min, raw_max))
        .unwrap_or(0.0);

    // significant when >1% of decoded frames went missing (advisory heuristic)
    let significant_gaps = stats.frames > 0 && stats.seq_gaps.saturating_mul(100) > stats.frames;
    let verdict = if cls.out_of_range > 0 || cls.implausible_jumps > 0 {
        "looks like bit-errors in-frame (LinkE UART / PWM noise corrupting contents)"
    } else if significant_gaps {
        "looks like CDC frame drops (values clean, whole frames lost)"
    } else {
        "stream looks clean"
    };

    println!("--- classification ---");
    println!("bytes:             {}", bytes.len());
    println!("frames decoded:    {}", stats.frames);
    println!("realigns:          {}", stats.realigns);
    println!("seq gaps:          {}", stats.seq_gaps);
    println!(
        "out-of-range pos:  {} (pos > 4095, in-frame bit corruption)",
        cls.out_of_range
    );
    println!(
        "implausible jumps: {} (|pos delta| > {} across consecutive seq; max consec delta {})",
        cls.implausible_jumps, MAX_STEP_COUNTS, cls.max_consec_delta
    );
    if observed_used {
        println!("note: rails from observed plausible pos bounds (no --raw-min/--raw-max)");
    }
    println!(
        "longest run:       {run_len} samples, span coverage {:.0}% of rails {raw_min}..{raw_max}",
        run_cov * 100.0
    );
    println!(
        "chunks:            {} (samples total {chunk_samples})",
        chunks.len()
    );
    println!("verdict:           {verdict}");

    println!("--- pipeline replay ---");
    if chunks.is_empty() {
        println!("no usable chunks");
    } else {
        println!(
            "stitching {} chunks ({chunk_samples} samples)",
            chunks.len()
        );
        match stitched_motor_revs(&chunks, args.fs, RIPPLE_PER_REV, raw_min, raw_max) {
            Some((m, cov)) => println!(
                "motor revs (full travel): {m:.2} (stitched coverage {:.0}%)",
                cov * 100.0
            ),
            None => println!("motor revs (full travel): unavailable"),
        }
        // same LUT summary wording as cal::run; build_multi decides populated vs
        // identity (stitched coverage / ripple SNR) internally.
        let l = build_multi(&chunks, args.fs, RIPPLE_PER_REV, raw_min, raw_max);
        let populated = l.corr.iter().any(|&c| c != 0);
        if populated {
            let maxc = l.corr.iter().map(|&c| c.unsigned_abs()).max().unwrap_or(0);
            println!("pot LUT: populated, max |corr| {maxc} counts");
        } else {
            let all_pos: Vec<u16> = chunks
                .iter()
                .flat_map(|(pos, _)| pos.iter().copied())
                .collect();
            let cov = lut::span_coverage(&all_pos, raw_min, raw_max);
            println!(
                "pot LUT: identity (stitched coverage {:.0}% of travel)",
                cov * 100.0
            );
        }
    }
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn mask_parses_hex_with_and_without_prefix() {
        assert_eq!(parse_mask("0x1B").unwrap(), 0x1B);
        assert_eq!(parse_mask("1b").unwrap(), 0x1B);
        assert_eq!(parse_mask("3F").unwrap(), 0x3F);
        assert_eq!(parse_mask(" 0X0f ").unwrap(), 0x0F);
        assert!(parse_mask("zz").is_err());
        assert!(parse_mask("").is_err());
    }

    fn frame(seq: u8, pos: u16) -> TelFrame {
        TelFrame {
            seq,
            pos: Some(pos),
            current: Some(0),
            ..Default::default()
        }
    }

    #[test]
    fn classify_counts_out_of_range_and_within_range_jumps() {
        let frames = vec![
            frame(0, 1000),
            frame(1, 1010), // +10, ok
            frame(2, 5000), // out-of-range (bit corruption)
            frame(3, 1020), // 2->3 consecutive but seq 2 out-of-range -> not a jump
            frame(4, 1900), // 1020->1900 = 880, within-range implausible jump
            frame(6, 1905), // seq gap (5 dropped) -> not consecutive, skipped
        ];
        let c = classify(&frames);
        assert_eq!(c.out_of_range, 1);
        assert_eq!(c.implausible_jumps, 1);
        assert_eq!(c.max_consec_delta, 880);
    }

    #[test]
    fn classify_clean_stream_has_no_flags() {
        // small monotonic steps, all consecutive, all in range
        let frames: Vec<TelFrame> = (0..8u8).map(|k| frame(k, 1000 + k as u16)).collect();
        let c = classify(&frames);
        assert_eq!(c.out_of_range, 0);
        assert_eq!(c.implausible_jumps, 0);
        assert_eq!(c.max_consec_delta, 1);
    }
}
