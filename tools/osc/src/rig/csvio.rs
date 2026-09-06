//! CSV record and replay. Two tiers: raw per-experiment logs (every parsed
//! snapshot, every TEL frame - the complete record), and derived fit-input
//! CSVs (dwell samples, rungs, step series) that `ident fit <dir>` reads
//! back so fit changes never need rig time. The derived columns mirror the
//! osc-ident structs exactly - the round-trip tests pin it.

use std::fs::File;
use std::io::{BufRead, BufReader, BufWriter, Write};
use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use osc_ident::exp::WindowSample;
use osc_ident::exp::ladder::RungSummary;
use osc_ident::exp::resistance::DwellSample;
use osc_ident::fits::{RungPoint, StepSeries};
use osc_ident::frame::{TelFrame, TelemetrySnapshot};

pub(crate) struct OutDir(pub(crate) PathBuf);

impl OutDir {
    pub(crate) fn create(base: &Path) -> Result<Self> {
        let stamp = std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .map(|d| d.as_secs())
            .unwrap_or(0);
        let dir = base.join(format!("{stamp}"));
        std::fs::create_dir_all(&dir).with_context(|| format!("mkdir {}", dir.display()))?;
        Ok(Self(dir))
    }

    pub(crate) fn file(&self, name: &str) -> Result<BufWriter<File>> {
        let p = self.0.join(name);
        Ok(BufWriter::new(
            File::create(&p).with_context(|| format!("create {}", p.display()))?,
        ))
    }
}

/// Raw snapshot log: one row per Read, full field dump.
pub(crate) struct SnapshotLog {
    w: BufWriter<File>,
}

impl SnapshotLog {
    pub(crate) fn create(dir: &OutDir, name: &str) -> Result<Self> {
        let mut w = dir.file(name)?;
        writeln!(
            w,
            "host_ms,fault_flags,fault_code,mode_active,theta_hat_q16,omega_hat_cps,\
             tau_d_counts,i_lim_counts,t_winding_cc,vbus_counts,duty_applied_q15,\
             omega_bemf_cps,r_hat_q12,i_hat_counts,sample_tick,pos,current,\
             current_trough,current_bias_counts,i_mean_counts,i_min_counts,\
             i_max_counts,vdiff_mean,duty_mean_q15,agg_seq"
        )?;
        Ok(Self { w })
    }

    pub(crate) fn push(&mut self, host_ms: f64, s: &TelemetrySnapshot) -> Result<()> {
        writeln!(
            self.w,
            "{host_ms:.1},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{},{}",
            s.fault_flags,
            s.fault_code,
            s.mode_active,
            s.theta_hat_q16,
            s.omega_hat_cps,
            s.tau_d_counts,
            s.i_lim_counts,
            s.t_winding_cc,
            s.vbus_counts,
            s.duty_applied_q15,
            s.omega_bemf_cps,
            s.r_hat_q12,
            s.i_hat_counts,
            s.sample_tick,
            s.pos,
            s.current,
            s.current_trough,
            s.current_bias_counts,
            s.i_mean_counts,
            s.i_min_counts,
            s.i_max_counts,
            s.vdiff_mean,
            s.duty_mean_q15,
            s.agg_seq
        )?;
        Ok(())
    }
}

pub(crate) fn write_tel_frames(dir: &OutDir, name: &str, frames: &[TelFrame]) -> Result<()> {
    let mut w = dir.file(name)?;
    writeln!(w, "seq,window_valid,pos,current,duty_q15,vdiff")?;
    let opt = |v: Option<i32>| v.map(|v| v.to_string()).unwrap_or_default();
    for f in frames {
        writeln!(
            w,
            "{},{},{},{},{},{}",
            f.seq,
            f.window_valid as u8,
            opt(f.pos.map(|v| v as i32)),
            opt(f.current.map(|v| v as i32)),
            opt(f.duty_q15.map(|v| v as i32)),
            opt(f.vdiff.map(|v| v as i32)),
        )?;
    }
    Ok(())
}

// --- derived fit inputs -----------------------------------------------------

pub(crate) fn write_dwell_samples(dir: &OutDir, samples: &[DwellSample]) -> Result<()> {
    let mut w = dir.file("resistance.csv")?;
    writeln!(w, "dwell,dir,t_ms,i,vdiff,duty_q15")?;
    for s in samples {
        writeln!(
            w,
            "{},{},{},{},{},{}",
            s.dwell, s.dir, s.w.t_ms, s.w.i, s.w.vdiff, s.w.duty_q15
        )?;
    }
    Ok(())
}

pub(crate) fn read_dwell_samples(dir: &Path) -> Result<Vec<DwellSample>> {
    let mut out = Vec::new();
    for parts in rows(&dir.join("resistance.csv"), 6)? {
        out.push(DwellSample {
            dwell: parts[0].parse()?,
            dir: parts[1].parse()?,
            w: WindowSample {
                t_ms: parts[2].parse()?,
                i: parts[3].parse()?,
                vdiff: parts[4].parse()?,
                duty_q15: parts[5].parse()?,
            },
        });
    }
    Ok(out)
}

pub(crate) fn write_rungs(dir: &OutDir, rungs: &[RungSummary]) -> Result<()> {
    let mut w = dir.file("rungs.csv")?;
    writeln!(w, "duty_q15,omega,omega_r2,i,v,windows,used,note")?;
    for r in rungs {
        writeln!(
            w,
            "{},{},{},{},{},{},{},{}",
            r.duty_q15,
            r.omega,
            r.omega_r2,
            r.i,
            r.v,
            r.windows,
            r.used as u8,
            r.note.as_deref().unwrap_or("").replace(',', ";"),
        )?;
    }
    Ok(())
}

/// Used rungs back as fit points (the unused ones only matter to humans).
pub(crate) fn read_rung_points(dir: &Path) -> Result<Vec<RungPoint>> {
    Ok(read_rungs(dir)?
        .iter()
        .filter(|r| r.used)
        .map(|r| RungPoint {
            omega: r.omega,
            i: r.i,
            v: r.v,
        })
        .collect())
}

pub(crate) fn read_rungs(dir: &Path) -> Result<Vec<RungSummary>> {
    let mut out = Vec::new();
    for parts in rows(&dir.join("rungs.csv"), 8)? {
        out.push(RungSummary {
            duty_q15: parts[0].parse()?,
            omega: parts[1].parse()?,
            omega_r2: parts[2].parse()?,
            i: parts[3].parse()?,
            v: parts[4].parse()?,
            windows: parts[5].parse()?,
            used: parts[6] == "1",
            note: if parts[7].is_empty() {
                None
            } else {
                Some(parts[7].clone())
            },
        });
    }
    Ok(out)
}

/// Long form, one row per sample; `src` tags tel/agg per step so the
/// offline fit picks the same smoothing window the live one did.
pub(crate) fn write_step_series(dir: &OutDir, series: &[(StepSeries, bool)]) -> Result<()> {
    let mut w = dir.file("inertia_steps.csv")?;
    writeln!(w, "step,src,duty_q15,t,pos,i,mask")?;
    for (k, (s, tel)) in series.iter().enumerate() {
        let src = if *tel { "tel" } else { "agg" };
        for j in 0..s.t.len() {
            writeln!(
                w,
                "{k},{src},{},{},{},{},{}",
                s.duty_q15, s.t[j], s.pos[j], s.i[j], s.mask[j] as u8
            )?;
        }
    }
    Ok(())
}

pub(crate) fn read_step_series(dir: &Path) -> Result<Vec<(StepSeries, bool)>> {
    let mut out: Vec<(StepSeries, bool)> = Vec::new();
    let mut cur: Option<usize> = None;
    for parts in rows(&dir.join("inertia_steps.csv"), 7)? {
        let k: usize = parts[0].parse()?;
        if cur != Some(k) {
            cur = Some(k);
            out.push((
                StepSeries {
                    t: Vec::new(),
                    pos: Vec::new(),
                    i: Vec::new(),
                    mask: Vec::new(),
                    duty_q15: parts[2].parse()?,
                },
                parts[1] == "tel",
            ));
        }
        let s = &mut out.last_mut().expect("pushed").0;
        s.t.push(parts[3].parse()?);
        s.pos.push(parts[4].parse()?);
        s.i.push(parts[5].parse()?);
        s.mask.push(parts[6] == "1");
    }
    Ok(out)
}

fn rows(path: &Path, cols: usize) -> Result<Vec<Vec<String>>> {
    let f = File::open(path).with_context(|| format!("open {}", path.display()))?;
    let mut out = Vec::new();
    for (n, line) in BufReader::new(f).lines().enumerate() {
        let line = line?;
        if n == 0 || line.is_empty() {
            continue;
        }
        let parts: Vec<String> = line.split(',').map(str::to_string).collect();
        anyhow::ensure!(
            parts.len() >= cols,
            "{}:{} has {} cols, want {cols}",
            path.display(),
            n + 1,
            parts.len()
        );
        out.push(parts);
    }
    Ok(out)
}

#[cfg(test)]
mod tests {
    use super::*;

    fn tmp() -> OutDir {
        let dir = std::env::temp_dir().join(format!("ident-csv-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        OutDir(dir)
    }

    #[test]
    fn dwell_samples_round_trip() {
        let dir = tmp();
        let samples = vec![
            DwellSample {
                dwell: 0,
                dir: 1,
                w: WindowSample {
                    t_ms: 12.8,
                    i: 55.5,
                    vdiff: 1700.0,
                    duty_q15: 8520.0,
                },
            },
            DwellSample {
                dwell: 3,
                dir: -1,
                w: WindowSample {
                    t_ms: 900.0,
                    i: -60.25,
                    vdiff: -1690.0,
                    duty_q15: -11468.0,
                },
            },
        ];
        write_dwell_samples(&dir, &samples).unwrap();
        let back = read_dwell_samples(&dir.0).unwrap();
        assert_eq!(back.len(), 2);
        for (a, b) in samples.iter().zip(&back) {
            assert_eq!(a.dwell, b.dwell);
            assert_eq!(a.dir, b.dir);
            assert_eq!(a.w, b.w);
        }
    }

    #[test]
    fn rungs_round_trip_used_only() {
        let dir = tmp();
        let rungs = vec![
            RungSummary {
                duty_q15: 8520,
                omega: 1500.5,
                omega_r2: 0.999,
                i: 40.0,
                v: 450.0,
                windows: 80,
                used: true,
                note: None,
            },
            RungSummary {
                duty_q15: 20971,
                omega: 0.0,
                omega_r2: 0.0,
                i: 0.0,
                v: 0.0,
                windows: 3,
                used: false,
                note: Some("steady segment too short, dropped".into()),
            },
        ];
        write_rungs(&dir, &rungs).unwrap();
        let pts = read_rung_points(&dir.0).unwrap();
        assert_eq!(pts.len(), 1);
        assert_eq!(pts[0].omega, 1500.5);
        assert_eq!(pts[0].i, 40.0);
        assert_eq!(pts[0].v, 450.0);
    }

    #[test]
    fn step_series_round_trip() {
        let dir = tmp();
        let series = vec![
            (
                StepSeries {
                    t: vec![0.0, 0.001, 0.002],
                    pos: vec![400.0, 402.5, 407.0],
                    i: vec![300.0, 280.0, 260.0],
                    mask: vec![true, true, false],
                    duty_q15: 14745.0,
                },
                true,
            ),
            (
                StepSeries {
                    t: vec![0.0, 0.0008],
                    pos: vec![3600.0, 3595.0],
                    i: vec![-310.0, -290.0],
                    mask: vec![true, true],
                    duty_q15: -14745.0,
                },
                false,
            ),
        ];
        write_step_series(&dir, &series).unwrap();
        let back = read_step_series(&dir.0).unwrap();
        assert_eq!(back.len(), 2);
        for ((s, tel), (bs, btel)) in series.iter().zip(&back) {
            assert_eq!(tel, btel);
            assert_eq!(s.t, bs.t);
            assert_eq!(s.pos, bs.pos);
            assert_eq!(s.i, bs.i);
            assert_eq!(s.mask, bs.mask);
            assert_eq!(s.duty_q15, bs.duty_q15);
        }
    }
}
