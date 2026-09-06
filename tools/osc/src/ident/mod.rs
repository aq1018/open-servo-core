//! `osc ident` -- the identification subcommand: drives the osc-ident
//! experiments over the osc-adapter, records raw + derived CSVs, fits the
//! plant, synthesizes and encodes gains, and writes them back with
//! snapshot/rollback safety. The sans-io engine lives in osc-ident; this
//! wrapper owns USB, the TEL serial port, wall time, and files.

pub(crate) mod params;

use std::path::PathBuf;

use crate::rig::pump::{self, Pump, write_reg};
use crate::rig::{csvio, snapshot};
use anyhow::{Context, Result, bail};
use clap::Subcommand;
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::exp::bias::{Bias, BiasCfg};
use osc_ident::exp::breakaway::{Breakaway, BreakawayCfg};
use osc_ident::exp::inertia::{Inertia, InertiaCfg};
use osc_ident::exp::ladder::{Ladder, LadderCfg, LadderResult};
use osc_ident::exp::resistance::{Resistance, ResistanceCfg};
use osc_ident::exp::verify::{
    VerifyCurrent, VerifyCurrentCfg, VerifyResult, VerifyVelocity, VerifyVelocityCfg,
};
use osc_ident::exp::{Guarded, RigParams};
use osc_ident::fits::{self, InertiaPriors};
use osc_ident::gains::{self, BwTargets, PlantParams};
use osc_ident::regs::{calib, control};
use osc_ident::report::{self, ReportInputs};
use params::{
    BiasJson, BreakawayJson, GainJson, InertiaJson, LadderJson, ParamsFile, PlantJson,
    ResistanceJson, SenseJson,
};

/// The `osc ident` arg group: TEL wiring, output, rig envelope, and
/// bandwidth targets, all scoped to the ident subtree. `--baud`/`--id` come
/// from the top-level osc globals.
#[derive(clap::Args, Debug)]
pub struct Args {
    /// TEL stream serial device (the LinkE CDC); empty disables TEL.
    #[arg(long, global = true, default_value = "/dev/cu.usbmodemB3608F06381D2")]
    tel_port: String,
    /// Output directory root; runs land in <out>/<timestamp>/.
    #[arg(long, global = true, default_value = "./ident-out")]
    out: PathBuf,
    // rig envelope
    #[arg(long, global = true, default_value_t = 150)]
    guard_lo: u16,
    #[arg(long, global = true, default_value_t = 3950)]
    guard_hi: u16,
    #[arg(long, global = true, default_value_t = 1250)]
    slip_lo: u16,
    #[arg(long, global = true, default_value_t = 1650)]
    slip_hi: u16,
    #[arg(long, global = true, default_value_t = 1100)]
    i_abort: i16,
    /// Motor inductance, henries (not identifiable from this telemetry).
    #[arg(long, global = true, default_value_t = gains::DEFAULT_L_HENRIES)]
    l_henries: f64,
    /// Nominal gear ratio, informational only (printed in the report dir).
    #[arg(long, global = true)]
    gear_ratio: Option<f64>,
    // bandwidth targets, Hz
    #[arg(long, global = true, default_value_t = 1000.0)]
    f_ci: f64,
    #[arg(long, global = true, default_value_t = 200.0)]
    f_cv: f64,
    #[arg(long, global = true, default_value_t = 25.0)]
    f_cp: f64,
    #[arg(long, global = true, default_value_t = 15.0)]
    f_o: f64,
    #[command(subcommand)]
    cmd: Cmd,
}

/// Runner context: the ident args plus the resolved bus baud, threaded
/// through every experiment fn (the runners predate the arg-group split and
/// still read `cli.field`).
struct Ctx {
    baud: String,
    tel_port: String,
    out: PathBuf,
    guard_lo: u16,
    guard_hi: u16,
    slip_lo: u16,
    slip_hi: u16,
    i_abort: i16,
    l_henries: f64,
    gear_ratio: Option<f64>,
    f_ci: f64,
    f_cv: f64,
    f_cp: f64,
    f_o: f64,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// The full pipeline: bias -> resistance -> breakaway -> ladder ->
    /// inertia -> fit -> report + params.json. Write-back stays explicit.
    Run,
    /// E0: torque-off noise and bias floor.
    Bias,
    /// E2: end-stop stall duty ladder -> winding R.
    Resistance,
    /// E1: breakaway duty ramp (needs R; runs its own bias first).
    Breakaway,
    /// E3: steady-state duty ladder -> Ke + friction line (needs R).
    Ladder,
    /// E4: duty-step transients -> B (TEL when wired).
    Inertia,
    /// E5/E6: closed-loop verification on the written gains.
    Verify,
    /// Refit offline from a recorded run directory.
    Fit { dir: PathBuf },
    /// Write a params.json gain set to the table (snapshot taken first).
    Write {
        params: PathBuf,
        /// Persist with MGMT SAVE after the verified write.
        #[arg(long)]
        save: bool,
    },
    /// Restore a snapshot.json written by `write`.
    Rollback { snapshot: PathBuf },
    /// Print the current table values of every ident-owned field.
    Show,
}

/// Entry from the top-level `osc ident` dispatch. `baud`/`id` are the osc
/// globals; `args` carries the ident-scoped flags and subcommand.
pub fn run(args: &Args, baud: String, id: u8) -> Result<()> {
    let cli = Ctx {
        baud,
        tel_port: args.tel_port.clone(),
        out: args.out.clone(),
        guard_lo: args.guard_lo,
        guard_hi: args.guard_hi,
        slip_lo: args.slip_lo,
        slip_hi: args.slip_hi,
        i_abort: args.i_abort,
        l_henries: args.l_henries,
        gear_ratio: args.gear_ratio,
        f_ci: args.f_ci,
        f_cv: args.f_cv,
        f_cp: args.f_cp,
        f_o: args.f_o,
    };
    pump::install_ctrlc();
    if let Cmd::Fit { dir } = &args.cmd {
        return fit_dir(&cli, dir.clone());
    }
    let mut c = crate::rig::connect(&cli.baud)?;
    let id = Id::new(id);
    match &args.cmd {
        Cmd::Run => run_all(&cli, &mut c, id),
        Cmd::Bias => {
            let out = csvio::OutDir::create(&cli.out)?;
            let (b, _) = run_bias(&cli, &mut c, id, &out)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    bias: Some(&b),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Resistance => {
            let out = csvio::OutDir::create(&cli.out)?;
            let r = run_resistance(&cli, &mut c, id, &out)?;
            println!(
                "R = {:.4} vcounts/ccount (r2 {:.4}, n {}, drift {:+.5}/s)",
                r.r_vpc, r.r2, r.n, r.drift_vpc_per_s
            );
            Ok(())
        }
        Cmd::Breakaway => {
            let out = csvio::OutDir::create(&cli.out)?;
            let (bias, vbus) = run_bias(&cli, &mut c, id, &out)?;
            let _ = bias;
            let r = run_resistance(&cli, &mut c, id, &out)?;
            let bk = run_breakaway(&cli, &mut c, id, &out, r.r_vpc, vbus)?;
            println!("{bk:#?}");
            Ok(())
        }
        Cmd::Ladder => {
            let out = csvio::OutDir::create(&cli.out)?;
            let r = run_resistance(&cli, &mut c, id, &out)?;
            let l = run_ladder(&cli, &mut c, id, &out, r.r_vpc)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    ladder: Some(&l),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Inertia => {
            let out = csvio::OutDir::create(&cli.out)?;
            let r = run_resistance(&cli, &mut c, id, &out)?;
            let l = run_ladder(&cli, &mut c, id, &out, r.r_vpc)?;
            let sense = read_sense(&mut c, id)?;
            let priors = priors_of(r.r_vpc, &l, &sense);
            let i = run_inertia(&cli, &mut c, id, &out, &priors)?;
            println!(
                "{}",
                render_partial(ReportInputs {
                    inertia: Some(&i),
                    ..Default::default()
                })
            );
            Ok(())
        }
        Cmd::Verify => run_verify(&cli, &mut c, id),
        Cmd::Write { params, save } => {
            let p = ParamsFile::load(params)?;
            if p.gains.is_empty() {
                bail!(
                    "{}: no gains section (run `ident fit` first)",
                    params.display()
                );
            }
            let snap = params
                .parent()
                .unwrap_or(std::path::Path::new("."))
                .join("snapshot.json");
            snapshot::take_snapshot(&mut c, id, &snap)?;
            snapshot::write_gains(&mut c, id, &p.gains)?;
            if *save {
                write_reg(&mut c, id, control::TORQUE_ENABLE, 0)?;
                c.save(id).context("MGMT SAVE")?;
                println!("saved");
            }
            Ok(())
        }
        Cmd::Rollback { snapshot } => snapshot::rollback(&mut c, id, snapshot),
        Cmd::Show => snapshot::show(&mut c, id),
        Cmd::Fit { .. } => unreachable!("handled above"),
    }
}

fn rig(cli: &Ctx) -> RigParams {
    RigParams {
        pos_guard: Some((cli.guard_lo, cli.guard_hi)),
        i_abort: cli.i_abort,
        slip: (cli.slip_lo, cli.slip_hi),
        ..RigParams::default()
    }
}

fn targets(cli: &Ctx) -> BwTargets {
    BwTargets {
        f_ci: cli.f_ci,
        f_cv: cli.f_cv,
        f_cp: cli.f_cp,
        f_o: cli.f_o,
    }
}

/// Every drive path funnels through this: run the closure, then force the
/// servo safe (duty/goals zero, torque and TEL off) whether it succeeded,
/// failed, or was ctrl-c'd. A hard kill skips this - the servo's own
/// protections are the backstop.
fn with_guard<T>(
    c: &mut Client<NusbPipe>,
    id: Id,
    f: impl FnOnce(&mut Client<NusbPipe>) -> Result<T>,
) -> Result<T> {
    let r = f(c);
    for (reg, v) in [
        (control::GOAL_DUTY, 0),
        (control::GOAL_CURRENT, 0),
        (control::GOAL_VELOCITY, 0),
        (control::TORQUE_ENABLE, 0),
        (control::TEL_ENABLE, 0),
        (control::TEL_MASK, 0),
    ] {
        let _ = write_reg(c, id, reg, v);
    }
    r
}

/// OpenLoop nudge to mid-travel. End-stop work (E2) parks the pot on a
/// rail where the next experiment's pos guard would abort before it can
/// move; every in-band experiment recenters first.
fn recenter(c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    const LO: u16 = 1750;
    const HI: u16 = 2350;
    let pos0 = pump::read_snapshot(c, id)?.pos;
    if (LO..=HI).contains(&pos0) {
        return Ok(());
    }
    println!("[recenter] pos {pos0} -> mid-travel");
    with_guard(c, id, |c| {
        pump::write_reg(c, id, control::MODE, 0)?;
        pump::write_reg(c, id, control::TORQUE_ENABLE, 1)?;
        for _ in 0..200 {
            let pos = pump::read_snapshot(c, id)?.pos;
            if (LO..=HI).contains(&pos) {
                return Ok(());
            }
            let duty = if pos < LO { 9000 } else { -9000 };
            pump::write_reg(c, id, control::GOAL_DUTY, duty)?;
            std::thread::sleep(std::time::Duration::from_millis(25));
        }
        anyhow::bail!("recenter: not in band after 5 s (gear slipping?)")
    })
}

fn read_sense(c: &mut Client<NusbPipe>, id: Id) -> Result<SenseJson> {
    Ok(SenseJson {
        shunt_r_mohm: snapshot::read_u16(c, id, calib::SHUNT_R_MOHM)?,
        gain_milli: snapshot::read_u16(c, id, calib::GAIN_MILLI)?,
        vmotor_div_top: snapshot::read_u16(c, id, calib::VMOTOR_DIV_TOP)?,
        vmotor_div_bot: snapshot::read_u16(c, id, calib::VMOTOR_DIV_BOT)?,
        tick_hz: snapshot::read_u16(c, id, calib::TICK_HZ)?,
    })
}

// --- experiment runners -----------------------------------------------------

fn run_bias(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
) -> Result<(osc_ident::exp::bias::BiasResult, f64)> {
    println!("[E0 bias]");
    // a rail-parked pot clips the noise measurement (and trips the guard)
    recenter(c, id)?;
    let mut log = csvio::SnapshotLog::create(out, "bias_snapshots.csv")?;
    let mut exp = Guarded::new(Bias::new(BiasCfg::default()), rig(cli));
    with_guard(c, id, |c| {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: Some(&mut log),
            tel_raw_path: None,
        }
        .run(&mut exp, |_| {})
    })?;
    check_abort("bias", exp.abort())?;
    let b = exp
        .into_inner()
        .result()
        .context("bias collected no samples")?;
    let vbus = b.vbus_mean;
    Ok((b, vbus))
}

fn run_resistance(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
) -> Result<osc_ident::exp::resistance::ResistanceResult> {
    println!("[E2 resistance] (end-stop stalls; pos guard off)");
    let params = rig(cli).without_pos_guard();
    let mut log = csvio::SnapshotLog::create(out, "resistance_snapshots.csv")?;
    let mut exp = Guarded::new(Resistance::new(ResistanceCfg::default(), &params), params);
    with_guard(c, id, |c| {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: Some(&mut log),
            tel_raw_path: None,
        }
        .run(&mut exp, |_| {})
    })?;
    check_abort("resistance", exp.abort())?;
    let exp = exp.into_inner();
    csvio::write_dwell_samples(out, exp.samples())?;
    exp.fit().context("resistance fit degenerate")
}

fn run_breakaway(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    r_vpc: f64,
    vbus_mean: f64,
) -> Result<osc_ident::exp::breakaway::BreakawayResult> {
    println!("[E1 breakaway]");
    recenter(c, id)?;
    let mut log = csvio::SnapshotLog::create(out, "breakaway_snapshots.csv")?;
    let mut exp = Guarded::new(Breakaway::new(BreakawayCfg::default()), rig(cli));
    with_guard(c, id, |c| {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: Some(&mut log),
            tel_raw_path: None,
        }
        .run(&mut exp, |_| {})
    })?;
    check_abort("breakaway", exp.abort())?;
    Ok(exp.into_inner().fit(r_vpc, vbus_mean))
}

fn run_ladder(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    r_vpc: f64,
) -> Result<LadderResult> {
    println!("[E3 ladder]");
    recenter(c, id)?;
    let params = rig(cli);
    let mut log = csvio::SnapshotLog::create(out, "ladder_snapshots.csv")?;
    let mut exp = Guarded::new(Ladder::new(LadderCfg::default(), &params), params);
    with_guard(c, id, |c| {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: Some(&mut log),
            tel_raw_path: None,
        }
        .run(&mut exp, |_| {})
    })?;
    check_abort("ladder", exp.abort())?;
    let l = exp
        .into_inner()
        .fit(r_vpc)
        .context("ladder fit degenerate")?;
    csvio::write_rungs(out, &l.rungs)?;
    Ok(l)
}

fn run_inertia(
    cli: &Ctx,
    c: &mut Client<NusbPipe>,
    id: Id,
    out: &csvio::OutDir,
    priors: &InertiaPriors,
) -> Result<osc_ident::exp::inertia::InertiaResult> {
    println!(
        "[E4 inertia]{}",
        if cli.tel_port.is_empty() {
            " (no TEL)"
        } else {
            ""
        }
    );
    recenter(c, id)?;
    let params = rig(cli);
    let cfg = InertiaCfg {
        tick_hz: priors.tick_hz,
        ..InertiaCfg::default()
    };
    let mut log = csvio::SnapshotLog::create(out, "inertia_snapshots.csv")?;
    let mut exp = Guarded::new(Inertia::new(cfg, &params), params);
    let mut all_tel = Vec::new();
    with_guard(c, id, |c| {
        let mut pump = Pump {
            client: c,
            id,
            tel_port: (!cli.tel_port.is_empty()).then(|| cli.tel_port.clone()),
            tel_mask: 0x1B,
            log: Some(&mut log),
            tel_raw_path: None,
        };
        // split borrow: hand frames to the guarded experiment mid-run
        let exp_cell = std::cell::RefCell::new(&mut exp);
        let mut adapter = PumpAdapter { exp: &exp_cell };
        pump.run(&mut adapter, |frames| {
            all_tel.extend_from_slice(frames);
            exp_cell.borrow_mut().inner_mut().push_tel(frames);
        })
    })?;
    check_abort("inertia", exp.abort())?;
    let exp = exp.into_inner();
    csvio::write_tel_frames(out, "inertia_tel.csv", &all_tel)?;
    csvio::write_step_series(out, &exp.step_series())?;
    exp.fit(priors).context("inertia fit degenerate")
}

/// Lets the TEL callback borrow the experiment the pump is stepping: the
/// pump only holds this thin adapter, and both it and the callback reach
/// the real experiment through the RefCell (never concurrently - the pump
/// is single-threaded).
struct PumpAdapter<'a, 'b, E> {
    exp: &'a std::cell::RefCell<&'b mut E>,
}

impl<E: osc_ident::exp::Experiment> osc_ident::exp::Experiment for PumpAdapter<'_, '_, E> {
    fn step(&mut self, obs: Option<&osc_ident::frame::TelemetrySnapshot>) -> osc_ident::exp::Cmd {
        self.exp.borrow_mut().step(obs)
    }
}

fn run_verify(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let params = rig(cli);
    let tick_hz = snapshot::read_u16(c, id, calib::TICK_HZ)? as f64;
    recenter(c, id)?;
    println!("[E5 current steps]");
    let mut e5 = Guarded::new(
        VerifyCurrent::new(VerifyCurrentCfg::default(), &params),
        params.without_pos_guard(),
    );
    with_guard(c, id, |c| {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: None,
            tel_raw_path: None,
        }
        .run(&mut e5, |_| {})
    })?;
    check_abort("verify-current", e5.abort())?;
    let cur = e5.into_inner().result();
    // E5 ends stalled against an end-stop; E6 runs with the pos guard on
    // and its first read would abort right there
    recenter(c, id)?;
    println!("[E6 velocity legs]");
    let mut e6 = Guarded::new(
        VerifyVelocity::new(VerifyVelocityCfg::default(), &params, tick_hz),
        params,
    );
    with_guard(c, id, |c| {
        Pump {
            client: c,
            id,
            tel_port: None,
            tel_mask: 0,
            log: None,
            tel_raw_path: None,
        }
        .run(&mut e6, |_| {})
    })?;
    check_abort("verify-velocity", e6.abort())?;
    let vel = e6.into_inner().result();
    for s in &cur.steps {
        println!(
            "  goal {:+5} -> {:+8.1} ({:.1}% err, settle {})",
            s.goal,
            s.mean_i,
            s.err_pct,
            s.settle_ms
                .map(|t| format!("{t:.0} ms"))
                .unwrap_or_else(|| "never".into()),
        );
    }
    for l in &vel.legs {
        println!(
            "  goal {:+6} c/s -> {:+8.1} ({:.1}% err, r2 {:.4}, n {})",
            l.goal_cps, l.meas_cps, l.err_pct, l.r2, l.n
        );
    }
    let v = VerifyResult::assemble(Some(cur), Some(vel));
    println!("verify: {}", if v.pass { "PASS" } else { "FAIL" });
    if !v.pass {
        std::process::exit(1);
    }
    Ok(())
}

fn check_abort(name: &str, abort: Option<osc_ident::exp::AbortReason>) -> Result<()> {
    match abort {
        None => Ok(()),
        Some(r) => bail!("{name} aborted by the safety envelope: {r:?}"),
    }
}

// --- fitting ----------------------------------------------------------------

fn priors_of(r_vpc: f64, l: &LadderResult, sense: &SenseJson) -> InertiaPriors {
    let mean_opt = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a + b) / 2.0,
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0.0,
    };
    InertiaPriors {
        r_vpc,
        ke_vpc: l.ke.ke_vpc,
        fc: mean_opt(l.fric_fwd.map(|f| f.fc), l.fric_rev.map(|f| f.fc)),
        fv: mean_opt(l.fric_fwd.map(|f| f.fv), l.fric_rev.map(|f| f.fv)),
        tick_hz: sense.tick_hz as f64,
    }
}

fn run_all(cli: &Ctx, c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let out = csvio::OutDir::create(&cli.out)?;
    println!("recording to {}", out.0.display());
    let sense = read_sense(c, id)?;
    let (bias, vbus) = run_bias(cli, c, id, &out)?;
    let resistance = run_resistance(cli, c, id, &out)?;
    let breakaway = run_breakaway(cli, c, id, &out, resistance.r_vpc, vbus)?;
    let ladder = run_ladder(cli, c, id, &out, resistance.r_vpc)?;
    let priors = priors_of(resistance.r_vpc, &ladder, &sense);
    // the live fit is discarded on purpose: run only records, fit_dir below
    // recomputes everything from the files so run and refit cannot diverge
    let _ = run_inertia(cli, c, id, &out, &priors)?;

    let p = ParamsFile {
        bias: Some(BiasJson::from(&bias)),
        resistance: Some(ResistanceJson::from(&resistance)),
        breakaway: Some(BreakawayJson::from(&breakaway)),
        sense: Some(sense),
        ..Default::default()
    };
    p.save(&out.0.join("params.json"))?;
    if let Some(g) = cli.gear_ratio {
        std::fs::write(out.0.join("gear_ratio.txt"), format!("{g}\n"))?;
    }
    // the offline path is THE fit path - run records, fit computes
    fit_dir(cli, out.0.clone())
}

/// Refit from a recorded directory: reads params.json (bias, breakaway,
/// sense) plus the derived CSVs, recomputes every fit, synthesizes, and
/// rewrites params.json with the plant and encoded gains.
fn fit_dir(cli: &Ctx, dir: PathBuf) -> Result<()> {
    let path = dir.join("params.json");
    let mut p = ParamsFile::load(&path)?;
    let sense = p.sense.context("params.json has no sense block")?;
    let tick_hz = sense.tick_hz as f64;

    let dwells = csvio::read_dwell_samples(&dir)?;
    let resistance = Resistance::fit_samples(&dwells).context("resistance refit degenerate")?;
    let rungs = csvio::read_rungs(&dir)?;
    let pts: Vec<fits::RungPoint> = csvio::read_rung_points(&dir)?;
    let ke = fits::ke_fit(&pts, resistance.r_vpc).context("ke refit degenerate")?;
    let fric_fwd = fits::friction_line(&pts, 1);
    let fric_rev = fits::friction_line(&pts, -1);
    let ladder = LadderResult {
        ke,
        fric_fwd,
        fric_rev,
        rungs,
        warnings: Vec::new(),
    };
    let priors = priors_of(resistance.r_vpc, &ladder, &sense);
    let series = csvio::read_step_series(&dir)?;
    let tel_steps = series.iter().filter(|(_, tel)| *tel).count();
    // same smoothing-window rule as Inertia::fit
    let hw = if tel_steps > 0 {
        (0.010 * tick_hz) as usize
    } else {
        12
    };
    let b_direct = fits::b_direct_fit(&series_only(&series), &priors, hw, 5.0);
    let b_exp = fits::b_exp_fit(&series_only(&series), &priors, hw);
    let b_best = match (&b_exp, &b_direct) {
        (Some(e), Some(d)) => {
            if d.r2 > 0.98 && d.r2 > 1.0 - e.spread {
                d.b
            } else {
                e.b
            }
        }
        (Some(e), None) => e.b,
        (None, Some(d)) => d.b,
        (None, None) => bail!("no inertia estimate from either estimator"),
    };
    let inertia = osc_ident::exp::inertia::InertiaResult {
        b_direct,
        b_exp,
        b_best,
        j_ff: 1.0 / b_best,
        tel_steps,
        warnings: Vec::new(),
    };

    let bias = p.bias;
    let sigma_theta = bias.map(|b| b.sigma_theta).unwrap_or(1.0);
    let l_cd = gains::l_cd_from_si(
        cli.l_henries,
        sense.shunt_r_mohm,
        sense.gain_milli,
        sense.vmotor_div_top,
        sense.vmotor_div_bot,
    )
    .context("sense scales degenerate")?;
    let mean_opt = |a: Option<f64>, b: Option<f64>| match (a, b) {
        (Some(a), Some(b)) => (a + b) / 2.0,
        (Some(a), None) | (None, Some(a)) => a,
        (None, None) => 0.0,
    };
    let plant = PlantParams {
        r_vpc: resistance.r_vpc,
        ke_vpc: ladder.ke.ke_vpc,
        fc: mean_opt(ladder.fric_fwd.map(|f| f.fc), ladder.fric_rev.map(|f| f.fc)),
        fv: mean_opt(ladder.fric_fwd.map(|f| f.fv), ladder.fric_rev.map(|f| f.fv)),
        b: inertia.b_best,
        sigma_theta,
        l_cd,
        tick_hz,
        f_med: tick_hz / 10.0,
    };
    let t = targets(cli);
    let gains_set = gains::synthesize(&plant, &t);
    let encoded = gains::encode(&gains_set);

    let bias_res = bias.map(|b| osc_ident::exp::bias::BiasResult {
        sigma_theta: b.sigma_theta,
        pos_mean: b.pos_mean,
        i_noise: b.i_noise,
        i_bias_delta: b.i_bias_delta,
        vbus_mean: b.vbus_mean,
        vbus_sd: b.vbus_sd,
        n: b.n,
    });
    let bk_res = p
        .breakaway
        .map(|b| osc_ident::exp::breakaway::BreakawayResult {
            duty_bk_fwd: b.duty_bk_fwd,
            duty_bk_rev: b.duty_bk_rev,
            fric_fwd_counts: b.fric_fwd_counts,
            fric_rev_counts: b.fric_rev_counts,
            model_derived: b.model_derived,
            asymmetry: b.asymmetry,
        });
    let text = report::render(&ReportInputs {
        bias: bias_res.as_ref(),
        resistance: Some(&resistance),
        breakaway: bk_res.as_ref(),
        ladder: Some(&ladder),
        inertia: Some(&inertia),
        gains: Some((&gains_set, &encoded)),
    });
    println!("{text}");
    std::fs::write(dir.join("report.txt"), &text)?;

    p.resistance = Some(ResistanceJson::from(&resistance));
    p.ladder = Some(LadderJson {
        ke_vpc: ladder.ke.ke_vpc,
        ke_r2: ladder.ke.r2,
        fc_fwd: ladder.fric_fwd.map(|f| f.fc),
        fv_fwd: ladder.fric_fwd.map(|f| f.fv),
        fc_rev: ladder.fric_rev.map(|f| f.fc),
        fv_rev: ladder.fric_rev.map(|f| f.fv),
        rungs_used: ladder.rungs.iter().filter(|r| r.used).count(),
    });
    p.inertia = Some(InertiaJson {
        b_best: inertia.b_best,
        b_direct: inertia.b_direct.as_ref().map(|d| d.b),
        b_exp: inertia.b_exp.as_ref().map(|e| e.b),
        j_ff: inertia.j_ff,
        tel_steps: inertia.tel_steps,
    });
    p.plant = Some(PlantJson::new(&plant, &t));
    p.gains = GainJson::set(&encoded);
    p.save(&path)?;
    println!("params: {}", path.display());
    println!(
        "next: ident write {} [--save], then ident verify",
        path.display()
    );
    Ok(())
}

fn series_only(s: &[(fits::StepSeries, bool)]) -> Vec<fits::StepSeries> {
    s.iter().map(|(s, _)| s.clone()).collect()
}

fn render_partial(inputs: ReportInputs<'_>) -> String {
    report::render(&inputs)
}
