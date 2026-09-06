//! `osc` -- the operator CLI for an osc-native bus, driven through the
//! osc-adapter: discovery, rescue, id/baud management, calibration,
//! persistence, profiles, one-shot reads/writes, typed get/set/dump, and the
//! adapter's own management verbs (rails, bootloader). A thin surface over
//! osc-client -- every choreography lives in the library. Wire measurement is
//! the bench `tool-*` binaries' job, not this tool's.

mod cal;
mod descriptor;
mod ident;
mod rig;

use anyhow::{Context, Result, bail};
use clap::{Parser, Subcommand};
use osc_client::blocking::Client;
use osc_client::mgmt::Uid;
use osc_client::nusb::NusbPipe;
use osc_client::{BaudRate, Id, Inst, Opcode, ResultCode};
use osc_protocol::build;
use osc_protocol::table::{
    PROFILE_SPANS_PER_SLOT, profile_slot_addr, profile_span_split, profile_span_word,
};
use osc_protocol::wire::UID_LEN;

#[derive(Parser, Debug)]
#[command(
    name = "osc",
    about = "Operate an osc-native servo bus through the osc-adapter."
)]
struct Cli {
    /// Wire baud, or `auto` to find the bus by ENUM probe.
    #[arg(short, long, global = true, default_value = "auto")]
    baud: String,
    /// Servo id for the unicast verbs.
    #[arg(short, long, global = true, default_value_t = 1)]
    id: u8,
    #[command(subcommand)]
    cmd: Cmd,
}

#[derive(Subcommand, Debug)]
enum Cmd {
    /// Adapter link info (version, tick rate).
    Info,
    /// Read the DUT power rail state (3V3 / 5V) without driving it.
    Rails,
    /// Drive the 3V3 rail, 5V untouched.
    #[command(name = "3v3")]
    Rail3v3 {
        #[arg(value_parser = on_off, action = clap::ArgAction::Set)]
        on: bool,
    },
    /// Drive the 5V rail, 3V3 untouched.
    #[command(name = "5v")]
    Rail5v {
        #[arg(value_parser = on_off, action = clap::ArgAction::Set)]
        on: bool,
    },
    /// Hand the adapter to the WCH bootloader (4348:55e0, wlink-iap).
    Bootloader,
    /// Enumerate every servo (id + UID) -- all supported bauds, or just --baud.
    Discover,
    /// Rescue-pulse the bus to 0.5 M and enumerate whoever answers.
    Rescue {
        /// After the rescue, migrate every found servo to this rate.
        #[arg(long)]
        set_baud: Option<String>,
        /// Persist each rescued servo's config with MGMT SAVE (torque must
        /// be off).
        #[arg(long)]
        save: bool,
    },
    /// One broadcast ENUM query (LSB-first bit prefix, protocol sec 9.2);
    /// prints the raw reply evidence.
    Enum {
        /// Prefix length in bits (0..=128).
        #[arg(long, default_value_t = 0)]
        prefix_len: u8,
        /// Prefix bytes as hex (`ceil(prefix_len/8)` bytes, wire order).
        #[arg(long, default_value = "")]
        prefix: String,
    },
    /// Move the servo with this UID to a new id (broadcast ASSIGN).
    Assign {
        /// Target UID: 16 hex bytes as printed by discover.
        uid: String,
        /// New id (1..=249). Volatile until SAVE.
        new_id: u8,
    },
    /// Fleet baud migration, servo-first, with the reunion roster.
    Baud {
        /// Target baud.
        to: String,
        /// Servos to migrate; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
    },
    /// Write the fleet RESPONSE_DEADLINE register + the engine's mirror.
    Deadline {
        us: u16,
        /// Servos to write; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
    },
    /// Persist the live config + profiles (protocol sec 9.4; torque must be off).
    Save,
    /// Wipe both saved slots; the servo reboots itself to board defaults.
    Factory,
    /// Ack, then reset once the ack has drained.
    Reboot,
    /// Read or configure a protocol sec 5.2 profile slot.
    Profile {
        #[command(subcommand)]
        cmd: ProfileCmd,
    },
    /// Broadcast CAL break trains (protocol sec 9.3) with per-servo
    /// convergence readback.
    Syntonize {
        /// Trains to send (>= 2 per the sec 9.3 boot guidance).
        #[arg(long, default_value_t = 3)]
        trains: u8,
        /// Break spacing in us, crystal-paced by the adapter.
        #[arg(long, default_value_t = 400)]
        gap_us: u16,
        /// Measured gaps per train (the adapter sends gaps + 1 breaks).
        #[arg(long, default_value_t = 8)]
        gaps: u8,
        /// Servos to trace trim_steps on; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
    },
    /// One ping: model, firmware, the ALERT bit.
    Ping,
    /// Identity + health from the common register block (protocol sec 5.4).
    Status,
    /// Zero the telemetry transport counters (rw-clear).
    Clear {
        /// Servos to clear; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
    },
    /// Read a control-table span, hex-dumped.
    Read {
        /// Table address (0x-hex or decimal).
        addr: String,
        /// Byte count (single status frame: <= 252).
        #[arg(default_value_t = 4)]
        len: u16,
    },
    /// Write hex bytes at a control-table address.
    Write {
        /// Table address (0x-hex or decimal).
        addr: String,
        /// Payload as hex bytes.
        data: String,
        /// Stage under HOLD; applied by the next COMMIT.
        #[arg(long)]
        hold: bool,
        /// Fire-and-forget: no status frame owed.
        #[arg(long, conflicts_with = "hold")]
        noreply: bool,
    },
    /// Uniform group read: one status chain in list order (protocol sec 6).
    Gread {
        /// Table address (0x-hex or decimal).
        addr: String,
        /// Bytes per target.
        #[arg(default_value_t = 4)]
        count: u16,
        /// Targets in chain order; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
    },
    /// Uniform group write: the same data to every target, no replies.
    Gwrite {
        /// Table address (0x-hex or decimal).
        addr: String,
        /// Per-target data as hex bytes.
        data: String,
        /// Targets; defaults to --id.
        #[arg(long, value_delimiter = ',')]
        ids: Vec<u8>,
        /// Stage under HOLD; the fleet applies on COMMIT.
        #[arg(long)]
        hold: bool,
    },
    /// Broadcast COMMIT: every held write applies in the same instant.
    Commit,
    /// Read one register by name, decoded per the device descriptor.
    Get {
        /// Register name (see `dump` for the roster).
        field: String,
    },
    /// Write one register by name, encoded per the device descriptor.
    Set {
        /// Register name.
        field: String,
        /// Value: decimal/0x-hex int, on|off for bool, variant name or number
        /// for enum, hex bytes for a bytes field.
        #[arg(allow_hyphen_values = true)]
        value: String,
        /// Stage under HOLD; applied by the next COMMIT.
        #[arg(long)]
        hold: bool,
        /// Fire-and-forget: no status frame owed.
        #[arg(long, conflicts_with = "hold")]
        noreply: bool,
    },
    /// Read the whole control table, one decoded line per descriptor field.
    Dump,
    /// Identify the motor and synthesize its gains: experiments, fits,
    /// write-back.
    Ident(ident::Args),
    /// Calibrate: find the end-stops, confirm the angle range, write limits +
    /// polarity + angle endpoints, then SAVE.
    Cal(cal::Args),
    /// Replay a saved cal tel-raw.bin offline: classify stream corruption and
    /// re-run the pot-LUT / motor-rev pipeline without a servo.
    CalReplay(cal::replay::Args),
}

#[derive(Subcommand, Debug)]
enum ProfileCmd {
    /// Decode a slot's span words and read the gathered bytes.
    Get {
        #[arg(default_value_t = 0)]
        slot: u8,
    },
    /// Configure a slot's spans (`addr:count`, comma-separated); no spans
    /// clears the slot.
    Set {
        slot: u8,
        #[arg(value_delimiter = ',')]
        spans: Vec<String>,
    },
}

fn on_off(s: &str) -> Result<bool, String> {
    match s {
        "on" => Ok(true),
        "off" => Ok(false),
        other => Err(format!("expected on|off, got {other:?}")),
    }
}

fn print_rails((v3v3, v5): (bool, bool)) -> Result<()> {
    println!(
        "rails: 3V3 {} / 5V {}",
        if v3v3 { "on" } else { "off" },
        if v5 { "on" } else { "off" },
    );
    Ok(())
}

fn hex(bytes: &[u8]) -> String {
    bytes
        .iter()
        .map(|b| format!("{b:02x}"))
        .collect::<Vec<_>>()
        .join(" ")
}

fn parse_hex(s: &str) -> Result<Vec<u8>> {
    let s: String = s.chars().filter(|c| !c.is_whitespace()).collect();
    if !s.len().is_multiple_of(2) {
        bail!("hex payload needs an even digit count");
    }
    (0..s.len())
        .step_by(2)
        .map(|i| Ok(u8::from_str_radix(&s[i..i + 2], 16)?))
        .collect()
}

fn parse_u16(s: &str) -> Result<u16> {
    Ok(match s.strip_prefix("0x") {
        Some(h) => u16::from_str_radix(h, 16)?,
        None => s.parse()?,
    })
}

fn parse_rate(s: &str) -> Result<BaudRate> {
    let bps: u32 = s.parse().context("baud is an integer rate or `auto`")?;
    [
        BaudRate::B500000,
        BaudRate::B1000000,
        BaudRate::B2000000,
        BaudRate::B3000000,
    ]
    .into_iter()
    .find(|r| r.as_hz() == bps)
    .ok_or_else(|| anyhow::anyhow!("unsupported baud {bps} (500000/1000000/2000000/3000000)"))
}

/// Parse a UID in discover's print order (most significant byte first).
fn parse_uid(s: &str) -> Result<Uid> {
    let bytes = parse_hex(s)?;
    let mut uid: [u8; UID_LEN] = bytes
        .try_into()
        .map_err(|b: Vec<u8>| anyhow::anyhow!("uid is 16 hex bytes, got {}", b.len()))?;
    uid.reverse();
    Ok(Uid(uid))
}

/// Parse one profile span given as `addr:count` (protocol sec 5.2 bounds).
fn parse_span(s: &str) -> Result<(u16, u8)> {
    let (a, c) = s
        .split_once(':')
        .ok_or_else(|| anyhow::anyhow!("span is `addr:count`"))?;
    let addr = parse_u16(a)?;
    let count: u8 = c.parse()?;
    if addr > 1023 || count == 0 || count > 63 {
        bail!("span {s}: addr caps at 1023, count at 1..=63");
    }
    Ok((addr, count))
}

fn ids_or_default(ids: &[u8], id: u8) -> Vec<Id> {
    if ids.is_empty() {
        vec![Id::new(id)]
    } else {
        ids.iter().map(|&i| Id::new(i)).collect()
    }
}

fn connect() -> Result<Client<NusbPipe>> {
    Ok(Client::connect(NusbPipe::open()?)?)
}

/// Connect for the bus verbs: fixed baud, or `auto` via ENUM probe.
fn connect_bus(cli: &Cli) -> Result<Client<NusbPipe>> {
    let mut c = connect()?;
    match cli.baud.as_str() {
        "auto" => match c.find_bus_baud()? {
            Some(rate) => println!("bus at {} baud", rate.as_hz()),
            None => bail!("no servo bus found at any supported baud"),
        },
        s => c.host_baud(parse_rate(s)?)?,
    }
    Ok(c)
}

fn discover(cli: &Cli) -> Result<()> {
    let mut c = connect()?;
    let rates: Vec<BaudRate> = match cli.baud.as_str() {
        "auto" => vec![
            BaudRate::B500000,
            BaudRate::B1000000,
            BaudRate::B2000000,
            BaudRate::B3000000,
        ],
        s => vec![parse_rate(s)?],
    };
    let mut total = 0;
    for rate in rates {
        c.host_baud(rate)?;
        let found = c.discover()?;
        if found.is_empty() {
            println!("{:>8}: -", rate.as_hz());
        }
        for f in &found {
            println!(
                "{:>8}: id {:<3} uid {:?}",
                rate.as_hz(),
                f.id.as_byte(),
                f.uid
            );
        }
        total += found.len();
    }
    println!("{total} servo(s)");
    Ok(())
}

fn rescue(set_baud: Option<&str>, do_save: bool) -> Result<()> {
    let mut c = connect()?;
    c.rescue()?;
    // Servo-side confirm completes ~100 us after the pulse ends.
    c.pause(std::time::Duration::from_millis(2));
    let found = c.discover()?;
    if found.is_empty() {
        bail!("rescue pulse sent but nothing answers at the rescue baud");
    }
    for f in &found {
        println!("rescued: id {:<3} uid {:?}", f.id.as_byte(), f.uid);
    }
    let ids: Vec<Id> = found.iter().map(|f| f.id).collect();
    if let Some(to) = set_baud {
        migrate(&mut c, &ids, parse_rate(to)?)?;
    } else {
        println!(
            "bus is at {} baud until reboot (volatile)",
            BaudRate::RESCUE.as_hz()
        );
    }
    if do_save {
        for &id in &ids {
            save(&mut c, id)?;
        }
    }
    Ok(())
}

/// Servo-first migration; the reunion roster is the verdict.
fn migrate(c: &mut Client<NusbPipe>, ids: &[Id], rate: BaudRate) -> Result<()> {
    let roster = c.set_baud(ids, rate)?;
    let mut lost = 0;
    for (id, alive) in roster {
        if !alive {
            println!("id {}: NOT answering at {}", id.as_byte(), rate.as_hz());
            lost += 1;
        }
    }
    if lost > 0 {
        bail!("{lost} servo(s) missing after migration (rescue recovers)");
    }
    println!("answering at {} baud (volatile until SAVE)", rate.as_hz());
    Ok(())
}

fn save(c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    match c.save(id) {
        Ok(()) => {
            println!("id {}: saved", id.as_byte());
            Ok(())
        }
        Err(osc_client::Error::Servo(ResultCode::Access)) => {
            bail!(
                "id {}: SAVE needs torque disabled (protocol sec 9.4)",
                id.as_byte()
            )
        }
        Err(e) => Err(e.into()),
    }
}

fn enum_probe(c: &mut Client<NusbPipe>, prefix_len: u8, prefix_hex: &str) -> Result<()> {
    let bytes = parse_hex(prefix_hex)?;
    if prefix_len > 128 || bytes.len() != (prefix_len as usize).div_ceil(8) {
        bail!("prefix carries ceil(prefix_len/8) bytes, prefix_len <= 128");
    }
    let mut prefix = [0u8; UID_LEN];
    prefix[..bytes.len()].copy_from_slice(&bytes);
    let mut p = [0u8; 2 + UID_LEN];
    let n = build::mgmt_enum(&mut p, prefix_len, &prefix)
        .ok_or_else(|| anyhow::anyhow!("bad prefix"))?;
    let reply = c.exchange(Id::BROADCAST, Inst::instruction(Opcode::Mgmt, 0), &p[..n])?;
    for s in &reply.statuses {
        println!(
            "status: id {:<3} {:?} payload {}",
            s.id,
            s.result,
            hex(&s.payload)
        );
    }
    println!(
        "evidence: {} status(es), garble {}, trailing {}",
        reply.statuses.len(),
        reply.garble,
        reply.trailing
    );
    let clean = reply.garble == 0 && !reply.trailing;
    match (reply.statuses.len(), clean) {
        (0, true) => println!("verdict: silent (empty subtree)"),
        (1, true) => println!("verdict: one clean match"),
        _ => println!("verdict: collision (descend the prefix tree)"),
    }
    Ok(())
}

fn status(c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let i = c.identity(id)?;
    let h = c.health(id)?;
    println!(
        "id {}: model {:#06x}  fw {}  hw {}  cap {:#010x}",
        id.as_byte(),
        i.model,
        i.fw,
        i.hw,
        i.capabilities,
    );
    println!(
        "      fault {:#04x}  dirty {}  trim {}  crc_fail {}  framing_drop {}",
        h.fault_flags, h.config_dirty, h.trim_steps, h.crc_fail_count, h.framing_drop_count,
    );
    Ok(())
}

fn profile(c: &mut Client<NusbPipe>, id: Id, cmd: &ProfileCmd) -> Result<()> {
    match cmd {
        ProfileCmd::Get { slot } => {
            let words = c.read(
                id,
                profile_slot_addr(*slot),
                PROFILE_SPANS_PER_SLOT as u16 * 2,
            )?;
            let mut total = 0u16;
            for pair in words.as_chunks::<2>().0 {
                let (addr, count) = profile_span_split(u16::from_le_bytes(*pair));
                if count != 0 {
                    println!("slot {slot}: {addr:#06x}:{count}");
                    total += count as u16;
                }
            }
            if total == 0 {
                println!("slot {slot}: empty");
                return Ok(());
            }
            let data = c.read_profile(id, *slot)?;
            println!("data ({total} B): {}", hex(&data));
            Ok(())
        }
        ProfileCmd::Set { slot, spans } => {
            let spans: Vec<(u16, u8)> =
                spans.iter().map(|s| parse_span(s)).collect::<Result<_>>()?;
            if spans.len() > PROFILE_SPANS_PER_SLOT {
                bail!("a slot holds at most {PROFILE_SPANS_PER_SLOT} spans");
            }
            let mut data = Vec::with_capacity(PROFILE_SPANS_PER_SLOT * 2);
            for &(addr, count) in &spans {
                data.extend_from_slice(&profile_span_word(addr, count).to_le_bytes());
            }
            data.resize(PROFILE_SPANS_PER_SLOT * 2, 0);
            c.write(id, profile_slot_addr(*slot), &data)?;
            if spans.is_empty() {
                println!("slot {slot}: cleared");
            } else {
                let total: u16 = spans.iter().map(|&(_, c)| c as u16).sum();
                println!("slot {slot}: {} span(s), {total} B", spans.len());
            }
            Ok(())
        }
    }
}

/// Identity read (protocol sec 5.4 front) plus descriptor selection. Prints
/// any advisory note at the call site so the codec stays print-free.
fn select_descriptor<'a>(
    c: &mut Client<NusbPipe>,
    id: Id,
    reg: &'a descriptor::Registry,
) -> Result<&'a descriptor::Descriptor> {
    let b = c.read(id, 0, 4)?;
    if b.len() < 4 {
        bail!("identity read returned {} B, expected 4", b.len());
    }
    let model = u16::from_le_bytes([b[0], b[1]]);
    let fw = b[2];
    let (d, note) = reg.select(model, fw)?;
    if let Some(note) = note {
        println!("{note}");
    }
    Ok(d)
}

fn get(c: &mut Client<NusbPipe>, id: Id, name: &str) -> Result<()> {
    let reg = descriptor::Registry::load()?;
    let d = select_descriptor(c, id, &reg)?;
    let f = d.field(name)?;
    let bytes = c.read(id, f.addr, f.width)?;
    println!(
        "{} = {}  (addr {:#06x}, {} B, raw {})",
        f.name,
        descriptor::decode(f, &bytes)?,
        f.addr,
        f.width,
        hex(&bytes),
    );
    Ok(())
}

fn set(
    c: &mut Client<NusbPipe>,
    id: Id,
    name: &str,
    value: &str,
    hold: bool,
    noreply: bool,
) -> Result<()> {
    let reg = descriptor::Registry::load()?;
    let d = select_descriptor(c, id, &reg)?;
    let f = d.field(name)?;
    let data = descriptor::encode(f, value)?;
    if hold {
        c.write_hold(id, f.addr, &data)?;
        println!(
            "staged {} = {value} at {:#06x} (HOLD; COMMIT applies)",
            f.name, f.addr
        );
    } else if noreply {
        c.write_noreply(id, f.addr, &data)?;
        println!("sent {} = {value} at {:#06x} (NOREPLY)", f.name, f.addr);
    } else {
        c.write(id, f.addr, &data)?;
        println!("wrote {} = {value} at {:#06x}", f.name, f.addr);
    }
    Ok(())
}

fn dump(c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    let reg = descriptor::Registry::load()?;
    let d = select_descriptor(c, id, &reg)?;
    // A status frame carries <= 252 payload bytes; walk the table in chunks
    // and slice each field's bytes out of the flat image.
    const CHUNK: u16 = 252;
    let mut table = vec![0u8; d.table_size as usize];
    let mut addr = 0u16;
    while addr < d.table_size {
        let len = CHUNK.min(d.table_size - addr);
        let bytes = c.read(id, addr, len)?;
        let start = addr as usize;
        table[start..start + bytes.len()].copy_from_slice(&bytes);
        addr += len;
    }
    for f in &d.fields {
        let bytes = &table[f.addr as usize..f.end() as usize];
        println!(
            "{:#06x}  {:<28} {:<2}  {}",
            f.addr,
            f.name,
            f.access.as_str(),
            descriptor::decode(f, bytes)?,
        );
    }
    Ok(())
}

fn main() -> Result<()> {
    let cli = Cli::parse();
    let id = Id::new(cli.id);
    match &cli.cmd {
        Cmd::Info => {
            let c = connect()?;
            let info = c.info();
            println!(
                "osc-adapter: link v{}, {} ticks/us",
                info.version, info.ticks_per_us
            );
            Ok(())
        }
        Cmd::Rails => print_rails(connect()?.rails()?),
        Cmd::Rail3v3 { on } => print_rails(connect()?.set_rail_3v3(*on)?),
        Cmd::Rail5v { on } => print_rails(connect()?.set_rail_5v(*on)?),
        Cmd::Bootloader => {
            let mut c = connect()?;
            c.enter_bootloader()?;
            println!("adapter resets to the WCH bootloader (4348:55e0); use wlink-iap");
            Ok(())
        }
        Cmd::Discover => discover(&cli),
        Cmd::Rescue { set_baud, save } => rescue(set_baud.as_deref(), *save),
        Cmd::Enum { prefix_len, prefix } => {
            enum_probe(&mut connect_bus(&cli)?, *prefix_len, prefix)
        }
        Cmd::Assign { uid, new_id } => {
            let mut c = connect_bus(&cli)?;
            c.assign(&parse_uid(uid)?, Id::new(*new_id))?;
            println!("id {new_id} (volatile until SAVE)");
            Ok(())
        }
        Cmd::Baud { to, ids } => migrate(
            &mut connect_bus(&cli)?,
            &ids_or_default(ids, cli.id),
            parse_rate(to)?,
        ),
        Cmd::Deadline { us, ids } => {
            let mut c = connect_bus(&cli)?;
            for &id in &ids_or_default(ids, cli.id) {
                c.write(
                    id,
                    osc_protocol::table::RESPONSE_DEADLINE_US,
                    &us.to_le_bytes(),
                )?;
                println!("id {}: response_deadline_us = {us}", id.as_byte());
            }
            c.set_response_deadline(*us)?;
            println!("engine mirror set");
            Ok(())
        }
        Cmd::Save => save(&mut connect_bus(&cli)?, id),
        Cmd::Factory => {
            let mut c = connect_bus(&cli)?;
            c.factory(id)?;
            println!(
                "id {}: slots wiped, rebooting to board defaults",
                id.as_byte()
            );
            Ok(())
        }
        Cmd::Reboot => {
            let mut c = connect_bus(&cli)?;
            c.reboot(id)?;
            println!(
                "id {}: rebooting (back at its configured baud)",
                id.as_byte()
            );
            Ok(())
        }
        Cmd::Syntonize {
            trains,
            gap_us,
            gaps,
            ids,
        } => {
            let mut c = connect_bus(&cli)?;
            let traces = c.cal_verify(&ids_or_default(ids, cli.id), *trains, *gap_us, *gaps)?;
            println!("{trains} train(s) x {gaps} gaps x {gap_us} us");
            for t in traces {
                let steps: Vec<String> = t.trims.iter().map(|v| v.to_string()).collect();
                println!(
                    "id {:<3} trim_steps {} ({})",
                    t.id.as_byte(),
                    steps.join(" -> "),
                    if t.converged() { "converged" } else { "MOVING" },
                );
            }
            Ok(())
        }
        Cmd::Ping => {
            let mut c = connect_bus(&cli)?;
            let p = c.ping(id)?;
            println!(
                "id {}  model {:#06x}  fw {}{}",
                id.as_byte(),
                p.model,
                p.fw,
                if p.alert { "  ALERT" } else { "" },
            );
            Ok(())
        }
        Cmd::Status => status(&mut connect_bus(&cli)?, id),
        Cmd::Clear { ids } => {
            let mut c = connect_bus(&cli)?;
            for &id in &ids_or_default(ids, cli.id) {
                c.clear_counters(id)?;
                println!("id {}: counters cleared", id.as_byte());
            }
            Ok(())
        }
        Cmd::Read { addr, len } => {
            let mut c = connect_bus(&cli)?;
            let addr = parse_u16(addr)?;
            let data = c.read(id, addr, *len)?;
            for (i, chunk) in data.chunks(16).enumerate() {
                println!("{:#06x}  {}", addr as usize + i * 16, hex(chunk));
            }
            Ok(())
        }
        Cmd::Write {
            addr,
            data,
            hold,
            noreply,
        } => {
            let mut c = connect_bus(&cli)?;
            let data = parse_hex(data)?;
            let a = parse_u16(addr)?;
            if *hold {
                c.write_hold(id, a, &data)?;
                println!("staged {} B at {addr} (HOLD; COMMIT applies)", data.len());
            } else if *noreply {
                c.write_noreply(id, a, &data)?;
                println!("sent {} B at {addr} (NOREPLY)", data.len());
            } else {
                c.write(id, a, &data)?;
                println!("wrote {} B at {addr}", data.len());
            }
            Ok(())
        }
        Cmd::Gread { addr, count, ids } => {
            let mut c = connect_bus(&cli)?;
            let ids = ids_or_default(ids, cli.id);
            let chain = c.gread(&ids, parse_u16(addr)?, *count)?;
            for s in &chain.statuses {
                println!(
                    "slot {} id {:<3} {:?}{}  {}",
                    s.slot,
                    s.id,
                    s.result,
                    if s.alert { " ALERT" } else { "" },
                    hex(&s.payload),
                );
            }
            match chain.timeout_slot {
                Some(slot) => bail!("chain timed out at slot {slot}"),
                None => Ok(()),
            }
        }
        Cmd::Gwrite {
            addr,
            data,
            ids,
            hold,
        } => {
            let mut c = connect_bus(&cli)?;
            let ids = ids_or_default(ids, cli.id);
            let data = parse_hex(data)?;
            let pairs: Vec<(Id, &[u8])> = ids.iter().map(|&id| (id, data.as_slice())).collect();
            let a = parse_u16(addr)?;
            if *hold {
                c.gwrite_hold(a, data.len() as u8, &pairs)?;
                println!(
                    "staged {} B x {} target(s) at {addr} (HOLD; COMMIT applies)",
                    data.len(),
                    pairs.len(),
                );
            } else {
                c.gwrite(a, data.len() as u8, &pairs)?;
                println!(
                    "sent {} B x {} target(s) at {addr}",
                    data.len(),
                    pairs.len()
                );
            }
            Ok(())
        }
        Cmd::Commit => {
            connect_bus(&cli)?.commit()?;
            println!("committed");
            Ok(())
        }
        Cmd::Profile { cmd } => profile(&mut connect_bus(&cli)?, id, cmd),
        Cmd::Get { field } => get(&mut connect_bus(&cli)?, id, field),
        Cmd::Set {
            field,
            value,
            hold,
            noreply,
        } => set(&mut connect_bus(&cli)?, id, field, value, *hold, *noreply),
        Cmd::Dump => dump(&mut connect_bus(&cli)?, id),
        Cmd::Ident(args) => ident::run(args, cli.baud.clone(), cli.id),
        Cmd::Cal(args) => cal::run(args, cli.baud.clone(), cli.id),
        Cmd::CalReplay(a) => cal::replay::run(a),
    }
}
