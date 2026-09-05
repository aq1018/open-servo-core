//! Table snapshot / write-back / rollback. The snapshot captures every
//! field `ident write` can touch (the calib block + the config gain
//! fields) BEFORE any write, so a bad fit is one `ident rollback` away.

use std::collections::BTreeMap;
use std::path::Path;

use anyhow::{Context, Result, bail};
use osc_client::Id;
use osc_client::blocking::Client;
use osc_client::nusb::NusbPipe;
use osc_ident::regs::{ALL, Reg, calib, config};

use super::pump::write_reg;
use crate::ident::params::GainJson;

/// name -> Reg for every writable field the gain set names.
pub(crate) fn reg_by_name(name: &str) -> Option<Reg> {
    ALL.iter().find(|(n, _)| *n == name).map(|(_, r)| *r)
}

/// The write-set superset: everything snapshot/rollback/show handles.
pub(crate) const SNAPSHOT_FIELDS: &[(&str, Reg)] = &[
    ("r_q12", calib::R_Q12),
    ("r0_q12", calib::R0_Q12),
    ("t0_cc", calib::T0_CC),
    ("ke_vpc_q", calib::KE_VPC_Q),
    ("recip_ke_q", calib::RECIP_KE_Q),
    ("b_i_q313", calib::B_I_Q313),
    ("fric_fc_counts", calib::FRIC_FC_COUNTS),
    ("fric_fv_q016", calib::FRIC_FV_Q016),
    ("fric_breakaway_counts", calib::FRIC_BREAKAWAY_COUNTS),
    ("i_kp_q88", config::I_KP_Q88),
    ("i_ki_q412", config::I_KI_Q412),
    ("i_kaw_q412", config::I_KAW_Q412),
    ("v_kp_q88", config::V_KP_Q88),
    ("v_ki_q412", config::V_KI_Q412),
    ("v_kaw_q412", config::V_KAW_Q412),
    ("j_ff_q88", config::J_FF_Q88),
    ("p_kp_q88", config::P_KP_Q88),
    ("pos_deadband_counts", config::POS_DEADBAND_COUNTS),
    ("l1_q016", config::L1_Q016),
    ("l2_q88", config::L2_Q88),
    ("l3_q88", config::L3_Q88),
    ("l_bemf_q016", config::L_BEMF_Q016),
];

pub(crate) fn read_u16(c: &mut Client<NusbPipe>, id: Id, reg: Reg) -> Result<u16> {
    let raw = c.read(id, reg.addr, 2).context("field read")?;
    Ok(u16::from_le_bytes([raw[0], raw[1]]))
}

pub(crate) fn take_snapshot(c: &mut Client<NusbPipe>, id: Id, path: &Path) -> Result<()> {
    let mut map = BTreeMap::new();
    for (name, reg) in SNAPSHOT_FIELDS {
        map.insert(name.to_string(), read_u16(c, id, *reg)?);
    }
    std::fs::write(path, serde_json::to_string_pretty(&map)?)
        .with_context(|| format!("write {}", path.display()))?;
    println!("snapshot: {} fields -> {}", map.len(), path.display());
    Ok(())
}

pub(crate) fn rollback(c: &mut Client<NusbPipe>, id: Id, path: &Path) -> Result<()> {
    let map: BTreeMap<String, u16> = serde_json::from_str(
        &std::fs::read_to_string(path).with_context(|| format!("read {}", path.display()))?,
    )?;
    for (name, raw) in &map {
        let reg = reg_by_name(name)
            .ok_or_else(|| anyhow::anyhow!("{name}: not a known field, snapshot stale?"))?;
        write_reg(c, id, reg, *raw as i32)?;
        let back = read_u16(c, id, reg)?;
        if back != *raw {
            bail!("{name}: wrote {raw}, read back {back}");
        }
    }
    println!("rolled back {} fields", map.len());
    Ok(())
}

/// Write the encoded set, read-back verifying each field.
pub(crate) fn write_gains(c: &mut Client<NusbPipe>, id: Id, gains: &[GainJson]) -> Result<()> {
    for g in gains {
        let reg =
            reg_by_name(&g.name).ok_or_else(|| anyhow::anyhow!("{}: not a known field", g.name))?;
        if g.saturated {
            println!("  {} SATURATED at {} - writing the clamp", g.name, g.raw);
        }
        write_reg(c, id, reg, g.raw as i32)?;
        let back = read_u16(c, id, reg)?;
        if back != g.raw {
            bail!("{}: wrote {}, read back {back}", g.name, g.raw);
        }
    }
    println!("wrote {} fields, all verified", gains.len());
    Ok(())
}

pub(crate) fn show(c: &mut Client<NusbPipe>, id: Id) -> Result<()> {
    for (name, reg) in SNAPSHOT_FIELDS {
        println!("{name:24} {}", read_u16(c, id, *reg)?);
    }
    Ok(())
}
