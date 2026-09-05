//! Register addresses the identification pipeline reads and writes,
//! mirrored from descriptors/osc-servo.json. The cross-check test below
//! fails loudly on any ABI drift, so these consts are safe to hardcode.

#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub struct Reg {
    pub addr: u16,
    pub width: u8,
}

const fn reg(addr: u16, width: u8) -> Reg {
    Reg { addr, width }
}

pub mod config {
    use super::{Reg, reg};

    pub const POS_MIN_PHYS_COUNTS: Reg = reg(0x0020, 4);
    pub const POS_MAX_PHYS_COUNTS: Reg = reg(0x0024, 4);
    pub const POS_MIN_SOFT_COUNTS: Reg = reg(0x0028, 4);
    pub const POS_MAX_SOFT_COUNTS: Reg = reg(0x002c, 4);
    pub const I_KP_Q88: Reg = reg(0x0030, 2);
    pub const I_KI_Q412: Reg = reg(0x0032, 2);
    pub const I_KAW_Q412: Reg = reg(0x0034, 2);
    pub const DUTY_MAX_Q15: Reg = reg(0x0036, 2);
    pub const V_KP_Q88: Reg = reg(0x0038, 2);
    pub const V_KI_Q412: Reg = reg(0x003a, 2);
    pub const V_KAW_Q412: Reg = reg(0x003c, 2);
    pub const J_FF_Q88: Reg = reg(0x003e, 2);
    pub const P_KP_Q88: Reg = reg(0x0040, 2);
    pub const POS_DEADBAND_COUNTS: Reg = reg(0x0042, 2);
    pub const CURRENT_LIMIT_COUNTS: Reg = reg(0x0048, 2);
    pub const DRIVE_POLARITY: Reg = reg(0x004b, 1);
    pub const V_UNDERVOLT_COUNTS: Reg = reg(0x0060, 2);
    pub const L1_Q016: Reg = reg(0x0066, 2);
    pub const L2_Q88: Reg = reg(0x0068, 2);
    pub const L3_Q88: Reg = reg(0x006a, 2);
    pub const L_BEMF_Q016: Reg = reg(0x006c, 2);
}

pub mod calib {
    use super::{Reg, reg};

    // PotLutBlock: the first CALIB block. lut_corr is a 110-byte Bytes field
    // written as one blob, so it stays out of the scalar ALL cross-check.
    pub const POT_LUT_RAW_MIN: Reg = reg(0x0080, 2);
    pub const POT_LUT_RAW_MAX: Reg = reg(0x0082, 2);
    pub const POT_LUT_CORR: Reg = reg(0x0084, 110);

    pub const SHUNT_R_MOHM: Reg = reg(0x00f2, 2);
    pub const GAIN_MILLI: Reg = reg(0x00f4, 2);
    pub const VMOTOR_DIV_TOP: Reg = reg(0x00f6, 2);
    pub const VMOTOR_DIV_BOT: Reg = reg(0x00f8, 2);
    pub const VDD_MV: Reg = reg(0x00fa, 2);
    pub const TICK_HZ: Reg = reg(0x00fc, 2);
    pub const I_WINDOW_MIN_TICKS: Reg = reg(0x00fe, 2);
    pub const V_WINDOW_MIN_TICKS: Reg = reg(0x0100, 2);
    pub const R0_Q12: Reg = reg(0x0102, 2);
    pub const T0_CC: Reg = reg(0x0104, 2);
    pub const R_Q12: Reg = reg(0x010c, 2);
    pub const RECIP_KE_Q: Reg = reg(0x010e, 2);
    pub const B_I_Q313: Reg = reg(0x0110, 2);
    pub const FRIC_FC_COUNTS: Reg = reg(0x0112, 2);
    pub const FRIC_FV_Q016: Reg = reg(0x0114, 2);
    pub const FRIC_BREAKAWAY_COUNTS: Reg = reg(0x0116, 2);
    pub const KE_VPC_Q: Reg = reg(0x0118, 2);
    pub const ANGLE_MIN_CDEG: Reg = reg(0x011a, 2);
    pub const ANGLE_MAX_CDEG: Reg = reg(0x011c, 2);
    pub const GEAR_RATIO_CENTI: Reg = reg(0x011e, 2);
}

pub mod control {
    use super::{Reg, reg};

    pub const TORQUE_ENABLE: Reg = reg(0x0180, 1);
    pub const TEL_ENABLE: Reg = reg(0x0181, 1);
    pub const TEL_MASK: Reg = reg(0x0182, 2);
    pub const MODE: Reg = reg(0x0184, 1);
    pub const GOAL_DUTY: Reg = reg(0x0186, 2);
    pub const GOAL_POSITION: Reg = reg(0x0188, 4);
    pub const GOAL_VELOCITY: Reg = reg(0x018c, 4);
    pub const GOAL_CURRENT: Reg = reg(0x0190, 2);
}

pub mod telemetry {
    use super::{Reg, reg};

    pub const FAULT_FLAGS: Reg = reg(0x0200, 1);
    pub const STATUS_FLAGS: Reg = reg(0x0201, 1);
    pub const MODE_ACTIVE: Reg = reg(0x0220, 1);
    pub const FAULT_CODE: Reg = reg(0x0221, 1);
    pub const THETA_HAT_Q16: Reg = reg(0x0224, 4);
    pub const OMEGA_HAT_CPS: Reg = reg(0x0228, 4);
    pub const TAU_D_COUNTS: Reg = reg(0x022c, 2);
    pub const I_LIM_COUNTS: Reg = reg(0x022e, 2);
    pub const T_WINDING_CC: Reg = reg(0x0230, 2);
    pub const VBUS_COUNTS: Reg = reg(0x0232, 2);
    pub const DUTY_APPLIED_Q15: Reg = reg(0x0234, 2);
    pub const OMEGA_BEMF_CPS: Reg = reg(0x0236, 2);
    pub const R_HAT_Q12: Reg = reg(0x0238, 2);
    pub const I_HAT_COUNTS: Reg = reg(0x023a, 2);
    pub const SAMPLE_TICK: Reg = reg(0x023c, 4);
    pub const POS: Reg = reg(0x0240, 2);
    pub const CURRENT: Reg = reg(0x0242, 2);
    pub const CURRENT_TROUGH: Reg = reg(0x0250, 2);
    pub const CURRENT_BIAS_COUNTS: Reg = reg(0x0252, 2);
    pub const I_MEAN_COUNTS: Reg = reg(0x0254, 2);
    pub const I_MIN_COUNTS: Reg = reg(0x0256, 2);
    pub const I_MAX_COUNTS: Reg = reg(0x0258, 2);
    pub const VDIFF_MEAN: Reg = reg(0x025a, 2);
    pub const DUTY_MEAN_Q15: Reg = reg(0x025c, 2);
    pub const AGG_SEQ: Reg = reg(0x025e, 2);
}

/// Every const above with its descriptor field name - the cross-check
/// test's worklist, and a name lookup for reports.
pub const ALL: &[(&str, Reg)] = &[
    ("pos_min_phys_counts", config::POS_MIN_PHYS_COUNTS),
    ("pos_max_phys_counts", config::POS_MAX_PHYS_COUNTS),
    ("pos_min_soft_counts", config::POS_MIN_SOFT_COUNTS),
    ("pos_max_soft_counts", config::POS_MAX_SOFT_COUNTS),
    ("i_kp_q88", config::I_KP_Q88),
    ("i_ki_q412", config::I_KI_Q412),
    ("i_kaw_q412", config::I_KAW_Q412),
    ("duty_max_q15", config::DUTY_MAX_Q15),
    ("v_kp_q88", config::V_KP_Q88),
    ("v_ki_q412", config::V_KI_Q412),
    ("v_kaw_q412", config::V_KAW_Q412),
    ("j_ff_q88", config::J_FF_Q88),
    ("p_kp_q88", config::P_KP_Q88),
    ("pos_deadband_counts", config::POS_DEADBAND_COUNTS),
    ("current_limit_counts", config::CURRENT_LIMIT_COUNTS),
    ("drive_polarity", config::DRIVE_POLARITY),
    ("v_undervolt_counts", config::V_UNDERVOLT_COUNTS),
    ("l1_q016", config::L1_Q016),
    ("l2_q88", config::L2_Q88),
    ("l3_q88", config::L3_Q88),
    ("l_bemf_q016", config::L_BEMF_Q016),
    // lut_corr omitted: a 110-byte Bytes field, not a scalar reg (write_reg
    // and reg_by_name assume width <= 4); raw_min/raw_max are plain u16.
    ("raw_min", calib::POT_LUT_RAW_MIN),
    ("raw_max", calib::POT_LUT_RAW_MAX),
    ("shunt_r_mohm", calib::SHUNT_R_MOHM),
    ("gain_milli", calib::GAIN_MILLI),
    ("vmotor_div_top", calib::VMOTOR_DIV_TOP),
    ("vmotor_div_bot", calib::VMOTOR_DIV_BOT),
    ("vdd_mv", calib::VDD_MV),
    ("tick_hz", calib::TICK_HZ),
    ("i_window_min_ticks", calib::I_WINDOW_MIN_TICKS),
    ("v_window_min_ticks", calib::V_WINDOW_MIN_TICKS),
    ("r0_q12", calib::R0_Q12),
    ("t0_cc", calib::T0_CC),
    ("r_q12", calib::R_Q12),
    ("recip_ke_q", calib::RECIP_KE_Q),
    ("b_i_q313", calib::B_I_Q313),
    ("fric_fc_counts", calib::FRIC_FC_COUNTS),
    ("fric_fv_q016", calib::FRIC_FV_Q016),
    ("fric_breakaway_counts", calib::FRIC_BREAKAWAY_COUNTS),
    ("ke_vpc_q", calib::KE_VPC_Q),
    ("angle_min_cdeg", calib::ANGLE_MIN_CDEG),
    ("angle_max_cdeg", calib::ANGLE_MAX_CDEG),
    ("gear_ratio_centi", calib::GEAR_RATIO_CENTI),
    ("torque_enable", control::TORQUE_ENABLE),
    ("tel_enable", control::TEL_ENABLE),
    ("tel_mask", control::TEL_MASK),
    ("mode", control::MODE),
    ("goal_duty", control::GOAL_DUTY),
    ("goal_position", control::GOAL_POSITION),
    ("goal_velocity", control::GOAL_VELOCITY),
    ("goal_current", control::GOAL_CURRENT),
    ("fault_flags", telemetry::FAULT_FLAGS),
    ("status_flags", telemetry::STATUS_FLAGS),
    ("mode_active", telemetry::MODE_ACTIVE),
    ("fault_code", telemetry::FAULT_CODE),
    ("theta_hat_q16", telemetry::THETA_HAT_Q16),
    ("omega_hat_cps", telemetry::OMEGA_HAT_CPS),
    ("tau_d_counts", telemetry::TAU_D_COUNTS),
    ("i_lim_counts", telemetry::I_LIM_COUNTS),
    ("t_winding_cc", telemetry::T_WINDING_CC),
    ("vbus_counts", telemetry::VBUS_COUNTS),
    ("duty_applied_q15", telemetry::DUTY_APPLIED_Q15),
    ("omega_bemf_cps", telemetry::OMEGA_BEMF_CPS),
    ("r_hat_q12", telemetry::R_HAT_Q12),
    ("i_hat_counts", telemetry::I_HAT_COUNTS),
    ("sample_tick", telemetry::SAMPLE_TICK),
    ("pos", telemetry::POS),
    ("current", telemetry::CURRENT),
    ("current_trough", telemetry::CURRENT_TROUGH),
    ("current_bias_counts", telemetry::CURRENT_BIAS_COUNTS),
    ("i_mean_counts", telemetry::I_MEAN_COUNTS),
    ("i_min_counts", telemetry::I_MIN_COUNTS),
    ("i_max_counts", telemetry::I_MAX_COUNTS),
    ("vdiff_mean", telemetry::VDIFF_MEAN),
    ("duty_mean_q15", telemetry::DUTY_MEAN_Q15),
    ("agg_seq", telemetry::AGG_SEQ),
];

#[cfg(test)]
mod tests {
    use super::ALL;

    #[test]
    fn consts_match_descriptor() {
        let path = concat!(env!("CARGO_MANIFEST_DIR"), "/../descriptors/osc-servo.json");
        let json: serde_json::Value =
            serde_json::from_str(&std::fs::read_to_string(path).expect("descriptor readable"))
                .expect("descriptor parses");
        let fields = json["fields"].as_array().expect("fields array");
        for (name, reg) in ALL {
            let f = fields
                .iter()
                .find(|f| f["name"] == *name)
                .unwrap_or_else(|| panic!("{name} missing from descriptor"));
            assert_eq!(f["addr"].as_u64().unwrap(), reg.addr as u64, "{name} addr");
            assert_eq!(
                f["width"].as_u64().unwrap(),
                reg.width as u64,
                "{name} width"
            );
        }
    }
}
