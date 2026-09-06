use control_table::{Block, Section};

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct PotLutBlock {
    pub raw_min: u16,
    pub raw_max: u16,
    /// Corrections vs the identity ramp raw_min..raw_max; all-zero = identity.
    pub lut_corr: [i16; 55],
}

/// Sense-chain primitives the host converts raw counts with (protocol sec
/// 5.5): divider legs in ohms, amplifier gain x1000. `vdd_mv` is the
/// host-measured VDD at the chip pin -- the v006 ADC reference is VDD itself.
/// `tick_hz` is stamped at install from the same chip constant that programs
/// TIM1; the window floors are board data in TIM1 ticks.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibSense {
    #[ct_field(access = ro)]
    pub shunt_r_mohm: u16,
    #[ct_field(access = ro)]
    pub gain_milli: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_top: u16,
    #[ct_field(access = ro)]
    pub vmotor_div_bot: u16,
    pub vdd_mv: u16,
    #[ct_field(access = ro)]
    pub tick_hz: u16,
    #[ct_field(access = ro)]
    pub i_window_min_ticks: u16,
    #[ct_field(access = ro)]
    pub v_window_min_ticks: u16,
}

/// Winding-resistance thermometry anchor, firmware-shaped: `r0_q12` is the
/// cold winding resistance in vcounts-per-ccount Q4.12 (host computes it from
/// its measurement), `t0_cc` the ambient the user entered for it (centi-degC),
/// `k_r2t_q88` the R-to-T slope, `mu_q016` the LMS step.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibWinding {
    pub r0_q12: u16,
    pub t0_cc: i16,
    pub k_r2t_q88: u16,
    pub mu_q016: u16,
}

/// Motor identification: `ke_uvs_per_rad` is the host-facing record; the rest
/// are firmware-shaped -- bemf subtract R (Q4.12), reciprocal Ke (c/s per
/// vcount Q6.10, estimator::bemf convention), fusion current gain (bakes
/// Ts/J, Q3.13: rig B runs ~3.4 so Q0.16 saturated), the friction model
/// (Coulomb ccounts, viscous Q0.16, breakaway
/// ccounts), and forward Ke (vcounts per c/s Q4.12, the current loop's bemf
/// decoupling feedforward). Forward and reciprocal Ke are both host-written:
/// the chip has no divide to derive one from the other. SG90 scale ~0.28
/// vcounts per c/s (~1150 stored); the u16 caps at 16, 57x headroom.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibMotor {
    pub ke_uvs_per_rad: u16,
    pub r_q12: u16,
    pub recip_ke_q: u16,
    pub b_i_q313: u16,
    pub fric_fc_counts: u16,
    pub fric_fv_q016: u16,
    pub fric_breakaway_counts: u16,
    pub ke_vpc_q: u16,
}

/// Count<->angle scale and gearing, pure host-facing metadata: firmware never
/// reads it, the ISR stays in counts. Angles are centi-degrees at the pot LUT
/// endpoints raw_min/raw_max (which equal pos_min/max_phys); `gear_ratio_centi`
/// is motor revs per output rev x100. All-zero = unset.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct CalibKinematics {
    pub angle_min_cdeg: i16,
    pub angle_max_cdeg: i16,
    pub gear_ratio_centi: u16,
}

/// Calibration section: always writable (normal field validation applies),
/// volatile until persisted -- persistence is SAVE's job, not a write gate.
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CALIB_BASE_ADDR,
    size = crate::regions::CALIB_REGION_SIZE,
)]
pub struct CalibRegs {
    pub pot_lut: PotLutBlock,
    pub sense: CalibSense,
    pub winding: CalibWinding,
    pub motor: CalibMotor,
    pub kinematics: CalibKinematics,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 96],
}
