use control_table::{Block, Enum, Section};

// Wire vocabulary, hoisted to the foundation crate so the host column can
// name it (grid law 2). The table field below stays a raw index because the
// `Enum` derive impls a control-table trait -- orphan-rule-bound to whichever
// crate defines the type.
pub use osc_protocol::wire::{BaudRate, DEFAULT_RESPONSE_DEADLINE_US};

/// Stall policy: Fault latches STALL; Yield folds the current limit.
/// `repr(u8)`; constructing from an unlisted discriminant is UB, so
/// validators MUST gate writes to `StallResponse::ALLOWED`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum StallResponse {
    #[default]
    Fault = 0,
    Yield = 1,
}

/// Off-window decay for OpenLoop drive; closed-loop modes force Slow.
/// `repr(u8)`; same UB gate as `StallResponse`.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum DecaySelect {
    #[default]
    Slow = 0,
    Fast = 1,
}

/// protocol sec 5.4 CONFIG-COMMON block: identity RO at the region front,
/// comms RW at 0x010, reserved bytes between and behind. Model-specific
/// config starts at 0x020.
#[repr(C)]
#[derive(Copy, Clone, Block)]
#[ct_block(hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ConfigCommon {
    #[ct_field(access = ro)]
    pub model_number: u16,
    #[ct_field(access = ro)]
    pub firmware_version: u8,
    #[ct_field(access = ro)]
    pub hardware_revision: u8,
    #[ct_field(access = ro)]
    pub capability_flags: u32,
    #[ct_field(skip)]
    pub _rsvd_identity: [u8; 8],
    #[ct_field(ge = 1u8, le = 249u8, hook = on_id_write)]
    pub id: u8,
    // `BaudRate` index; le gate = the enum ceiling. Zero (0.5M, the rescue
    // floor) keeps the const image all-zero so SHARED lands in .bss; the
    // operational default is `ConfigDefaults.baud`, seeded at boot.
    #[ct_field(le = 3u8, hook = on_baud_rate_idx_write)]
    pub baud_rate_idx: u8,
    #[ct_field(hook = on_response_deadline_us_write)]
    pub response_deadline_us: u16,
    #[ct_field(skip)]
    pub _rsvd_tail: [u8; 12],
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigPosLimits {
    #[ct_field(lt = &addr::pos_limits::POS_MAX_PHYS_COUNTS)]
    pub pos_min_phys_counts: i32,
    #[ct_field(gt = &addr::pos_limits::POS_MIN_PHYS_COUNTS)]
    pub pos_max_phys_counts: i32,
    #[ct_field(
        ge = &addr::pos_limits::POS_MIN_PHYS_COUNTS,
        le = &addr::pos_limits::POS_MAX_PHYS_COUNTS,
        lt = &addr::pos_limits::POS_MAX_SOFT_COUNTS,
    )]
    pub pos_min_soft_counts: i32,
    #[ct_field(
        ge = &addr::pos_limits::POS_MIN_PHYS_COUNTS,
        le = &addr::pos_limits::POS_MAX_PHYS_COUNTS,
        gt = &addr::pos_limits::POS_MIN_SOFT_COUNTS,
    )]
    pub pos_max_soft_counts: i32,
}

/// Current PI gains, voltage-domain clamp + back-calc anti-windup.
/// Gains carry i_/v_/p_ prefixes: descriptor field names are table-flat.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopCurrent {
    pub i_kp_q88: u16,
    pub i_ki_q412: u16,
    pub i_kaw_q412: u16,
    #[ct_field(le = 32767u16)]
    pub duty_max_q15: u16,
}

// Full-scale duty by default; gains stay zero so the loop boots inert.
pub const DEFAULT_DUTY_MAX_Q15: u16 = 32767;

/// Velocity PI gains + inertia feedforward.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopVelocity {
    pub v_kp_q88: u16,
    pub v_ki_q412: u16,
    pub v_kaw_q412: u16,
    pub j_ff_q88: u16,
}

/// Position P gain, anti-hunt hold deadband, trajectory limits.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLoopPosition {
    pub p_kp_q88: u16,
    /// 0 = hold disabled. Hold coasts on position rest alone (position.rs).
    pub pos_deadband_counts: u16,
    pub velocity_limit_cps: u16,
    /// c/s per medium tick.
    pub accel_limit_q88: u16,
}

// Trajectory limits are clamps, not gains: at 0 the profile pins omega_ref
// to 0 and velocity/position modes are inert even with live gains, so they
// get real defaults (bench-validated on the SG90 rig: 1500 c/s, 15 c/s per
// medium tick = 0 -> 1500 in 50 ms).
pub const DEFAULT_VELOCITY_LIMIT_CPS: u16 = 1500;
pub const DEFAULT_ACCEL_LIMIT_Q88: u16 = 3840;

// Anti-hunt hold: at 0 the park never engages and the position loop chases
// pot noise at rest (audible shake on the SG90 rig). Modest pot-noise-scale
// backstop; identification refines it from the measured sigma_theta.
pub const DEFAULT_POS_DEADBAND_COUNTS: u16 = 12;

/// Current ceiling, stall policy, overcurrent trip window.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigLimits {
    pub current_limit_counts: u16,
    pub stall_response: StallResponse,
    /// true = positive duty increases position counts.
    pub drive_polarity: bool,
    pub stall_omega_max_cps: u16,
    pub stall_time_ms: u16,
    pub stall_yield_counts: u16,
    pub stall_release_counts: u16,
    /// Collision trip: |tau_d| above this is model-unexplained torque.
    pub stall_tau_trip_counts: u16,
    pub oc_trip_counts: u16,
    pub oc_trip_ticks: u8,
    pub openloop_decay: DecaySelect,
}

// Permissive-safe SG90-class limits: core-owned policy seeded at boot,
// host-tunable per rig.
pub const DEFAULT_CURRENT_LIMIT_COUNTS: u16 = 1200;
pub const DEFAULT_DRIVE_POLARITY: bool = true;
pub const DEFAULT_STALL_OMEGA_MAX_CPS: u16 = 500;
pub const DEFAULT_STALL_TIME_MS: u16 = 500;
pub const DEFAULT_STALL_YIELD_COUNTS: u16 = 300;
pub const DEFAULT_STALL_RELEASE_COUNTS: u16 = 150;
pub const DEFAULT_STALL_TAU_TRIP_COUNTS: u16 = 1200;
pub const DEFAULT_OC_TRIP_COUNTS: u16 = 2400;
pub const DEFAULT_OC_TRIP_TICKS: u8 = 8;

/// Thermal derate/cutoff, undervolt floor, winding-R estimator gates.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigThermal {
    pub derate_start_cc: i16,
    #[ct_field(gt = &addr::thermal::RECOVER_CC)]
    pub cutoff_cc: i16,
    #[ct_field(lt = &addr::thermal::CUTOFF_CC)]
    pub recover_cc: i16,
    pub v_undervolt_counts: u16,
    pub rtherm_i_min_counts: u16,
    pub rtherm_omega_max_cps: u16,
}

// 80C derate onset / 100C cutoff / 90C recover; undervolt below the 2S
// brown-out floor in vbus counts.
pub const DEFAULT_DERATE_START_CC: i16 = 8000;
pub const DEFAULT_CUTOFF_CC: i16 = 10000;
pub const DEFAULT_RECOVER_CC: i16 = 9000;
pub const DEFAULT_V_UNDERVOLT_COUNTS: u16 = 2200;
pub const DEFAULT_RTHERM_I_MIN_COUNTS: u16 = 300;
pub const DEFAULT_RTHERM_OMEGA_MAX_CPS: u16 = 400;

/// Fusion observer correction gains; l_bemf 0 = bemf blend off.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigFusion {
    pub l1_q016: u16,
    pub l2_q88: u16,
    pub l3_q88: u16,
    pub l_bemf_q016: u16,
}

/// Fault thresholds: position-error window, sensor-delta screen.
#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ConfigFaultCfg {
    pub pos_error_counts: u16,
    pub pos_error_time_ms: u16,
    pub sensor_delta_max: u16,
    pub sensor_bad_count: u8,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
}

// Wide screens so healthy motion never trips; tighten per application.
pub const DEFAULT_POS_ERROR_COUNTS: u16 = 400;
pub const DEFAULT_POS_ERROR_TIME_MS: u16 = 500;
pub const DEFAULT_SENSOR_DELTA_MAX: u16 = 256;
pub const DEFAULT_SENSOR_BAD_COUNT: u8 = 4;

/// Config section: always writable (normal field validation applies), volatile
/// until `MGMT SAVE` persists it -- SAVE is the only torque-gated operation
/// (osc-native sec 9.4 separates write from persistence).
#[repr(C)]
#[derive(Section)]
#[ct_section(
    base = crate::regions::CONFIG_BASE_ADDR,
    size = crate::regions::CONFIG_REGION_SIZE,
    hooks = crate::regions::hooks::ControlTableHookEvents,
)]
pub struct ConfigRegs {
    pub common: ConfigCommon,
    pub pos_limits: ConfigPosLimits,
    pub loop_current: ConfigLoopCurrent,
    pub loop_velocity: ConfigLoopVelocity,
    pub loop_position: ConfigLoopPosition,
    pub limits: ConfigLimits,
    pub thermal: ConfigThermal,
    pub fusion: ConfigFusion,
    pub fault_cfg: ConfigFaultCfg,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 10],
}

/// Boot-time seed for `ControlTable.config`; stamped pre-IRQ, then host-owned.
#[derive(Copy, Clone, Debug, Default)]
pub struct ConfigDefaults {
    pub pos_min_phys_counts: i32,
    pub pos_max_phys_counts: i32,
    pub id: u8,
    pub baud: BaudRate,
    pub response_deadline_us: u16,
}
