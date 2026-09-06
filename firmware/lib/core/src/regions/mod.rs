//! Host-visible register state as a flat 1024-byte map. Sections carve
//! contiguous byte ranges; every address in `[0, 1024)` reads back (reserved
//! and skip bytes read as zero). Writes to non-writable bytes fail with
//! `AccessError`; addresses past the map end fail with `DataRange`.
//!
//!   CONFIG    0x000..0x080  (128 B) -- persistent via MGMT SAVE
//!   CALIB     0x080..0x180  (256 B) -- persistence deferred (unused today)
//!   CONTROL   0x180..0x200  (128 B) -- RW volatile
//!   TELEMETRY 0x200..0x280  (128 B) -- RO from host
//!   PROFILE   0x280..0x2C0  ( 64 B) -- read-profile span words (sec 5.2)
//!   (reserved 0x2C0..0x400  320 B)
//!
//! Owners go through `RegionStorage::with`/`with_mut` on the storage cell.
//! Cross-domain readers that must avoid forming `&T` (aliasing-sensitive
//! paths) use raw-pointer reads via `RegionStorageRaw::region_ptr`.

pub mod calib;
pub mod config;
pub mod control;
pub(crate) mod hooks;
pub mod profile;
pub mod telemetry;

pub use calib::{CalibKinematics, CalibMotor, CalibRegs, CalibSense, CalibWinding, PotLutBlock};
pub use config::{
    BaudRate, ConfigCommon, ConfigFaultCfg, ConfigFusion, ConfigLimits, ConfigLoopCurrent,
    ConfigLoopPosition, ConfigLoopVelocity, ConfigPosLimits, ConfigRegs, ConfigThermal,
    DecaySelect, StallResponse,
};
pub use control::{BootMode, ControlLifecycle, ControlRegs, ControlSystem, Mode};
pub use profile::{PROFILE_SLOTS, ProfileRegs, ProfileSlots, SPANS_PER_SLOT};
pub use telemetry::{
    TelemetryCommon, TelemetryEstimates, TelemetryIdent, TelemetryMode, TelemetryRegs,
    TelemetrySensors,
};

use crate::regions::config::ConfigDefaults;
use control_table::{RegionStorage, Table};

pub const CONFIG_REGION_SIZE: u16 = 128;
pub const CALIB_REGION_SIZE: u16 = 256;
pub const CONTROL_REGION_SIZE: u16 = 128;
pub const TELEMETRY_REGION_SIZE: u16 = 128;
pub const PROFILE_REGION_SIZE: u16 = 64;

pub const CONFIG_BASE_ADDR: u16 = 0x000;
pub const CALIB_BASE_ADDR: u16 = 0x080;
pub const CONTROL_BASE_ADDR: u16 = 0x180;
pub const TELEMETRY_BASE_ADDR: u16 = 0x200;
pub const PROFILE_BASE_ADDR: u16 = 0x280;

#[repr(C)]
#[derive(Table)]
#[ct_table(size = 1024, hooks = crate::regions::hooks::ControlTableHookEvents)]
pub struct ControlTable {
    pub config: ConfigRegs,
    pub calib: CalibRegs,
    pub control: ControlRegs,
    pub telemetry: TelemetryRegs,
    pub profile: ProfileRegs,
    #[ct_table(skip)]
    _rsvd: [u8; 320],
}

impl ControlTableCell {
    /// Soft limits init to physical limits per control-table doc.
    /// Caller must be sole writer (install-time, pre-IRQ).
    pub fn seed_config_defaults(&self, defaults: &ConfigDefaults) {
        crate::log::debug!(
            "seed CONFIG: phys=[{}, {}] counts  id={}  baud_idx={}",
            defaults.pos_min_phys_counts,
            defaults.pos_max_phys_counts,
            defaults.id,
            defaults.baud.as_idx(),
        );
        // SAFETY: install-time, pre-IRQ, sole writer.
        self.with_mut(|t| {
            let cfg = &mut t.config;
            cfg.pos_limits.pos_min_phys_counts = defaults.pos_min_phys_counts;
            cfg.pos_limits.pos_max_phys_counts = defaults.pos_max_phys_counts;
            cfg.pos_limits.pos_min_soft_counts = defaults.pos_min_phys_counts;
            cfg.pos_limits.pos_max_soft_counts = defaults.pos_max_phys_counts;
            cfg.common.id = defaults.id;
            cfg.common.baud_rate_idx = defaults.baud.as_idx();
            cfg.common.response_deadline_us = defaults.response_deadline_us;
            // Core-owned safe defaults; gains stay at their zero seed so
            // every loop boots inert. Enum defaults are discriminant 0.
            // Trajectory limits are clamps, not gains - zero would pin the
            // profile at 0 and keep the loops inert even once gains land.
            cfg.loop_current.duty_max_q15 = config::DEFAULT_DUTY_MAX_Q15;
            cfg.loop_position.velocity_limit_cps = config::DEFAULT_VELOCITY_LIMIT_CPS;
            cfg.loop_position.accel_limit_q88 = config::DEFAULT_ACCEL_LIMIT_Q88;
            cfg.loop_position.pos_deadband_counts = config::DEFAULT_POS_DEADBAND_COUNTS;
            cfg.limits.current_limit_counts = config::DEFAULT_CURRENT_LIMIT_COUNTS;
            cfg.limits.drive_polarity = config::DEFAULT_DRIVE_POLARITY;
            cfg.limits.stall_omega_max_cps = config::DEFAULT_STALL_OMEGA_MAX_CPS;
            cfg.limits.stall_time_ms = config::DEFAULT_STALL_TIME_MS;
            cfg.limits.stall_yield_counts = config::DEFAULT_STALL_YIELD_COUNTS;
            cfg.limits.stall_release_counts = config::DEFAULT_STALL_RELEASE_COUNTS;
            cfg.limits.stall_tau_trip_counts = config::DEFAULT_STALL_TAU_TRIP_COUNTS;
            cfg.limits.oc_trip_counts = config::DEFAULT_OC_TRIP_COUNTS;
            cfg.limits.oc_trip_ticks = config::DEFAULT_OC_TRIP_TICKS;
            cfg.thermal.derate_start_cc = config::DEFAULT_DERATE_START_CC;
            cfg.thermal.cutoff_cc = config::DEFAULT_CUTOFF_CC;
            cfg.thermal.recover_cc = config::DEFAULT_RECOVER_CC;
            cfg.thermal.v_undervolt_counts = config::DEFAULT_V_UNDERVOLT_COUNTS;
            cfg.thermal.rtherm_i_min_counts = config::DEFAULT_RTHERM_I_MIN_COUNTS;
            cfg.thermal.rtherm_omega_max_cps = config::DEFAULT_RTHERM_OMEGA_MAX_CPS;
            cfg.fault_cfg.pos_error_counts = config::DEFAULT_POS_ERROR_COUNTS;
            cfg.fault_cfg.pos_error_time_ms = config::DEFAULT_POS_ERROR_TIME_MS;
            cfg.fault_cfg.sensor_delta_max = config::DEFAULT_SENSOR_DELTA_MAX;
            cfg.fault_cfg.sensor_bad_count = config::DEFAULT_SENSOR_BAD_COUNT;
        });
    }

    /// Stamp the RO identity block (protocol sec 5.4). `firmware_version` is
    /// not a caller argument: it is the compiled-in `crate::FIRMWARE_VERSION`
    /// fact of this build, so a board cannot claim a version it is not running.
    /// Caller must be sole writer (install-time, pre-IRQ).
    pub fn seed_identity(&self, model: u16, hw_rev: u8) {
        // SAFETY: install-time, pre-IRQ, sole writer.
        self.with_mut(|t| {
            let common = &mut t.config.common;
            common.model_number = model;
            common.hardware_revision = hw_rev;
            common.firmware_version = crate::FIRMWARE_VERSION;
        });
    }

    /// Stamp the RO sense primitives the host converts raw counts with
    /// (protocol sec 5.5) -- board facts, not host-writable state.
    /// Caller must be sole writer (install-time, pre-IRQ).
    pub fn seed_calib_sense(&self, sense: &CalibSense) {
        // SAFETY: install-time, pre-IRQ, sole writer.
        self.with_mut(|t| t.calib.sense = *sense);
    }

    /// Stamp the boot-measured zero-current output of the sense chain, in raw
    /// ADC counts -- the offset every `sensors.current` reading is relative to.
    /// Caller must be sole writer (install-time, pre-IRQ).
    pub fn seed_current_bias(&self, counts: u16) {
        crate::log::debug!("seed current bias: {} counts", counts);
        // SAFETY: install-time, pre-IRQ, sole writer.
        self.with_mut(|t| t.telemetry.sensors.current_bias_counts = counts);
    }
}

#[cfg(test)]
mod tests {
    use osc_protocol::table;

    use super::ControlTable;
    use super::config::addr as config_addr;
    use super::telemetry::addr as telemetry_addr;
    use control_table::RegionStorage;
    use control_table::descriptor::{EnumVariant, FieldKind};

    #[test]
    fn seed_identity_stamps_the_block() {
        let table = crate::ControlTableCell::new();
        table.seed_identity(0x0101, 3);
        table.with(|t| {
            assert_eq!(t.config.common.model_number, 0x0101);
            assert_eq!(t.config.common.hardware_revision, 3);
            assert_eq!(t.config.common.firmware_version, crate::FIRMWARE_VERSION);
        });
    }

    /// Pins the struct layout to the protocol sec 5.4 common register block:
    /// every generated field address equals its protocol const, and the
    /// model-specific space begins exactly at each block's end.
    #[test]
    fn common_register_block_offsets_match_the_protocol() {
        assert_eq!(config_addr::common::MODEL_NUMBER, table::MODEL_NUMBER);
        assert_eq!(
            config_addr::common::FIRMWARE_VERSION,
            table::FIRMWARE_VERSION
        );
        assert_eq!(
            config_addr::common::HARDWARE_REVISION,
            table::HARDWARE_REVISION
        );
        assert_eq!(
            config_addr::common::CAPABILITY_FLAGS,
            table::CAPABILITY_FLAGS
        );
        assert_eq!(config_addr::common::ID, table::ID);
        assert_eq!(config_addr::common::BAUD_RATE_IDX, table::BAUD_RATE_IDX);
        assert_eq!(
            config_addr::common::RESPONSE_DEADLINE_US,
            table::RESPONSE_DEADLINE_US
        );
        assert_eq!(
            config_addr::pos_limits::POS_MIN_PHYS_COUNTS,
            table::CONFIG_COMMON_END
        );

        assert_eq!(telemetry_addr::common::FAULT_FLAGS, table::FAULT_FLAGS);
        assert_eq!(telemetry_addr::common::STATUS_FLAGS, table::STATUS_FLAGS);
        assert_eq!(telemetry_addr::common::TRIM_STEPS, table::TRIM_STEPS);
        assert_eq!(
            telemetry_addr::common::CRC_FAIL_COUNT,
            table::CRC_FAIL_COUNT
        );
        assert_eq!(
            telemetry_addr::common::FRAMING_DROP_COUNT,
            table::FRAMING_DROP_COUNT
        );
        assert_eq!(
            telemetry_addr::mode::MODE_ACTIVE,
            table::TELEMETRY_COMMON_END
        );
    }

    /// Pins the exported field descriptors against the protocol consts: names
    /// unique, addresses ascending, and the load-bearing identity/enum/bounds
    /// facts a device-description exporter would rely on.
    #[test]
    fn field_descriptors_match_the_protocol() {
        let f = ControlTable::FIELDS;
        let by = |n: &str| f.iter().find(|d| d.name == n).unwrap();

        // Names unique, addresses strictly ascending.
        for (i, d) in f.iter().enumerate() {
            assert!(
                f[i + 1..].iter().all(|o| o.name != d.name),
                "duplicate field name {}",
                d.name
            );
        }
        for w in f.windows(2) {
            assert!(
                w[0].addr < w[1].addr,
                "addrs not ascending at {}",
                w[1].name
            );
        }

        let id = by("id");
        assert_eq!(id.addr, table::ID);
        assert_eq!(id.width, 1);
        assert!(id.writable);
        assert_eq!(id.min, Some(1));
        assert_eq!(id.max, Some(249));

        let model = by("model_number");
        assert_eq!(model.addr, table::MODEL_NUMBER);
        assert_eq!(model.width, 2);
        assert!(!model.writable);
        assert_eq!(model.kind, FieldKind::UInt);

        assert_eq!(
            by("stall_response").kind,
            FieldKind::Enum(&[
                EnumVariant {
                    name: "Fault",
                    value: 0
                },
                EnumVariant {
                    name: "Yield",
                    value: 1
                },
            ])
        );

        // goal_position's bounds are register-RHS (phys limits); they must not
        // export as scalar bounds.
        let goal = by("goal_position");
        assert!(goal.writable);
        assert_eq!((goal.min, goal.max), (None, None));

        let lut = by("lut_corr");
        assert_eq!(lut.kind, FieldKind::Bytes);
        assert_eq!(lut.width, 110);
    }

    /// Pins the profile region to its protocol sec 5.4 address pin and its
    /// sec 5.2 slot geometry.
    #[test]
    fn profile_region_matches_the_protocol() {
        use super::profile;
        assert_eq!(super::PROFILE_BASE_ADDR, table::PROFILE_START);
        assert_eq!(
            super::PROFILE_BASE_ADDR + super::PROFILE_REGION_SIZE,
            table::PROFILE_END
        );
        assert_eq!(profile::PROFILE_SLOTS, table::PROFILE_SLOTS);
        assert_eq!(profile::SPANS_PER_SLOT, table::PROFILE_SPANS_PER_SLOT);
        assert_eq!(
            profile::span_word(0x208, 63),
            table::profile_span_word(0x208, 63)
        );
    }
}
