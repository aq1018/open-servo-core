#![no_std]
#![feature(sync_unsafe_cell)]

pub mod debug;
pub mod estimator;
pub mod kernel;

/// Firmware version stamped into the identity block (protocol sec 5.4). A
/// table-ABI counter, bumped when a client-visible table ABI change lands;
/// 0 is reserved for an unversioned dev build.
pub const FIRMWARE_VERSION: u8 = 1;

pub mod log;
pub mod math;
pub mod persist;
pub mod regions;
pub mod sensor_frame;
pub mod services;
pub mod shared;
pub mod tel;
pub mod traits;

pub use control_table::{
    Error, Region, RegionStorage, RegionStorageRaw, StagedWrites, ValidationKind,
};
pub use kernel::{Kernel, KernelTiming};
pub use persist::{ConfigStore, StoreError};
pub use regions::config::{BaudRate, ConfigDefaults};
pub use regions::{
    BootMode, CalibKinematics, CalibMotor, CalibRegs, CalibSense, CalibWinding, ConfigCommon,
    ConfigFaultCfg, ConfigFusion, ConfigLimits, ConfigLoopCurrent, ConfigLoopPosition,
    ConfigLoopVelocity, ConfigPosLimits, ConfigRegs, ConfigThermal, ControlLifecycle, ControlRegs,
    ControlSystem, ControlTable, ControlTableCell, DecaySelect, Mode, PotLutBlock, StallResponse,
    TelemetryCommon, TelemetryEstimates, TelemetryIdent, TelemetryMode, TelemetryRegs,
    TelemetrySensors,
};
pub use sensor_frame::SensorFrame;
pub use services::bus::{Dispatcher, Session};
pub use shared::Shared;
pub use tel::{TelSample, TelStream};
pub use traits::{
    Capabilities, ControlIo, DecayMode, Dispatch, Dispatched, Motor, MotorCmd, Reply, Request,
    RequestCtx, SendError, Sensors, Status,
};
