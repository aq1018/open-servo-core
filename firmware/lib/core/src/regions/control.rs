use crate::regions::config;
use control_table::{Block, Enum, Section};

/// Control mode. `repr(u8)` so the byte-level commit path round-trips
/// cleanly; validators MUST gate writes to `Mode::ALLOWED` because constructing a
/// `Mode` from an unlisted discriminant is UB.
#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum Mode {
    #[default]
    OpenLoop = 0,
    Current = 1,
    Velocity = 2,
    Position = 3,
}

#[derive(Copy, Clone, Debug, PartialEq, Eq, Default, Enum)]
#[repr(u8)]
pub enum BootMode {
    #[default]
    App = 0,
    Bootloader = 1,
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ControlLifecycle {
    pub torque_enable: bool,
    /// TEL stream gate, deliberately adjacent to `torque_enable`: one 2-byte
    /// write starts motion and capture in the same bus transaction. The
    /// stream runs when enabled AND `tel_mask` != 0.
    pub tel_enable: bool,
    /// TEL frame layout, one bit per field (`tel` module). The le rule
    /// rejects reserved bits; every v1 mask fits the wire budget.
    #[ct_field(le = crate::tel::MASK_ALL)]
    pub tel_mask: u16,
    pub mode: Mode,
    #[ct_field(skip)]
    pub _rsvd_align: u8,
    #[ct_field(le = &config::addr::loop_current::DUTY_MAX_Q15, abs)]
    pub goal_duty: i16,
    /// Phys-validated, soft-clamped: garbage outside the rails rejects; an
    /// out-of-soft goal runs to the soft wall (trajectory clamp) instead of
    /// bouncing the write.
    #[ct_field(
        ge = &config::addr::pos_limits::POS_MIN_PHYS_COUNTS,
        le = &config::addr::pos_limits::POS_MAX_PHYS_COUNTS,
    )]
    pub goal_position: i32,
    pub goal_velocity: i32,
    #[ct_field(le = &config::addr::limits::CURRENT_LIMIT_COUNTS, abs)]
    pub goal_current: i16,
    #[ct_field(skip)]
    pub _rsvd_tail: [u8; 2],
}

#[repr(C)]
#[derive(Copy, Clone, Block)]
pub struct ControlSystem {
    pub boot_mode: BootMode,
}

#[repr(C)]
#[derive(Section)]
#[ct_section(base = crate::regions::CONTROL_BASE_ADDR, size = crate::regions::CONTROL_REGION_SIZE)]
pub struct ControlRegs {
    pub lifecycle: ControlLifecycle,
    pub system: ControlSystem,
    #[ct_section(skip)]
    pub _rsvd_tail: [u8; 107],
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::mem::size_of;

    #[test]
    fn region_fits_declared_size() {
        assert_eq!(
            size_of::<ControlRegs>(),
            crate::regions::CONTROL_REGION_SIZE as usize
        );
    }
}
