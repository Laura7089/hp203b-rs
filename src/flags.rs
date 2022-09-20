use bitflags::bitflags;
use embedded_hal::i2c::blocking::I2c;

#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug)]
pub enum FlagRegister {
    /// Enable or disable interrupts
    INT_EN = 0x0B,
    /// Enable or disable interrupt output on the `INT1` pin
    INT_CFG = 0x0C,
    /// Interrupts and other outgoing flags
    ///
    /// This register is read-only.
    INT_SRC = 0x0D,
    /// Direction of window/traversal events
    ///
    /// 1 = high for `WIN` interrupts, or low -> high for `TRAV` interrupts.
    /// 0 = low for `WIN` interrupts, or high -> low for `TRAV` interrupts.
    ///
    /// This register is read-only.
    /// See the datasheet for more information.
    INT_DIR = 0x0E,
    PARA = 0x0F,
}

pub(crate) const INT_SRC_VARIANTS: [INT_SRC; 6] = [
    INT_SRC::PA_RDY,
    INT_SRC::T_RDY,
    INT_SRC::PA_TRAV,
    INT_SRC::T_TRAV,
    INT_SRC::PA_WIN,
    INT_SRC::T_WIN,
];

bitflags! {
    /// Flags in [`Register8::INT_SRC`]
    #[allow(non_camel_case_types)]
    pub struct INT_SRC: u8 {
        const TH_ERR = 0b1000_0000;
        /// Device ready bit
        const DEV_RDY = 0b0100_0000;
        /// Pressure or altitude measurement ready to read
        const PA_RDY = 0b0010_0000;
        /// Temperature measurement ready to read
        const T_RDY = 0b0001_0000;
        /// Pressure or altitude traversed the middle threshold
        const PA_TRAV = 0b0000_1000;
        /// Temperature traversed the middle threshold
        const T_TRAV = 0b0000_0100;
        /// Pressure or altitude is outside of the lower and upper bounds
        const PA_WIN = 0b0000_0010;
        /// Temperature is outside of the lower and upper bounds
        const T_WIN = 0b0000_0001;
    }

    /// Flags in [`Register8::INT_EN`]
    ///
    /// Enable or disable individual interrupts.
    #[allow(non_camel_case_types)]
    pub struct INT_EN: u8 {
        const PA_RDY_EN = 0b0010_0000;
        const T_RDY_EN = 0b0001_0000;
        const RDY_EN = Self::PA_RDY_EN.bits | Self::T_RDY_EN.bits;
        const PA_TRAV_EN = 0b0000_1000;
        const T_TRAV_EN = 0b0000_0100;
        const PA_WIN_EN = 0b0000_0010;
        const T_WIN_EN = 0b0000_0001;
    }

    /// Flags in [`Register8::INT_CFG`]
    ///
    /// `*_CFG` values denote whether that interrupt will set the `INT1` pin.
    #[allow(non_camel_case_types)]
    pub struct INT_CFG: u8 {
        /// Pressure/Altitude selection
        ///
        /// State | Effect
        /// ---|---
        /// Enabled | All `PA_*` items refer to altitude
        /// Disabled | All `PA_*` items refer to pressure
        const PA_MODE = 0b0100_0000;
        const PA_RDY_CFG = 0b0010_0000;
        const T_RDY_CFG = 0b0001_0000;
        const PA_TRAV_CFG = 0b0000_1000;
        const T_TRAV_CFG = 0b0000_0100;
        const PA_WIN_CFG = 0b0000_0010;
        const T_WIN_CFG = 0b0000_0001;
    }


    /// Flags in [`Register8::INT_DIR`]
    ///
    /// Check details of traversal or window interrupts.
    ///
    /// When windows interrupts happen, `WIN` flags indicate that the temperature or
    /// value is above the window if they are set, otherwise it is below.
    ///
    /// When traversal interrupts happen, `TRAV` flags indicate that the value is rising from
    /// low to high, otherwise it is falling from high to low.
    #[allow(non_camel_case_types)]
    pub struct INT_DIR: u8 {
        const CMPS_EN = 0b1000_0000;
        const P_TRAV_DIR = 0b0000_1000;
        const T_TRAV_DIR = 0b0000_0100;
        const P_WIN_DIR = 0b0000_0010;
        const T_WIN_DIR = 0b0000_0001;
    }

    #[allow(non_camel_case_types)]
    pub struct PARA: u8 {
        const CMPS_EN = 0b1000_0000;
    }
}

macro_rules! getter {
    ($funcname:ident -> $reg:expr , $flags:ty) => {
        fn $funcname(&mut self) -> Result<$flags, I::Error> {
            Ok(<$flags>::from_bits_truncate(self.read_flags($reg)?))
        }
    };
}
macro_rules! setter {
    ($funcname:ident -> $reg:expr , $flags:ty) => {
        fn $funcname(&mut self, val: $flags) -> Result<(), I::Error> {
            self.write_flags($reg, val.bits())
        }
    };
}

// TODO: merge this with `Registers`?
pub trait Flags<I: I2c> {
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error>;
    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error>;

    getter! { get_interrupts -> FlagRegister::INT_SRC, INT_SRC }
    getter! { get_interrupt_extras -> FlagRegister::INT_DIR, INT_DIR }
    getter! { get_interrupts_enabled -> FlagRegister::INT_EN, INT_EN }
    getter! { get_interrupts_pinout -> FlagRegister::INT_CFG, INT_CFG }
    getter! { para -> FlagRegister::PARA, PARA }

    setter! { set_interrupts_enabled -> FlagRegister::INT_EN, INT_EN }
    setter! { set_interrupts_pinout -> FlagRegister::INT_CFG, INT_CFG }
    setter! { set_para -> FlagRegister::PARA, PARA }
}

impl<I: I2c, T> Flags<I> for T
where
    T: super::Registers<I>,
{
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error> {
        self.read_raw(reg as u8)
    }

    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error> {
        self.write_raw(reg as u8, val)
    }
}
