#![allow(non_camel_case_types)]

use bitflags::bitflags;
#[cfg(feature = "defmt")]
use defmt::trace;
use embedded_hal::i2c::I2c;

#[allow(clippy::upper_case_acronyms)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct INT_SRC: u8 {
        /// Threshold error - bounds are wrong
        const TH_ERR = 0b1000_0000;
        /// Device ready bit
        const DEV_RDY = 0b0100_0000;
        /// Pressure or altitude measurement ready to read
        const PA_RDY = 0b0010_0000;
        /// Temperature measurement ready to read
        const T_RDY = 0b0001_0000;
        /// All ready
        const READ_RDY = Self::PA_RDY.bits | Self::T_RDY.bits;
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
    pub struct INT_DIR: u8 {
        const CMPS_EN = 0b1000_0000;
        const P_TRAV_DIR = 0b0000_1000;
        const T_TRAV_DIR = 0b0000_0100;
        const P_WIN_DIR = 0b0000_0010;
        const T_WIN_DIR = 0b0000_0001;
    }

    #[cfg_attr(feature = "defmt", derive(defmt::Format))]
    pub struct PARA: u8 {
        const CMPS_EN = 0b1000_0000;
    }
}

macro_rules! getter {
    ($funcname:ident -> $reg:ident) => {
        fn $funcname(&mut self) -> Result<$reg, I::Error> {
            #[cfg(feature = "defmt")]
            trace!("Reading flags from {}", FlagRegister::$reg);
            Ok(<$reg>::from_bits_truncate(
                self.read_flags(FlagRegister::$reg)?,
            ))
        }
    };
}
macro_rules! setter {
    ($funcname:ident -> $reg:ident) => {
        fn $funcname(&mut self, val: $reg) -> Result<(), I::Error> {
            #[cfg(feature = "defmt")]
            trace!("Writing flags to {}", FlagRegister::$reg);
            self.write_flags(FlagRegister::$reg, val.bits())
        }
    };
}

// TODO: merge this with `Registers`?
pub trait Flags<I: I2c> {
    fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error>;
    fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error>;

    getter! { get_interrupts -> INT_SRC }
    getter! { get_interrupt_extras -> INT_DIR }
    getter! { get_interrupts_enabled -> INT_EN }
    getter! { get_interrupts_pinout -> INT_CFG }
    getter! { para -> PARA }

    setter! { set_interrupts_enabled -> INT_EN }
    setter! { set_interrupts_pinout -> INT_CFG }
    setter! { set_para -> PARA }
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

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn contains_sanity_check() {
        assert!(INT_SRC::from_bits_truncate(0x50).contains(INT_SRC::DEV_RDY));
    }

    #[test]
    #[should_panic]
    fn no_contains_sanity_check() {
        assert!(INT_SRC::from_bits_truncate(0x50).contains(INT_SRC::PA_RDY));
    }
}
