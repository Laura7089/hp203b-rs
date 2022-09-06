use crate::{Command, HP203B};
use embedded_hal::i2c::blocking::{I2c, Operation};

/// Configure the I2C address select pin
pub mod csb {
    /// I2C address select pin
    pub trait CSB {
        const ADDR: u8;
    }
    /// CSB pin held high
    pub struct CSBHigh;
    /// CSB pin held low
    pub struct CSBLow;
    impl CSB for CSBHigh {
        const ADDR: u8 = 0xED;
    }
    impl CSB for CSBLow {
        const ADDR: u8 = 0xEF;
    }
}

/// 8-bit numeric registers
#[allow(non_camel_case_types)]
#[allow(clippy::upper_case_acronyms)]
pub enum Register8 {
    /// Temperature event upper bound threshold
    ///
    /// 2's complement, in degress celsius.
    T_H_TH = 0x08,
    /// Temperature event middle threshold
    ///
    /// 2's complement, in degress celsius.
    T_M_TH = 0x09,
    /// Temperature event lower bound threshold
    ///
    /// 2's complement, in degress celsius.
    T_L_TH = 0x0A,
}

/// 16-bit numeric registers
#[allow(non_camel_case_types)]
pub enum Register16 {
    /// Altitude offset
    ///
    /// 2's complement, in cm.
    ///
    /// See the HP203B datasheet for more information.
    ALT_OFF = 0x00,
    /// Pressure event upper bound threshold
    ///
    /// 16-bit unsigned.
    /// When [`INT_CFG::PA_MODE`] is set, unit is 1 metre.
    /// When [`INT_CFG::PA_MODE`] is unset, unit is 0.02mb.
    PA_H_TH_LS = 0x02,
    /// Pressure event middle threshold
    ///
    /// 16-bit unsigned.
    /// When [`INT_CFG::PA_MODE`] is set, unit is 1 metre.
    /// When [`INT_CFG::PA_MODE`] is unset, unit is 0.02mb.
    PA_M_TH_LS = 0x04,
    /// Pressure event lower bound threshold
    ///
    /// 16-bit unsigned.
    /// When [`INT_CFG::PA_MODE`] is set, unit is 1 metre.
    /// When [`INT_CFG::PA_MODE`] is unset, unit is 0.02mb.
    PA_L_TH_LS = 0x06,
}

pub(crate) mod flags {
    use bitflags::bitflags;
    use embedded_hal::i2c::blocking::I2c;

    #[allow(non_camel_case_types)]
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

    bitflags! {
        /// Flags in [`Register8::INT_EN`]
        ///
        /// Enable or disable individual interrupts.
        #[allow(non_camel_case_types)]
        pub struct INT_EN: u8 {
            const PA_RDY_EN = 0b0010_0000;
            const T_RDY_EN = 0b0001_0000;
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

    pub trait Flags<I: I2c> {
        fn read_flags(&mut self, reg: FlagRegister) -> Result<u8, I::Error>;
        fn write_flags(&mut self, reg: FlagRegister, val: u8) -> Result<(), I::Error>;

        getter! { interrupts -> FlagRegister::INT_SRC, INT_SRC }
        getter! { get_interrupt_events -> FlagRegister::INT_DIR, INT_DIR }
        getter! { get_interrupts_enabled -> FlagRegister::INT_EN, INT_DIR }
        getter! { get_interrupts_pinout -> FlagRegister::INT_CFG, INT_CFG }
        getter! { para -> FlagRegister::PARA, PARA }

        setter! { set_interrupts_enabled -> FlagRegister::INT_EN, INT_DIR }
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
}

pub trait Registers<I: I2c> {
    const ADDR: u8;

    fn i2c(&mut self) -> &mut I;

    fn read_raw(&mut self, regaddr: u8) -> Result<u8, I::Error> {
        let mut val = [0];

        let to_write = [Command::READ_REG as u8 + regaddr];
        self.i2c().transaction(
            Self::ADDR,
            &mut [Operation::Write(&to_write), Operation::Read(&mut val)],
        )?;
        Ok(val[0])
    }

    fn write_raw(&mut self, regaddr: u8, val: u8) -> Result<(), I::Error> {
        let to_write = [Command::WRITE_REG as u8 + regaddr, val];
        self.i2c().write(Self::ADDR, &to_write)
    }

    fn read_reg8(&mut self, reg: Register8) -> Result<i8, I::Error> {
        let raw = self.read_raw(reg as u8)?;
        Ok(bytemuck::cast(raw))
    }

    fn read_reg16u(&mut self, reg: Register16) -> Result<u16, I::Error> {
        let lsb_addr = reg as u8;
        let raw = [self.read_raw(lsb_addr + 1)?, self.read_raw(lsb_addr)?];
        Ok(bytemuck::cast(raw))
    }

    fn read_reg16s(&mut self, reg: Register16) -> Result<i16, I::Error> {
        let lsb_addr = reg as u8;
        let raw = [self.read_raw(lsb_addr + 1)?, self.read_raw(lsb_addr)?];
        Ok(bytemuck::cast(raw))
    }

    fn write_reg8(&mut self, reg: Register8, val: i8) -> Result<(), I::Error> {
        self.write_raw(reg as u8, bytemuck::cast(val))
    }

    fn write_reg16u(&mut self, reg: Register16, val: u16) -> Result<(), I::Error> {
        let lsb_addr = reg as u8;
        let raw = bytemuck::bytes_of(&val);

        self.write_raw(lsb_addr, raw[1])
            .and(self.write_raw(lsb_addr + 1, raw[0]))
    }

    fn write_reg16s(&mut self, reg: Register16, val: i16) -> Result<(), I::Error> {
        let lsb_addr = reg as u8;
        let raw = bytemuck::bytes_of(&val);

        self.write_raw(lsb_addr, raw[1])
            .and(self.write_raw(lsb_addr + 1, raw[0]))
    }
}

impl<I2C, E, C> Registers<I2C> for HP203B<I2C, C>
where
    C: csb::CSB,
    I2C: I2c<Error = E>,
{
    const ADDR: u8 = C::ADDR;
    fn i2c(&mut self) -> &mut I2C {
        &mut self.i2c
    }
}
