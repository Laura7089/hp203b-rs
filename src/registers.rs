use crate::{Command, HP203B};
use embedded_hal::i2c::blocking::{I2c, Operation};

/// Configure the I2C address select pin
pub mod csb {
    /// I2C address select pin
    pub trait CSB {
        /// I2C address indicated by the pin state
        const ADDR: u8;
    }
    /// CSB pin held high
    #[derive(Copy, Clone, Debug)]
    pub struct CSBHigh;
    /// CSB pin held low
    #[derive(Copy, Clone, Debug)]
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
#[derive(Copy, Clone, Debug)]
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
#[derive(Copy, Clone, Debug)]
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
