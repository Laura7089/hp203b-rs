#![no_std]
mod registers;

use registers::{flags, Register16, Register8, Registers};
pub use registers::{CSBHigh, CSBLow, CSB};

use core::marker::PhantomData;
use embedded_hal::i2c::blocking::I2c;

pub struct HP203B<I2C, C> {
    i2c: I2C,
    _c: PhantomData<C>,
}

/// Decimation rate of internal digital filter
pub enum OSR {
    OSR4096 = 0b000,
    OSR2048 = 0b001,
    OSR1024 = 0b010,
    OSR512 = 0b011,
    OSR256 = 0b100,
    OSR128 = 0b101,
}

/// Which data to convert with internal ADC
pub enum Channel {
    SensorPressureTemperate = 0b00,
    Temperature = 0b10,
}

#[allow(non_camel_case_types)]
enum Command {
    SOFT_RST = 0x06,
    ADC_CVT = 0b01000000,
    READ_PT = 0x10,
    READ_AT = 0x11,
    READ_P = 0x30,
    READ_A = 0x31,
    READ_T = 0x32,
    ANA_CAL = 0x28,
    READ_REG = 0x80,
    WRITE_REG = 0xC0,
}

impl<I2C, E, C> HP203B<I2C, C>
where
    I2C: I2c<Error = E>,
    HP203B<I2C, C>: Registers<I2C>,
{
    pub fn new(i2c: I2C, ch: Channel, osr: OSR) -> Result<Self, E> {
        let mut new = Self {
            i2c,
            _c: PhantomData,
        };
        new.i2c.write(
            Self::ADDR,
            &[Command::ADC_CVT as u8 + ((osr as u8) << 2) + ch as u8],
        )?;
        Ok(new)
    }

    pub fn destroy(self) -> I2C {
        self.i2c
    }

    pub fn reset(&mut self) -> Result<(), E> {
        self.command(Command::SOFT_RST)
    }

    pub fn is_ready(&mut self) -> Result<bool, E> {
        Ok(
            !flags::INT_SRC::from_bits_truncate(self.read_reg8(Register8::INT_SRC)?)
                .contains(flags::INT_SRC::DEV_RDY),
        )
    }

    pub fn read_temp_pressure(&mut self) -> Result<(f32, f32), E> {
        todo!()
    }

    pub fn read_temp_alti(&mut self) -> Result<(f32, f32), E> {
        todo!()
    }

    fn command(&mut self, cmd: Command) -> Result<(), E> {
        self.i2c.write(Self::ADDR, &[cmd as u8])?;
        Ok(())
    }
}
