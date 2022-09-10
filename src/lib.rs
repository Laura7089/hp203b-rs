//! To get started, create a [`HP203B`].
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![deny(clippy::pedantic)]
#![allow(clippy::missing_errors_doc)]

// TODO: add methods for setting window and traversal figures
// TODO: choose between `Barometric` and `Pressure/Altitude` to refer to the PA readings

mod flags;
// TODO: cargo flag
pub mod interrupts;
mod registers;

use flags::Flags;
pub use registers::csb;
use registers::{Register16, Registers};

use core::marker::PhantomData;
use embedded_hal::i2c::blocking::I2c;

/// A HOPERF HP203B altimeter/thermometer.
pub struct HP203B<I2C, C = csb::CSBLow>
where
    I2C: I2c,
    C: csb::CSB,
{
    i2c: I2C,
    _c: PhantomData<C>,
}

/// Decimation rate of internal digital filter
///
/// From the datasheet:
///
/// OSR | Temp Conversion Time (ms) | Temp + Pressure/Alt Conv. Time (ms)
/// ---|---|---
/// 128 | 2.1 | 4.1
/// 256 | 4.1 | 8.2
/// 512 | 8.2 | 16.4
/// 1024 | 16.4 | 32.8
/// 2048 | 32.8 | 65.6
/// 4096 | 65.6 | 131.1
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum OSR {
    /// Decimeation rate = 4096
    OSR4096 = 0b000,
    /// Decimeation rate = 2048
    OSR2048 = 0b001,
    /// Decimeation rate = 1024
    OSR1024 = 0b010,
    /// Decimeation rate = 512
    OSR512 = 0b011,
    /// Decimeation rate = 256
    OSR256 = 0b100,
    /// Decimeation rate = 128
    OSR128 = 0b101,
}

/// Which data to convert with internal ADC
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Channel {
    /// Convert Pressure/Altitude *and* temperature
    SensorPressureTemperature = 0b00,
    /// Just convert temperature
    Temperature = 0b10,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
enum Command {
    SOFT_RST = 0x06,
    ADC_CVT = 0x40,
    ANA_CAL = 0x28,
    READ_REG = 0x80,
    WRITE_REG = 0xC0,
}

/// Which measurement to make from the barometric function of the device
pub enum BaroMeasurement {
    /// Pressure in pascals
    Pressure,
    // TODO: check this unit
    /// Altitude in metres
    Altitude,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
enum ReadValDouble {
    PT = 0x10,
    AT = 0x11,
}

impl Into<ReadValDouble> for BaroMeasurement {
    fn into(self) -> ReadValDouble {
        match self {
            BaroMeasurement::Pressure => ReadValDouble::PT,
            BaroMeasurement::Altitude => ReadValDouble::AT,
        }
    }
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
enum ReadValSingle {
    P = 0x30,
    A = 0x31,
    T = 0x32,
}

impl Into<ReadValSingle> for BaroMeasurement {
    fn into(self) -> ReadValSingle {
        match self {
            BaroMeasurement::Pressure => ReadValSingle::P,
            BaroMeasurement::Altitude => ReadValSingle::A,
        }
    }
}

impl<I2C, E, C> HP203B<I2C, C>
where
    I2C: I2c<Error = E>,
    C: csb::CSB,
    HP203B<I2C, C>: Registers<I2C>,
{
    /// Initialise the device
    ///
    /// Takes an I2C bus and settings for the onboard ADC.
    /// See [`Self::osr_channel`].
    pub fn new(i2c: I2C, osr: OSR, ch: Channel) -> Result<Self, E> {
        let mut new = Self {
            i2c,
            _c: PhantomData,
        };
        new.reset()?;
        // TODO: sleep?
        new.osr_channel(osr, ch)?;
        Ok(new)
    }

    /// Destroy the sensor struct and yield the I2C device it held
    pub fn destroy(self) -> I2C {
        self.i2c
    }

    /// Set the decimation rate of the filter and the channel to perform ADC on
    pub fn osr_channel(&mut self, osr: OSR, ch: Channel) -> Result<(), E> {
        self.i2c.write(
            Self::ADDR,
            &[Command::ADC_CVT as u8 + ((osr as u8) << 2) + ch as u8],
        )
    }

    /// Perform a software reset
    pub fn reset(&mut self) -> Result<(), E> {
        self.command(Command::SOFT_RST)
        // TODO: sleep?
    }

    /// Check the "device ready" flag
    pub fn is_ready(&mut self) -> Result<bool, E> {
        Ok(self.get_interrupts()?.contains(flags::INT_SRC::DEV_RDY))
    }

    /// Set the altitude offset
    ///
    /// `offset` is the current altitude in centimetres.
    pub fn set_alt_offset(&mut self, offset: i16) -> Result<(), E> {
        self.write_reg16s(Register16::ALT_OFF, offset)?;
        Ok(())
    }

    /// Recalibrate the internal analog blocks
    pub fn recalibrate_analog(&mut self) -> Result<(), E> {
        self.command(Command::ANA_CAL)
    }

    /// Enable or disable compensation
    pub fn compensate(&mut self, comp: bool) -> Result<(), E> {
        let flag = match comp {
            true => flags::PARA::CMPS_EN,
            false => flags::PARA::empty(),
        };
        self.set_para(flag)
    }

    /// Check if compensation is enabled
    pub fn is_compensation_enabled(&mut self) -> Result<bool, E> {
        Ok(self.para()?.contains(flags::PARA::CMPS_EN))
    }

    /// Read both a barometric measurement and the temperature
    ///
    /// From the device, read and return (in that order) both:
    ///
    /// 1. temperature value (celsius)
    /// 1. `measure`
    pub fn read_both(&mut self, measure: BaroMeasurement) -> Result<(f32, f32), E> {
        self.read_two(measure.into())
    }

    /// Read a barometric measurement
    pub fn read_bar(&mut self, measure: BaroMeasurement) -> Result<f32, E> {
        self.read_one(measure.into())
    }

    /// Gets temperature in celsius
    pub fn read_temp(&mut self) -> Result<f32, E> {
        self.read_one(ReadValSingle::T)
    }

    fn read_one(&mut self, cmd: ReadValSingle) -> Result<f32, E> {
        let mut raw = [0; 3];
        self.i2c.write_read(Self::ADDR, &[cmd as u8], &mut raw)?;
        Ok(raw_reading_to_float(&raw))
    }

    fn read_two(&mut self, cmd: ReadValDouble) -> Result<(f32, f32), E> {
        let mut raw = [0; 6];
        self.i2c.write_read(Self::ADDR, &[cmd as u8], &mut raw)?;
        Ok((
            raw_reading_to_float(&raw[0..3]),
            raw_reading_to_float(&raw[3..6]),
        ))
    }

    fn command(&mut self, cmd: Command) -> Result<(), E> {
        self.i2c.write(Self::ADDR, &[cmd as u8])
    }
}

/// Takes a 24-bit 2's complement number and converts it to a float/100
#[allow(clippy::cast_precision_loss)]
fn raw_reading_to_float(reading: &[u8]) -> f32 {
    assert!(reading.len() == 3);
    let signed: i32 = {
        let mut base = if reading[0] & (1 << 3) == (1 << 3) {
            i32::MIN + 0x7FF8_0000
        } else {
            0
        };
        base += i32::from(reading[0] & 0b0000_0111) << 16;
        base += i32::from(reading[1]) << 8;
        base += i32::from(reading[2]);
        base
    };
    signed as f32 / 100.0
}

#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(&[0x00, 0x0A, 0x5C], 26.52)]
    #[test_case(&[0xFF, 0xFC, 0x02], -10.22)]
    #[test_case(&[0x01, 0x8A, 0x9E], 1010.22)]
    fn reading_to_float(input: &[u8], expected: f32) {
        assert_eq!(raw_reading_to_float(input), expected);
    }
}
