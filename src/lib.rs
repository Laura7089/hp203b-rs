//! To get started, create an [`HP203B`].
//!
//! ## Examples
//!
//! ```ignore
//! use hp203b::{HP203B, csb::CSBLow, OSR, Channel};
//! # fn main() -> Result<(), _> {
//!
//! // ... initialise i2c device
//! # let i2c = todo!();
//!
//! let altimeter = HP203B::<_, _, CSBLow>::new(
//!     i2c,
//!     OSR::OSR1024,
//!     Channel::SensorPressureTemperature,
//! )?;
//! let mut altimeter = altimeter.to_altitude()?;
//! altimeter.set_offset(1000)?; // We're 1000m above sea level
//! let alti = nb::block!(altimeter.read_alti())?;
//! println!("Altitude: {alti}m");
//! # Ok(())
//! # }
//! ```
//!
//! ## Features
//!
//! The `defmt` feature provides logging of various levels with the
//! [`defmt`](https://defmt.ferrous-systems.com/introduction.html) crate.
//! It is disabled by default.
#![no_std]
#![forbid(unsafe_code)]
#![deny(missing_docs)]
#![deny(clippy::pedantic)]
#![allow(clippy::missing_errors_doc)]
#![allow(clippy::enum_glob_use)]

// TODO: choose between `Barometric` and `Pressure/Altitude` to refer to the PA readings
// TODO: remove user access to the `*_RDY` interrupts?
// TODO: allow `no_run` tests to be run, use `embedded-hal-mock`?

mod flags;
pub mod interrupts;
mod registers;

use flags::{Flags, INT_CFG, INT_EN, INT_SRC};
pub use registers::csb;
use registers::{Register16, Register8, Registers};

use core::marker::PhantomData;
#[cfg(feature = "defmt")]
use defmt::{assert, debug, info, trace};
use embedded_hal::i2c::blocking::I2c;

/// Mode-setting for the altimeter
#[allow(missing_docs)]
pub mod mode {
    pub trait BarometricMeasurement {}
    /// Altitude, in metres
    pub struct Altitude;
    impl BarometricMeasurement for Altitude {}
    /// Pressure, in pascals
    pub struct Pressure;
    impl BarometricMeasurement for Pressure {}
}

/// A HOPERF HP203B altimeter/thermometer.
///
/// The type parameter `M` of this struct encodes whether the device shall be used to read altitude
/// or pressure.
/// Use [`Self::to_altitude`] and [`Self::to_pressure`] to switch between them.
/// Note that [`Self::new`] returns the device set for pressure.
///
/// The type parameter `C` encodes whether the `CSB` pin on the device is set high or low.
/// No direct mechanism for changing this is provided, on the assumption that the nature of the
/// device's connection will not change during program runtime.
pub struct HP203B<I, M = mode::Altitude, C = csb::CSBLow>
where
    I: I2c,
    M: mode::BarometricMeasurement,
    C: csb::CSB,
{
    i2c: I,
    waiting_baro: bool,
    waiting_temp: bool,
    waiting_reset: bool,
    _c: PhantomData<(C, M)>,
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
    OSR4096 = 0b0_0000,
    /// Decimeation rate = 2048
    OSR2048 = 0b0_0100,
    /// Decimeation rate = 1024
    OSR1024 = 0b0_1000,
    /// Decimeation rate = 512
    OSR512 = 0b0_1100,
    /// Decimeation rate = 256
    OSR256 = 0b1_0000,
    /// Decimeation rate = 128
    OSR128 = 0b1_0100,
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
    READ_PT = 0x10,
    READ_AT = 0x11,
    READ_P = 0x30,
    READ_A = 0x31,
    READ_T = 0x32,
}

impl<I, E, M, C> HP203B<I, M, C>
where
    I: I2c<Error = E>,
    M: mode::BarometricMeasurement,
    C: csb::CSB,
{
    /// Destroy the sensor struct and yield the I2C device it held
    pub fn destroy(self) -> I {
        self.i2c
    }

    /// Set the decimation rate of the filter and the channel to perform ADC on
    pub fn osr_channel(&mut self, osr: OSR, ch: Channel) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting {} and {}", osr, ch);
        let command = Command::ADC_CVT as u8 + osr as u8 + ch as u8;
        self.i2c.write(Self::ADDR, &[command])
    }

    /// Perform a software reset
    ///
    /// Returns [`nb::Result`] with `WouldBlock` until the device sets the `DEV_RDY` flag.
    ///
    /// # Note
    ///
    /// It is the caller's responsibility to ensure no other methods are called on the device until
    /// this method returns `Ok(())`.
    pub fn reset(&mut self) -> nb::Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Resetting device");
        self.waiting_temp = false;
        self.waiting_baro = false;
        if !self.waiting_reset {
            self.command(Command::SOFT_RST)?;
            self.waiting_reset = true;
        }

        if !self.is_ready()? {
            return Err(nb::Error::WouldBlock);
        }

        self.waiting_reset = false;
        Ok(())
    }

    /// Check the "device ready" flag
    pub fn is_ready(&mut self) -> Result<bool, E> {
        #[cfg(feature = "defmt")]
        debug!("Checking ready flag");
        Ok(self.get_interrupts()?.contains(flags::INT_SRC::DEV_RDY))
    }

    /// Set the window bounds for the temperature measurement
    ///
    /// Units are in degrees celsius.
    /// Used by the [`interrupts::Event::TemperatureOutsideWindow`] interrupt.
    ///
    /// # Panics
    ///
    /// If `lower > upper`.
    pub fn set_temp_bounds(&mut self, lower: i8, upper: i8) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting temperature bounds ({}, {})", upper, lower);
        assert!(
            lower <= upper,
            "Lower bound {} is larger than upper bound {}",
            lower,
            upper,
        );
        self.write_reg8(Register8::T_L_TH, lower)
            .and(self.write_reg8(Register8::T_H_TH, upper))
    }

    /// Set the middle threshold for the temperature measurement
    ///
    /// Units are in degrees celvius.
    /// Used by the [`interrupts::Event::TemperatureTraversed`] interrupt.
    pub fn set_temp_mid(&mut self, mid: i8) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting temperature mid bound {}", mid);
        self.write_reg8(Register8::T_M_TH, mid)
    }

    /// Recalibrate the internal analog blocks
    pub fn recalibrate_analog(&mut self) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Recalibrating analog blocks");
        self.command(Command::ANA_CAL)
    }

    /// Enable or disable compensation
    pub fn compensate(&mut self, comp: bool) -> Result<(), E> {
        let flag = if comp {
            flags::PARA::CMPS_EN
        } else {
            flags::PARA::empty()
        };
        #[cfg(feature = "defmt")]
        debug!("Setting compensation flag = {}", flag);
        self.set_para(flag)
    }

    /// Check if compensation is enabled
    pub fn is_compensation_enabled(&mut self) -> Result<bool, E> {
        #[cfg(feature = "defmt")]
        debug!("Checking compensation enabled");
        Ok(self.para()?.contains(flags::PARA::CMPS_EN))
    }

    /// Get temperature in celsius
    ///
    /// Returns [`nb::Result`] with `WouldBlock` until the device sets the `T_RDY` flag.
    pub fn read_temp(&mut self) -> nb::Result<f32, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading temperature");
        if !self.waiting_temp {
            self.command(Command::READ_T)?;
            self.waiting_temp = true;
        }
        self.read_one(INT_SRC::T_RDY)
    }

    fn read_one(&mut self, ready: INT_SRC) -> nb::Result<f32, E> {
        #[cfg(feature = "defmt")]
        trace!("Reading value {}", ready);

        if !self.get_interrupts()?.contains(ready) {
            #[cfg(feature = "defmt")]
            trace!("Device hasn't set {}, sending WouldBlock", ready);
            return Err(nb::Error::WouldBlock);
        }

        let mut raw = [0; 3];
        self.i2c.read(Self::ADDR, &mut raw)?;
        match ready {
            INT_SRC::PA_RDY => self.waiting_baro = false,
            INT_SRC::T_RDY => self.waiting_temp = false,
            _ => unreachable!(),
        }
        Ok(raw_reading_to_float(&raw))
    }

    fn read_two(&mut self) -> nb::Result<(f32, f32), E> {
        #[cfg(feature = "defmt")]
        trace!("Reading both values from device");

        if !self
            .get_interrupts()?
            .contains(INT_SRC::PA_RDY & INT_SRC::T_RDY)
        {
            #[cfg(feature = "defmt")]
            trace!("Device doesn't have both ready flags set, sending WouldBlock");
            return Err(nb::Error::WouldBlock);
        }

        let mut raw = [0; 6];
        self.i2c.read(Self::ADDR, &mut raw)?;
        self.waiting_temp = false;
        self.waiting_baro = false;
        Ok((
            raw_reading_to_float(&raw[0..3]),
            raw_reading_to_float(&raw[3..6]),
        ))
    }

    fn command(&mut self, cmd: Command) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        trace!("Sending command {}", cmd);
        self.i2c.write(Self::ADDR, &[cmd as u8])
    }
}

impl<I, E, C> HP203B<I, mode::Pressure, C>
where
    I: I2c<Error = E>,
    C: csb::CSB,
{
    /// The maximum value that the pressure bounds on the altimeter can accept
    pub const PRES_MAX: f32 = (u16::MAX as f32) / 0.02;

    #[allow(clippy::cast_sign_loss)]
    #[allow(clippy::cast_possible_truncation)]
    fn bound_to_dev_int(bound: f32) -> u16 {
        assert!(bound.is_sign_positive());
        assert!(bound <= Self::PRES_MAX,);
        assert!(bound.is_normal());

        (bound / 0.02) as u16
    }

    /// Initialise the device in pressure mode
    ///
    /// Actions carried out:
    ///
    /// 1. Takes ownership of an I2C bus/device.
    /// 1. Blocks on resetting the device with [`Self::reset`].
    /// 1. Sets settings for the onboard ADC (see [`Self::osr_channel`]).
    /// 1. Resets the configuration registers.
    pub fn new(i2c: I, osr: OSR, ch: Channel) -> Result<Self, E> {
        #[cfg(feature = "defmt")]
        debug!("Creating new HP203B altimeter");
        let mut new = Self {
            i2c,
            _c: PhantomData,
            waiting_baro: false,
            waiting_temp: false,
            waiting_reset: false,
        };
        nb::block!(new.reset())?;
        new.osr_channel(osr, ch)?;
        new.set_interrupts_enabled(INT_EN::RDY_EN)?;
        new.set_interrupts_pinout(INT_CFG::from_bits_truncate(0))?;
        #[cfg(feature = "defmt")]
        info!("HP203B altimeter object created and configured");
        Ok(new)
    }

    /// Convert the altimeter to read altitude
    ///
    /// Note that this resets the interrupt flags for [`interrupts::Event::PATraversed`] and
    /// [`interrupts::Event::PAOutsideWindow`]
    pub fn to_altitude(self) -> Result<HP203B<I, mode::Altitude, C>, E> {
        let mut new = HP203B::<_, mode::Altitude, _> {
            _c: PhantomData,
            waiting_baro: false,
            waiting_temp: self.waiting_temp,
            waiting_reset: false,
            i2c: self.destroy(),
        };

        #[cfg(feature = "defmt")]
        debug!("Converting altimeter to altitude mode");

        new.set_alti_bounds(0, 0)?;
        new.set_alti_mid(0)?;

        let mut new_en_flags = new.get_interrupts_enabled()?;
        new_en_flags.remove(INT_EN::PA_TRAV_EN);
        new_en_flags.remove(INT_EN::PA_WIN_EN);
        new.set_interrupts_enabled(new_en_flags)?;

        new.set_offset(0)?;

        let mut new_pinout_flags = new.get_interrupts_pinout()?;
        new_pinout_flags.insert(INT_CFG::PA_MODE);
        new.set_interrupts_pinout(new_pinout_flags)?;

        #[cfg(feature = "defmt")]
        info!("Altimeter succesfully set to altitude mode");

        Ok(new)
    }

    /// Set the window bounds for the pressure measurement
    ///
    /// Units are in mbar.
    /// Used by the [`interrupts::Event::PAOutsideWindow`] interrupt.
    ///
    /// # Panics
    ///
    /// - If `lower > upper`
    /// - Either are not in the range `0`, [`Self::PRES_MAX`]
    /// - Either are not "normal"; see [`f32::is_normal`]
    pub fn set_pres_bounds(&mut self, lower: f32, upper: f32) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting pressure bounds: ({}mbar, {}mbar)", lower, upper);

        assert!(
            lower <= upper,
            "Lower bound {} is larger than upper bound {}",
            lower,
            upper,
        );
        let lower = Self::bound_to_dev_int(lower);
        let upper = Self::bound_to_dev_int(upper);

        self.write_reg16u(Register16::PA_L_TH_LS, lower)
            .and(self.write_reg16u(Register16::PA_H_TH_LS, upper))
    }

    /// Set the middle threshold for the pressure measurement
    ///
    /// Units are in mbar.
    /// Used by the [`interrupts::Event::PATraversed`] interrupt.
    ///
    /// # Panics
    ///
    /// - `mid` is not in the range `0`, [`Self::PRES_MAX`]
    /// - `mid` is not "normal"; see [`f32::is_normal`]
    pub fn set_pres_mid(&mut self, mid: f32) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting pressure midpoint: {}mbar", mid);

        let mid = Self::bound_to_dev_int(mid);
        self.write_reg16u(Register16::PA_M_TH_LS, mid)
    }

    /// Read both the temperature and pressure
    ///
    /// Returns [`nb::Result`] with `WouldBlock` until the device sets **both** the `T_RDY`
    /// and `PA_RDY` flags.
    pub fn read_pres_temp(&mut self) -> nb::Result<(f32, f32), E> {
        #[cfg(feature = "defmt")]
        debug!("Reading temperature and pressure");

        match (self.waiting_temp, self.waiting_baro) {
            (false, false) => self.command(Command::READ_PT)?,
            (true, false) => self.command(Command::READ_P)?,
            (false, true) => self.command(Command::READ_T)?,
            (true, true) => (),
        }
        self.waiting_temp = true;
        self.waiting_baro = true;

        self.read_two()
    }

    /// Read a pressure measurement
    ///
    /// Returns [`nb::Result`] with `WouldBlock` until the devices sets the `PA_RDY` flag.
    pub fn read_pres(&mut self) -> nb::Result<f32, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading pressure");

        if !self.waiting_baro {
            self.command(Command::READ_P)?;
        }
        self.waiting_baro = true;

        self.read_one(INT_SRC::PA_RDY)
    }
}

impl<I, E, C> HP203B<I, mode::Altitude, C>
where
    I: I2c<Error = E>,
    C: csb::CSB,
{
    /// Convert the altimeter to read pressure
    ///
    /// Note that this resets the interrupt flags for [`interrupts::Event::PATraversed`] and
    /// [`interrupts::Event::PAOutsideWindow`]
    pub fn to_pressure(self) -> Result<HP203B<I, mode::Pressure, C>, E> {
        let mut new = HP203B::<_, mode::Pressure, _> {
            _c: PhantomData,
            waiting_temp: self.waiting_temp,
            waiting_baro: false,
            waiting_reset: false,
            i2c: self.destroy(),
        };

        #[cfg(feature = "defmt")]
        debug!("Converting altimeter to pressure mode");

        new.set_pres_bounds(0.0, 0.0)?;
        new.set_pres_mid(0.0)?;

        let mut new_en_flags = new.get_interrupts_enabled()?;
        new_en_flags.remove(INT_EN::PA_TRAV_EN);
        new_en_flags.remove(INT_EN::PA_WIN_EN);
        new.set_interrupts_enabled(new_en_flags)?;

        let mut new_pinout_flags = new.get_interrupts_pinout()?;
        new_pinout_flags.remove(INT_CFG::PA_MODE);
        new.set_interrupts_pinout(new_pinout_flags)?;

        #[cfg(feature = "defmt")]
        info!("Altimeter succesfully set to pressure mode");

        Ok(new)
    }

    /// Set the altitude offset
    ///
    /// `offset` is the current altitude in centimetres.
    pub fn set_offset(&mut self, offset: i16) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting altitude offset to {}cm", offset);

        self.write_reg16s(Register16::ALT_OFF, offset)
    }

    /// Set the window bounds for the altitude measurement
    ///
    /// Units are in metres.
    /// Used by the [`interrupts::Event::PAOutsideWindow`] interrupt.
    ///
    /// # Panics
    ///
    /// If `lower > upper`.
    pub fn set_alti_bounds(&mut self, lower: i16, upper: i16) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting altitude outer bounds to ({}m, {}m)", lower, upper);

        assert!(
            lower <= upper,
            "Lower bound {} is larger than upper bound {}",
            lower,
            upper,
        );
        self.write_reg16s(Register16::PA_L_TH_LS, lower)
            .and(self.write_reg16s(Register16::PA_H_TH_LS, upper))
    }

    /// Set the middle threshold for the altitude measurement
    ///
    /// Units are in metres.
    /// Used by the [`interrupts::Event::PATraversed`] interrupt.
    pub fn set_alti_mid(&mut self, mid: i16) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting altitude mid bound to {}m)", mid);
        self.write_reg16s(Register16::PA_M_TH_LS, mid)
    }

    /// Read both the temperature and altitude
    ///
    /// Returns [`nb::Result`] with `WouldBlock` until the device sets **both** the `T_RDY`
    /// and `PA_RDY` flags.
    pub fn read_alti_temp(&mut self) -> nb::Result<(f32, f32), E> {
        #[cfg(feature = "defmt")]
        debug!("Reading altitude and temperature");

        match (self.waiting_temp, self.waiting_baro) {
            (false, false) => self.command(Command::READ_AT)?,
            (true, false) => self.command(Command::READ_A)?,
            (false, true) => self.command(Command::READ_T)?,
            (true, true) => (),
        }
        self.waiting_temp = true;
        self.waiting_baro = true;

        self.read_two()
    }

    /// Read an altitude measurement
    ///
    /// Returns [`nb::Result`] with `WouldBlock` until the device sets the `PA_RDY` flag.
    pub fn read_alti(&mut self) -> nb::Result<f32, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading altitude");

        if !self.waiting_baro {
            self.command(Command::READ_A)?;
        }

        self.read_one(INT_SRC::PA_RDY)
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

// TODO: more tests
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
