//! To get started, create an [`HP203B`].
//!
//! ## Examples
//!
//! ```no_run
//! use hp203b::{HP203B, ReadGuard, csb::CSBLow, OSR, Channel};
//! # use embedded_hal::i2c::ErrorKind;
//! # use embedded_hal_mock::i2c::Mock;
//! # fn main() -> Result<(), ErrorKind> {
//!
//! // ... initialise i2c device
//! # let i2c = Mock::new(&[]);
//!
//! let altimeter = HP203B::<_, _, CSBLow>::new(
//!     i2c,
//!     OSR::OSR1024,
//!     Channel::SensorPressureTemperature,
//! )?;
//! let mut altimeter = altimeter.to_altitude()?;
//! altimeter.set_offset(1000)?; // We're 1000m above sea level
//! let mut alti_guard = altimeter.read_alti()?;
//! let alti = nb::block!(alti_guard.try_take())?;
//! println!("Altitude: {}m", alti.0);
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

// TODO: remove user access to the `*_RDY` interrupts?
// TODO: allow non-blocking reads
// TODO: fixed-point decimal representation?
// TODO: RAII guards for value reads w/ attached delay, panic on reuse

mod flags;
pub mod interrupts;
mod registers;

use flags::{Flags, INT_CFG, INT_EN};
pub use registers::csb;
use registers::{Register16, Register8, Registers};

use core::marker::PhantomData;
#[cfg(feature = "defmt")]
use defmt::{assert, debug, info, trace};
use embedded_hal::i2c::I2c;

/// Mode-setting for the altimeter
#[allow(missing_docs)]
pub mod mode {
    pub trait BarometricMeasurement {}
    pub struct Altitude;
    impl BarometricMeasurement for Altitude {}
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
    osr: OSR,
    _c: PhantomData<(C, M)>,
}

// TODO: derive `Default` on settings-related enums

/// Decimation rate of internal digital filter
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum OSR {
    /// Decimation rate = 4096
    ///
    /// Time for measurement = 65.6ms
    OSR4096 = 0b0_0000,
    /// Decimation rate = 2048
    ///
    /// Time for measurement = 32.8ms
    OSR2048 = 0b0_0100,
    /// Decimation rate = 1024
    ///
    /// Time for measurement = 16.4ms
    OSR1024 = 0b0_1000,
    /// Decimation rate = 512
    ///
    /// Time for measurement = 8.2ms
    OSR512 = 0b0_1100,
    /// Decimation rate = 256
    ///
    /// Time for measurement = 4.1ms
    OSR256 = 0b1_0000,
    /// Decimation rate = 128
    ///
    /// Time for measurement = 2.1ms
    OSR128 = 0b1_0100,
}

use fugit::{ExtU32, MicrosDurationU32};
impl OSR {
    /// Time delay associated with OSR setting
    ///
    /// The amount of time, in microseconds, that a single measurement is expected to take on the
    /// onboard ADC.
    /// Double this for [`HP203B::read_alti_temp`] and [`HP203B::read_pres_temp`].
    #[must_use]
    pub fn associated_delay(self) -> MicrosDurationU32 {
        match self {
            OSR::OSR128 => 2100,
            OSR::OSR256 => 4100,
            OSR::OSR512 => 8200,
            OSR::OSR1024 => 16400,
            OSR::OSR2048 => 32800,
            OSR::OSR4096 => 65600,
        }
        .millis()
    }
}

/// Which data to convert with internal ADC
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
pub enum Channel {
    /// Convert Pressure/Altitude *and* temperature
    SensorPressureTemperature = 0b00,
    /// Just convert temperature
    Temperature = 0b10,
}

#[allow(non_camel_case_types)]
#[derive(Copy, Clone, Debug)]
#[cfg_attr(feature = "defmt", derive(defmt::Format))]
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
    pub fn set_osr_channel(&mut self, osr: OSR, ch: Channel) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Setting {} and {}", osr, ch);
        let command = Command::ADC_CVT as u8 + osr as u8 + ch as u8;
        self.i2c.write(Self::ADDR, &[command])
    }

    /// Get the expected delay before another measurement will be ready
    ///
    /// Double this if you want to read two measurements.
    /// Calls [`OSR::associated_delay`] under the hood.
    pub fn read_delay(&self) -> MicrosDurationU32 {
        self.osr.associated_delay()
    }

    /// Perform a software reset
    ///
    /// Doesn't wait for the ready state - use [`Self::is_ready`] for this.
    ///
    /// # Note
    ///
    /// It is the caller's responsibility to ensure no other methods are called on the device until
    /// this method returns `Ok(())`.
    pub fn reset(&mut self) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        debug!("Resetting device");
        self.command(Command::SOFT_RST)?;
        Ok(())
    }

    /// Check the "device ready" flag
    pub fn is_ready(&mut self) -> Result<bool, E> {
        #[cfg(feature = "defmt")]
        debug!("Checking ready flag");
        Ok(self.get_interrupts()?.contains(flags::INT_SRC::DEV_RDY))
    }

    /// Returns `Err(WouldBlock)` until the `DEV_RDY` flag is set on the device
    pub fn wait_ready(&mut self) -> nb::Result<(), E> {
        if !self.is_ready()? {
            #[cfg(feature = "defmt")]
            trace!("Device hasn't set ready flag, sending WouldBlock");
            return Err(nb::Error::WouldBlock);
        }

        Ok(())
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
    pub fn read_temp(&mut self) -> Result<TemperatureGuard<I, M, C>, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading temperature");
        self.i2c.write(Self::ADDR, &[Command::READ_T as u8])?;
        Ok(TemperatureGuard(Some(self)))
    }

    /// Get temperature in celsius, block until ready
    #[inline]
    pub fn read_temp_blocking(&mut self) -> Result<Temperature, E> {
        let mut guard = self.read_temp()?;
        nb::block!(guard.try_take())
    }

    fn read_one(&mut self) -> Result<[u8; 3], E> {
        let mut raw = [0; 3];
        #[cfg(feature = "defmt")]
        trace!("Reading 3 bytes");
        self.i2c.read(Self::ADDR, &mut raw)?;
        Ok(raw)
    }

    fn read_two(&mut self) -> Result<[u8; 6], E> {
        #[cfg(feature = "defmt")]
        trace!("Reading both values from device");
        let mut raw = [0; 6];
        self.i2c.read(Self::ADDR, &mut raw)?;
        Ok(raw)
    }

    fn command(&mut self, cmd: Command) -> Result<(), E> {
        #[cfg(feature = "defmt")]
        trace!("Sending command {}", cmd);
        self.i2c.write(Self::ADDR, &[cmd as u8])
    }

    fn inner_block(&mut self, event: flags::INT_SRC) -> nb::Result<(), E> {
        if self.get_interrupts()?.contains(event) {
            Ok(())
        } else {
            Err(nb::Error::WouldBlock)
        }
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
    /// 1. Sets settings for the onboard ADC (see [`Self::set_osr_channel`]).
    /// 1. Resets the configuration registers.
    pub fn new(i2c: I, osr: OSR, ch: Channel) -> Result<Self, E> {
        #[cfg(feature = "defmt")]
        debug!("Creating new HP203B altimeter");
        let mut new = Self {
            i2c,
            _c: PhantomData,
            osr,
        };
        new.reset()?;
        nb::block!(new.wait_ready())?;
        new.set_osr_channel(osr, ch)?;
        new.set_interrupts_enabled(INT_EN::RDY_EN)?;
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
            osr: self.osr,
            i2c: self.destroy(),
        };

        #[cfg(feature = "defmt")]
        debug!("Converting altimeter to altitude mode");

        new.set_alti_bounds(0, 0)?;
        new.set_alti_mid(0)?;
        new.set_offset(0)?;

        let mut new_en_flags = new.get_interrupts_enabled()?;
        new_en_flags.remove(INT_EN::PA_TRAV_EN);
        new_en_flags.remove(INT_EN::PA_WIN_EN);
        new.set_interrupts_enabled(new_en_flags)?;

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
    pub fn read_pres_temp(&mut self) -> Result<TemperaturePressureGuard<I, mode::Pressure, C>, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading temperature and pressure");
        self.i2c.write(Self::ADDR, &[Command::READ_PT as u8])?;
        Ok(TemperaturePressureGuard(Some(self)))
    }

    /// Read both the temperature and pressure, block until ready
    pub fn read_pres_temp_blocking(&mut self) -> Result<(Temperature, Pressure), E> {
        let mut guard = self.read_pres_temp()?;
        nb::block!(guard.try_take())
    }

    /// Read a pressure measurement
    pub fn read_pres(&mut self) -> Result<PressureGuard<I, mode::Pressure, C>, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading pressure");
        self.i2c.write(Self::ADDR, &[Command::READ_P as u8])?;
        Ok(PressureGuard(Some(self)))
    }

    /// Read a pressure measurement, block until ready
    pub fn read_pres_blocking(&mut self) -> Result<Pressure, E> {
        let mut guard = self.read_pres()?;
        nb::block!(guard.try_take())
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
            osr: self.osr,
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
    pub fn read_alti_temp(&mut self) -> Result<TemperatureAltitudeGuard<I, mode::Altitude, C>, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading altitude and temperature");
        self.i2c.write(Self::ADDR, &[Command::READ_AT as u8])?;
        Ok(TemperatureAltitudeGuard(Some(self)))
    }

    /// Read both the temperature and altitude, block until ready
    pub fn read_alti_temp_blocking(&mut self) -> Result<(Temperature, Altitude), E> {
        let mut guard = self.read_alti_temp()?;
        nb::block!(guard.try_take())
    }

    /// Read an altitude measurement
    pub fn read_alti(&mut self) -> Result<AltitudeGuard<I, mode::Altitude, C>, E> {
        #[cfg(feature = "defmt")]
        debug!("Reading altitude");
        self.i2c.write(Self::ADDR, &[Command::READ_A as u8])?;
        Ok(AltitudeGuard(Some(self)))
    }

    /// Read an altitude measurement, block until ready
    pub fn read_alti_blocking(&mut self) -> Result<Altitude, E> {
        let mut guard = self.read_alti()?;
        nb::block!(guard.try_take())
    }
}

/// An RAII guard for a value read from an [`HP203B`]
///
/// This trait allows this driver to yield control flow while the altimeter generates a reading,
/// while still enforcing exclusive access to the device.
/// A typical flow (using [`HP203B::read_temp`]) might look like this:
///
/// ```no_run
/// use hp203b::{HP203B, ReadGuard};
/// # use hp203b::{csb::CSBLow, OSR, Channel};
/// # use embedded_hal::i2c::ErrorKind;
/// # use embedded_hal_mock::i2c::Mock;
/// # fn main() -> Result<(), ErrorKind> {
///
/// let mut alti = {
///     // ... initialise device
/// # let i2c = Mock::new(&[]);
/// # HP203B::<_, _, CSBLow>::new(
/// #     i2c,
/// #     OSR::OSR1024,
/// #     Channel::SensorPressureTemperature,
/// # )?
/// };
/// let mut temp_guard = alti.read_temp()?;
///
/// // do something else while the altimeter calculates...
///
/// let temp = nb::block!(temp_guard.try_take())?;
/// println!("Temperature: {} degrees", temp.0);
/// # Ok(()) }
/// ```
pub trait ReadGuard<T> {
    /// Associated error type
    ///
    /// Will match the error type of the I2C given to [`HP203B`].
    type Error;

    /// Expected delay of this measurement being ready
    fn delay(&self) -> MicrosDurationU32;
    /// Read the value off the device or yield execution if not ready
    ///
    /// # Panics
    ///
    /// Panics if the value has previously been taken.
    fn try_take(&mut self) -> nb::Result<T, Self::Error>;
}

macro_rules! readguard_impl_inner {
    ($name:ident, $rettype:ty, $flag:expr, $reader:path, $conv:path, #[$doc:meta]) => {
        #[$doc]
        pub struct $name<'a, I: I2c, M: mode::BarometricMeasurement, C: csb::CSB>(
            Option<&'a mut HP203B<I, M, C>>,
        );

        impl<'a, I: I2c<Error = E>, M: mode::BarometricMeasurement, C: csb::CSB, E>
            ReadGuard<$rettype> for $name<'a, I, M, C>
        {
            type Error = E;

            fn delay(&self) -> MicrosDurationU32 {
                self.0.as_ref().expect("Reading already used").read_delay()
            }

            fn try_take(&mut self) -> nb::Result<$rettype, Self::Error> {
                self.0
                    .as_mut()
                    .expect("Reading already used")
                    .inner_block($flag)?;
                let dev = self.0.take().unwrap();
                let read = $reader(dev)?;
                Ok($conv(read))
            }
        }
    };
}
macro_rules! readguard_impl {
    ($kind:ident, $flag:expr) => {
        paste::paste! {
        readguard_impl_inner!(
            [<$kind Guard>],
            $kind,
            $flag,
            HP203B::read_one,
            Into::into,
            #[doc = "RAII Guard (see [`ReadGuard`]) for a [`" $kind "`] reading"]
        );
        }
    };
    ($kind1:ident, $kind2:ident, $flag:expr) => {
        paste::paste! {
        #[allow(non_snake_case)]
        #[inline(always)]
        fn [<__ $kind1 $kind2 conv>](val: [u8; 6]) -> ($kind1, $kind2) {
            (val[0..3].into(), val[3..6].into())
        }
        readguard_impl_inner!(
            [<$kind1 $kind2 Guard>],
            ($kind1, $kind2),
            $flag,
            HP203B::read_two,
            [<__ $kind1 $kind2 conv>],
            #[doc = "RAII Guard (see [`ReadGuard`]) for a reading of [`" $kind1 "`] and [`" $kind2 "`]"]
        );
        }
    };
}

readguard_impl!(Temperature, flags::INT_SRC::T_RDY);
readguard_impl!(Pressure, flags::INT_SRC::PA_RDY);
readguard_impl!(Altitude, flags::INT_SRC::PA_RDY);
readguard_impl!(Temperature, Pressure, flags::INT_SRC::READ_RDY);
readguard_impl!(Temperature, Altitude, flags::INT_SRC::READ_RDY);

fn read_signed(reading: &[u8]) -> f32 {
    assert!(reading.len() == 3);
    let signed: i32 = {
        let base = if reading[0] & 0b1000 == 0b1000 {
            i32::MIN + 0x7FF8_0000
        } else {
            0
        };
        base + (i32::from(reading[0] & 0b0000_0111) << 16)
            + (i32::from(reading[1]) << 8)
            + i32::from(reading[2])
    };
    signed as f32
}

fn read_unsigned(reading: &[u8]) -> f32 {
    assert!(reading.len() == 3);
    let signed: u32 = {
        (u32::from(reading[0] & 0b0000_1111) << 16)
            + (u32::from(reading[1]) << 8)
            + u32::from(reading[2])
    };
    signed as f32
}

macro_rules! reading_impl {
    ($kind:ident, $unit:expr, $reader:path) => {
        paste::paste! {
        #[doc = $kind " reading, in"]
        #[doc = $unit]
        #[derive(Copy, Clone, Debug, PartialEq)]
        #[repr(transparent)]
        pub struct $kind(pub f32);
        impl From<&[u8]> for $kind {
            fn from(reading: &[u8]) -> Self {
                let res = $reader(reading) / 100.0;
                #[cfg(feature = "defmt")]
                trace!("Converted raw output {} to {}", reading, res);
                Self(res)
            }
        }
        impl From<[u8; 3]> for $kind {
            fn from(reading: [u8; 3]) -> Self {
                <Self as From<&[u8]>>::from(&reading)
            }
        }
        #[cfg(feature = "defmt")]
        impl defmt::Format for $kind {
            fn format(&self, f: defmt::Formatter) {
                defmt::write!(f, "{}{}", self.0, $unit);
            }
        }
        }
    };
}

reading_impl!(Pressure, "mBar", read_unsigned);
reading_impl!(Altitude, "m", read_signed);
reading_impl!(Temperature, "°C", read_signed);

// TODO: more tests
#[cfg(test)]
mod tests {
    use super::*;
    use test_case::test_case;

    #[test_case(&[0x00, 0x0A, 0x5C], 26.52)]
    #[test_case(&[0xFF, 0xFC, 0x02], -10.22)]
    fn raw_to_temp(input: &[u8], expected: f32) {
        assert_eq!(
            <&[u8] as Into<Temperature>>::into(input),
            Temperature(expected)
        );
    }

    #[test_case(&[0x00, 0x13, 0x88], 50.0)]
    #[test_case(&[0xFF, 0xEC, 0x78], -50.0)]
    fn raw_to_alti(input: &[u8], expected: f32) {
        assert_eq!(<&[u8] as Into<Altitude>>::into(input), Altitude(expected));
    }

    #[test_case(&[0x01, 0x8a, 0x9e], 1010.22)]
    fn raw_to_pres(input: &[u8], expected: f32) {
        assert_eq!(<&[u8] as Into<Pressure>>::into(input), Pressure(expected));
    }
}
