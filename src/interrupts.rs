//! Types and methods for configuring and receiving interrupts on the altimeter
//!
//! The [`HasInterrupts`] trait is the place to start here - it contains all the fancy methods for
//! getting and configuring the interrupts from the device.
//!
//! ## Example
// TODO: make this runnable
//! ```no_run
//! use hp203b::interrupts::{Event, HasInterrupts, InterruptSetting};
//! # use hp203b::{HP203B, OSR, Channel};
//! # use embedded_hal_mock::{i2c::Mock, delay::StdSleep};
//! # fn main() -> Result<(), embedded_hal::i2c::ErrorKind> {
//! # let mut delay = StdSleep::default();
//! # let mut alti: HP203B<_, _, hp203b::csb::CSBLow> = HP203B::new(
//! #     Mock::new(&[]),
//! #     OSR::OSR128,
//! #     Channel::Temperature,
//! #     &mut delay,
//! # )?;
//!
//! alti.set_pres_mid(100.0)?;
//! alti.setup_interrupts([(Event::PATraversed, InterruptSetting::Enabled)])?;
//!
//! let _ = alti.read_pres()?;
//!
//! // Let's check if it moved across our midpoint
//! for int in alti.interrupts()? {
//!     if int == Event::PATraversed {
//!         println!("The pressure reading crossed the midpoint!");
//!     }
//! }
//! # Ok(())
//! # }
//! ```
//!
//! ## Note: `INT1` pin
//!
//! This crate does not have any mechanism for dealing with the interrupt pin from the altimeter, defined as `INT1` in the datasheet.
//! You must watch that pin yourself, and call [`HP203B::interrupts`] and related functions as
//! appropriate for your use case.

use core::iter::FusedIterator;

use crate::flags::{self, Flags, INT_CFG, INT_DIR, INT_EN};
use crate::HP203B;

/// An interrupt-based event sent by the Altimeter
// TODO: allow users to set pinout on the *_RDY flags, but not disable them entirely -
// alternatively, type-model whether they're enabled (will make very verbose)
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum Event {
    /// The pressure (or altitude) value has traversed the middle threshold during the last measurement
    PATraversed,
    /// The temperature value has traversed the middle threshold during the last measurement
    TemperatureTraversed,
    /// The pressure (or altitude) value were outside the pre-defined window  during the last measurement
    ///
    /// Here, the "window" is the range between the upper bound and lower bound thresholds.
    PAOutsideWindow,
    /// The temperature value locates outside the pre-defined window during the last measurement
    ///
    /// The "window" is the same as above.
    TemperatureOutsideWindow,
}

#[allow(clippy::from_over_into)]
impl Into<flags::INT_EN> for Event {
    fn into(self) -> flags::INT_EN {
        use Event::*;

        match self {
            PATraversed => INT_EN::PA_TRAV_EN,
            TemperatureTraversed => INT_EN::T_TRAV_EN,
            PAOutsideWindow => INT_EN::PA_WIN_EN,
            TemperatureOutsideWindow => INT_EN::T_WIN_EN,
        }
    }
}

#[allow(clippy::from_over_into)]
impl Into<flags::INT_CFG> for Event {
    fn into(self) -> flags::INT_CFG {
        use Event::*;

        match self {
            PATraversed => INT_CFG::PA_TRAV_CFG,
            TemperatureTraversed => INT_CFG::T_TRAV_CFG,
            PAOutsideWindow => INT_CFG::PA_WIN_CFG,
            TemperatureOutsideWindow => INT_CFG::T_WIN_CFG,
        }
    }
}

/// Configuration state for a single interrupt
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum InterruptSetting {
    /// Disabled, regardless of interrupt pinout setting
    Disabled,
    /// Can be triggered, but will not set the interrupt pin
    Enabled,
    /// Enabled, and will set the interrupt pin when triggered
    EnabledWithPin,
}

/// Whether a measurement has exited the set window above or below
///
/// Only relevant to [`Event::PAOutsideWindow`] and
/// [`Event::TemperatureOutsideWindow`].
#[allow(missing_docs)]
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum WindowPosition {
    AboveWindow,
    BelowWindow,
}

/// Direction of a measurement's change
///
/// Only relevant to [`Event::PATraversed`] and [`Event::TemperatureTraversed`].
#[derive(Copy, Clone, Debug, PartialEq, Eq)]
pub enum TraversalDirection {
    /// The value is rising from low to high
    Rising,
    /// The value is falling from high to low
    Falling,
}

/// An [`Iterator`] over interrupts ([`Event`]s) set on the device
/// ```no_run
/// use hp203b::interrupts::{Event, HasInterrupts};
/// use hp203b::HP203B;
///
/// # use hp203b::{csb::{CSBLow, CSB}, OSR, Channel};
/// # use embedded_hal_mock::i2c::{Mock, Transaction as Tr};
/// # use embedded_hal_mock::delay::MockNoop;
/// # fn main() -> Result<(), embedded_hal::i2c::ErrorKind> {
/// # let mut delay = MockNoop::default();
/// # let mut alti: HP203B<_, _, CSBLow> = HP203B::new(
/// #   Mock::new(&[]),
/// #   OSR::OSR128,
/// #   Channel::Temperature,
/// #   &mut delay,
/// # )?;
///
/// for event in alti.interrupts()? {
///     match event {
///         Event::PATraversed => {
///             // Do something
///         },
///         // ...
///         _ => continue,
///     }
/// }
/// # Ok(())
/// # }
/// ```
#[derive(Clone)]
pub struct Interrupts(flags::INT_SRC);

impl Interrupts {
    pub(crate) fn new(flags: flags::INT_SRC) -> Self {
        Self(flags)
    }
}

impl Iterator for Interrupts {
    type Item = Event;

    fn next(&mut self) -> Option<Self::Item> {
        use flags::INT_SRC;

        for flag in crate::flags::INT_SRC_VARIANTS {
            if self.0.contains(flag) {
                self.0.remove(flag);
                return Some(match flag {
                    INT_SRC::PA_TRAV => Event::PATraversed,
                    INT_SRC::PA_WIN => Event::PAOutsideWindow,
                    INT_SRC::T_TRAV => Event::TemperatureTraversed,
                    INT_SRC::T_WIN => Event::TemperatureOutsideWindow,
                    _ => continue,
                });
            }
        }
        None
    }
}
impl FusedIterator for Interrupts {}

/// Interrupt-related methods on the altimeter
#[allow(clippy::module_name_repetitions)]
pub trait HasInterrupts<E> {
    /// Get the [`Interrupts`] set
    fn interrupts(&mut self) -> Result<Interrupts, E>;

    /// Get the settings for a particular [`Event`]
    fn interrupt_setting(&mut self, event: Event) -> Result<InterruptSetting, E>;

    /// Configure the interrupts settings on the altimeter
    ///
    /// Takes an [`IntoIterator`] of tuple pairs representing a mapping of
    /// `interrupt_type`: `setting`.
    fn setup_interrupts<S: IntoIterator<Item = (Event, InterruptSetting)>>(
        &mut self,
        settings: S,
    ) -> Result<(), E>;

    /// Check the traversal direction of the pressure/altitude measurement
    ///
    /// Only relevant to [`Event::PATraversed`].
    /// If this isn't enabled on the chip, returns `Ok(None)`.
    ///
    /// # Note
    ///
    /// TODO: is this true?
    ///
    /// This has little meaning if you have not configured the traversal threshold for
    /// pressure/altitude.
    fn pa_traverse_direction(&mut self) -> Result<Option<TraversalDirection>, E>;

    /// Check the traversal direction of the temperature measurement
    ///
    /// Only relevant to [`Event::PATraversed`].
    /// If this isn't enabled on the chip, returns `Ok(None)`.
    ///
    /// # Note
    ///
    /// TODO: is this true?
    ///
    /// This has little meaning if you have not configured the traversal threshold for temperature.
    fn temp_traverse_direction(&mut self) -> Result<Option<TraversalDirection>, E>;

    /// Check the position of the baro measurement relative to the set window
    ///
    /// Only relevant to [`Event::PAOutsideWindow`].
    /// If this isn't enabled on the chip, returns `Ok(None)`.
    ///
    /// # Note
    ///
    /// TODO: is this true?
    ///
    /// This has little meaning if you have not configured the window thresholds for
    /// pressure/altitude.
    fn pa_window_status(&mut self) -> Result<Option<WindowPosition>, E>;

    /// Check the position of the temperature measurement relative to the set window
    ///
    /// Only relevant to [`Event::TemperatureOutsideWindow`].
    /// If this isn't enabled on the chip, returns `Ok(None)`.
    ///
    /// # Note
    ///
    /// TODO: is this true?
    ///
    /// This has little meaning if you have not configured the window thresholds for temperature.
    fn temp_window_status(&mut self) -> Result<Option<WindowPosition>, E>;
}

impl<I, M, C, E> HasInterrupts<E> for HP203B<I, M, C>
where
    I: embedded_hal::i2c::blocking::I2c<Error = E>,
    M: crate::mode::BarometricMeasurement,
    C: crate::csb::CSB,
{
    fn interrupts(&mut self) -> Result<Interrupts, E> {
        Ok(Interrupts::new(self.get_interrupts()?))
    }

    fn setup_interrupts<S: IntoIterator<Item = (Event, InterruptSetting)>>(
        &mut self,
        settings: S,
    ) -> Result<(), E> {
        let mut enabled_flags = self.get_interrupts_enabled()?;
        let mut pinout_flags = self.get_interrupts_pinout()?;

        for (event, setting) in settings {
            let en_flag = event.into();
            let pinout_flag = event.into();

            let (en, pinout) = match setting {
                InterruptSetting::Enabled => (true, false),
                InterruptSetting::EnabledWithPin => (true, true),
                InterruptSetting::Disabled => (false, false),
            };

            enabled_flags.set(en_flag, en);
            pinout_flags.set(pinout_flag, pinout);
        }

        self.set_interrupts_enabled(enabled_flags)?;
        self.set_interrupts_pinout(pinout_flags)?;

        Ok(())
    }

    fn interrupt_setting(&mut self, event: Event) -> Result<InterruptSetting, E> {
        let enabled = self.get_interrupts_enabled()?.contains(event.into());
        let pinout = self.get_interrupts_pinout()?.contains(event.into());

        Ok(match (enabled, pinout) {
            (true, true) => InterruptSetting::EnabledWithPin,
            (true, false) => InterruptSetting::Enabled,
            (false, _) => InterruptSetting::Disabled,
        })
    }

    fn pa_traverse_direction(&mut self) -> Result<Option<TraversalDirection>, E> {
        if self.interrupt_setting(Event::PATraversed)? == InterruptSetting::Disabled {
            return Ok(None);
        }

        let extras = self.get_interrupt_extras()?;

        Ok(Some(if extras.contains(INT_DIR::P_TRAV_DIR) {
            TraversalDirection::Rising
        } else {
            TraversalDirection::Falling
        }))
    }

    fn temp_traverse_direction(&mut self) -> Result<Option<TraversalDirection>, E> {
        if self.interrupt_setting(Event::TemperatureTraversed)? == InterruptSetting::Disabled {
            return Ok(None);
        }

        let extras = self.get_interrupt_extras()?;

        Ok(Some(if extras.contains(INT_DIR::P_TRAV_DIR) {
            TraversalDirection::Rising
        } else {
            TraversalDirection::Falling
        }))
    }

    fn pa_window_status(&mut self) -> Result<Option<WindowPosition>, E> {
        if self.interrupt_setting(Event::PATraversed)? == InterruptSetting::Disabled {
            return Ok(None);
        }

        let extras = self.get_interrupt_extras()?;

        Ok(Some(if extras.contains(INT_DIR::P_WIN_DIR) {
            WindowPosition::AboveWindow
        } else {
            WindowPosition::BelowWindow
        }))
    }

    fn temp_window_status(&mut self) -> Result<Option<WindowPosition>, E> {
        if self.interrupt_setting(Event::TemperatureTraversed)? == InterruptSetting::Disabled {
            return Ok(None);
        }

        let extras = self.get_interrupt_extras()?;

        Ok(Some(if extras.contains(INT_DIR::T_WIN_DIR) {
            WindowPosition::AboveWindow
        } else {
            WindowPosition::BelowWindow
        }))
    }
}
