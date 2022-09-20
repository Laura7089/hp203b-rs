#![no_std]
#![no_main]

// transport layer for defmt logs
use defmt_rtt as _;
// panicking behavior
use hp203b as _;
use panic_probe as _;

#[link_section = ".boot_loader"]
#[used]
pub static BOOT_LOADER: [u8; 256] = rp2040_boot2::BOOT_LOADER_W25Q080;

use fugit::RateExtU32;
use hal::{gpio, i2c, pac};
use rp2040_hal as hal;

type I2CPin<P> = gpio::Pin<P, gpio::Function<gpio::I2C>>;
type I2C = i2c::I2C<pac::I2C0, (I2CPin<gpio::bank0::Gpio16>, I2CPin<gpio::bank0::Gpio17>)>;

fn setup() -> I2C {
    let mut perips = pac::Peripherals::take().unwrap();
    let sio = hal::Sio::new(perips.SIO);
    let pins = hal::gpio::Pins::new(
        perips.IO_BANK0,
        perips.PADS_BANK0,
        sio.gpio_bank0,
        &mut perips.RESETS,
    );

    hal::I2C::i2c0(
        perips.I2C0,
        pins.gpio16.into_mode(),
        pins.gpio17.into_mode(),
        400.kHz(),
        &mut perips.RESETS,
        125_000_000.Hz(),
    )
}

#[defmt_test::tests]
mod tests {
    use super::{setup, I2C};
    use defmt::assert;
    #[allow(unused_imports)]
    use hp203b::interrupts::HasInterrupts;
    use hp203b::{Channel, OSR};

    type HP203B<M> = hp203b::HP203B<I2C, M, hp203b::csb::CSBLow>;

    #[test]
    fn always_passes() {
        assert!(true);
    }

    #[test]
    fn make_new() {
        let _: HP203B<hp203b::mode::Pressure> =
            HP203B::new(setup(), OSR::OSR128, Channel::SensorPressureTemperature)
                .expect("HP203B::new returned an error");
    }
}
