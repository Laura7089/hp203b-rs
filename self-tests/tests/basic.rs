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

#[defmt_test::tests]
mod tests {
    use hp203b::{Channel, OSR};

    use defmt::info;
    use embedded_hal::delay::blocking::DelayUs;
    use embedded_hal_compat::ForwardCompat;
    use fugit::RateExtU32;
    use hal::{gpio, i2c, pac};
    use rp2040_hal as hal;
    use shared_bus::BusManagerSimple;

    type I2CPin<P> = gpio::Pin<P, gpio::Function<gpio::I2C>>;
    type I2C = BusManagerSimple<
        i2c::I2C<pac::I2C0, (I2CPin<gpio::bank0::Gpio16>, I2CPin<gpio::bank0::Gpio17>)>,
    >;
    type Delay = embedded_hal_compat::Forward<cortex_m::delay::Delay>;

    type HP203B<I, M> = hp203b::HP203B<I, M, hp203b::csb::CSBHigh>;

    #[init]
    fn setup() -> (I2C, Delay) {
        let mut perips = pac::Peripherals::take().unwrap();
        let core_perips = pac::CorePeripherals::take().unwrap();
        let sio = hal::Sio::new(perips.SIO);
        let pins = hal::gpio::Pins::new(
            perips.IO_BANK0,
            perips.PADS_BANK0,
            sio.gpio_bank0,
            &mut perips.RESETS,
        );

        let mut delay = cortex_m::delay::Delay::new(core_perips.SYST, 100).forward();
        delay.delay_ms(1000).unwrap();

        (
            BusManagerSimple::new(hal::I2C::i2c0(
                perips.I2C0,
                pins.gpio16.into_mode(),
                pins.gpio17.into_mode(),
                400.kHz(),
                &mut perips.RESETS,
                125_000_000.Hz(),
            )),
            delay,
        )
    }

    #[test]
    fn make_new((i2c, delay): &mut (I2C, Delay)) {
        HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
            delay,
        )
        .unwrap();
    }

    #[test]
    fn is_ready((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
            delay,
        )
        .unwrap();
        alti.is_ready().unwrap();
    }

    #[test]
    fn read_pressure((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
            delay,
        )
        .unwrap();
        let pres = nb::block!(alti.read_pres()).unwrap();
        info!("Pressure reading: {}Pa", pres);

        assert!(pres.0 > 0.0);
    }

    #[test]
    fn read_alti((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
            delay,
        )
        .unwrap()
        .to_altitude()
        .unwrap();
        info!(
            "Altitude reading: {}m",
            nb::block!(alti.read_alti()).unwrap()
        );
    }
}
