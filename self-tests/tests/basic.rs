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
    use fugit::RateExtU32;
    use hal::{gpio, i2c, pac};
    use rp2040_hal as hal;
    use shared_bus::BusManagerSimple;

    type I2CPin<P> = gpio::Pin<P, gpio::Function<gpio::I2C>>;
    type I2C = i2c::I2C<pac::I2C0, (I2CPin<gpio::bank0::Gpio16>, I2CPin<gpio::bank0::Gpio17>)>;

    type HP203B<I, M> = hp203b::HP203B<I, M, hp203b::csb::CSBHigh>;

    #[init]
    fn get_i2c() -> BusManagerSimple<I2C> {
        let mut perips = pac::Peripherals::take().unwrap();
        let sio = hal::Sio::new(perips.SIO);
        let pins = hal::gpio::Pins::new(
            perips.IO_BANK0,
            perips.PADS_BANK0,
            sio.gpio_bank0,
            &mut perips.RESETS,
        );

        BusManagerSimple::new(hal::I2C::i2c0(
            perips.I2C0,
            pins.gpio16.into_mode(),
            pins.gpio17.into_mode(),
            400.kHz(),
            &mut perips.RESETS,
            125_000_000.Hz(),
        ))
    }

    #[test]
    fn make_new(i2c: &mut BusManagerSimple<I2C>) {
        HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
        )
        .unwrap();
    }

    #[test]
    fn is_ready(i2c: &mut BusManagerSimple<I2C>) {
        let mut alti = HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
        )
        .unwrap();
        alti.is_ready().unwrap();
    }

    #[test]
    fn read_pressure(i2c: &mut BusManagerSimple<I2C>) {
        let mut alti = HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
        )
        .unwrap();
        let pres = nb::block!(alti.read_pres()).unwrap();
        info!("Pressure reading: {}Pa", pres);

        assert!(pres > 0.0);
    }

    #[test]
    fn read_alti(i2c: &mut BusManagerSimple<I2C>) {
        let mut alti = HP203B::new(
            i2c.acquire_i2c(),
            OSR::OSR1024,
            Channel::SensorPressureTemperature,
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
