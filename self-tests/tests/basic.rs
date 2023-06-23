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

macro_rules! test_alti {
    ($i2c:expr, $delay:expr) => {{
        let a = HP203B::new($i2c, DEF_OSR, DEF_CHANNEL).unwrap();
        let d = a.read_delay().to_millis();
        debug!("Read delay: {}ms", d);
        $delay.delay_ms(d);
        a
    }};
    ($i2c:expr, $mult:expr => $delay:expr) => {{
        let a = HP203B::new($i2c, DEF_OSR, DEF_CHANNEL).unwrap();
        let d = a.read_delay().to_millis() * $mult;
        debug!("Read delay: {}ms", d);
        $delay.delay_ms(d);
        a
    }};
}

#[defmt_test::tests]
mod tests {
    use cortex_m::delay::Delay;
    use defmt::{debug, info};
    use fugit::RateExtU32;
    use hal::{gpio, i2c, pac};
    use rp2040_hal as hal;

    use hp203b::ReadGuard;

    type I2CPin<P> = gpio::Pin<P, gpio::Function<gpio::I2C>>;
    type I2C = i2c::I2C<pac::I2C0, (I2CPin<gpio::bank0::Gpio16>, I2CPin<gpio::bank0::Gpio17>)>;

    type HP203B<I, M> = hp203b::HP203B<I, M, hp203b::csb::CSBHigh>;

    const DEF_OSR: hp203b::OSR = hp203b::OSR::OSR1024;
    const DEF_CHANNEL: hp203b::Channel = hp203b::Channel::SensorPressureTemperature;

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

        let mut delay = cortex_m::delay::Delay::new(core_perips.SYST, 100);
        debug!("Waiting for 400ms");
        delay.delay_ms(400);

        (
            hal::I2C::i2c0(
                perips.I2C0,
                pins.gpio16.into_mode(),
                pins.gpio17.into_mode(),
                400.kHz(),
                &mut perips.RESETS,
                125.MHz(),
            ),
            delay,
        )
    }

    #[test]
    fn make_new((i2c, delay): &mut (I2C, Delay)) {
        test_alti!(i2c, delay);
    }

    #[test]
    fn read_temp((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = test_alti!(i2c, delay);
        let mut guard = alti.read_temp().unwrap();
        let t = nb::block!(guard.try_take()).unwrap();
        info!("Temperature reading: {}", t);
        assert!(t.0 != 655.35); // From an issue we previously had
    }

    #[test]
    fn read_pressure((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = test_alti!(i2c, delay);
        let mut guard = alti.read_pres().unwrap();
        let p = nb::block!(guard.try_take()).unwrap();
        info!("Pressure reading: {}", p);
        assert!(p.0 >= 0.0);
    }

    #[test]
    fn read_both_pres((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = test_alti!(i2c, 2 => delay);
        let mut guard = alti.read_pres_temp().unwrap();
        let (t, p) = nb::block!(guard.try_take()).unwrap();
        info!("Pressure: {}, temp: {}", p, t);
    }

    #[test]
    fn read_alti((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = test_alti!(i2c, delay).to_altitude().unwrap();
        info!("Altitude reading: {}", alti.read_alti().unwrap());
    }

    #[test]
    fn read_both_alti((i2c, delay): &mut (I2C, Delay)) {
        let mut alti = test_alti!(i2c, delay).to_altitude().unwrap();
        let mut guard = alti.read_alti_temp().unwrap();
        let (t, a) = nb::block!(guard.try_take()).unwrap();
        info!("Altitude: {}, temp: {}", a, t);
    }
}
