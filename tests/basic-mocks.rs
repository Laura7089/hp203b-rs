use hp203b::*;

use csb::CSB;
use embedded_hal_mock::delay::MockNoop;
use embedded_hal_mock::i2c::{Mock, Transaction as Tr};
use once_cell::sync::Lazy;
use rand::prelude::*;
use std::ops::Deref;

// We use CSBLow for all the tests since there's no point changing it in mocks
type HP203B<M> = hp203b::HP203B<Mock, M, csb::CSBLow>;

const OSR: hp203b::OSR = hp203b::OSR::OSR1024;
const CHAN: hp203b::Channel = hp203b::Channel::SensorPressureTemperature;
static NEW: Lazy<[Tr; 4]> = Lazy::new(|| {
    [
        // SOFT_RST
        Tr::write(csb::CSBLow::ADDR, vec![0x06]),
        // Check Ready
        Tr::write_read(
            csb::CSBLow::ADDR,
            vec![0x80 + 0x0d],
            vec![thread_rng().gen::<u8>() | 0b0100_0000],
        ),
        // Set OSR and channel
        Tr::write(csb::CSBLow::ADDR, vec![0x40 + OSR as u8 + CHAN as u8]),
        // Disable all interrupts except the *_RDY ones
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x0b, 0b0011_0000]),
    ]
});

static READ_TEMP: Lazy<[Tr; 2]> = Lazy::new(|| {
    [
        // Check temp is ready
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0001_0000]),
        // READ_T
        Tr::write_read(csb::CSBLow::ADDR, vec![0x32], vec![0x00, 0x0A, 0x5C]),
    ]
});
const TEMP_VAL: Temperature = Temperature(26.52);

static READ_PRES: Lazy<[Tr; 2]> = Lazy::new(|| {
    [
        // Check pressure is ready
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0010_0000]),
        // READ_P
        Tr::write_read(csb::CSBLow::ADDR, vec![0x30], vec![0x00, 0x0A, 0x5C]),
    ]
});
const PRES_VAL: Pressure = Pressure(26.52);

static TO_ALTI: Lazy<[Tr; 12]> = Lazy::new(|| {
    let mut rng = thread_rng();
    let enabled_flags = rng.gen();
    let pinout_flags = rng.gen();
    [
        // Set lower bound = 0
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x06, 0]),
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x07, 0]),
        // Set upper bound = 0
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x02, 0]),
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x03, 0]),
        // Set mid bound = 0
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x04, 0]),
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x05, 0]),
        // Set offset = 0
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x00, 0]),
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x01, 0]),
        // Correct interrupt flags
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0b], vec![enabled_flags]),
        Tr::write(
            csb::CSBLow::ADDR,
            vec![0xc0 + 0x0b, enabled_flags & 0b0011_0101],
        ),
        // Correct pinout flags
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0c], vec![pinout_flags]),
        Tr::write(
            csb::CSBLow::ADDR,
            vec![0xc0 + 0x0c, (pinout_flags | 0b0100_0000) & 0b0111_1111],
        ),
    ]
});

static READ_ALTI: Lazy<[Tr; 2]> = Lazy::new(|| {
    [
        // Check alti is ready
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0010_0000]),
        // Read alti
        Tr::write_read(csb::CSBLow::ADDR, vec![0x31], vec![0x00, 0x0A, 0x5C]),
    ]
});
const ALTI_VAL: Altitude = Altitude(26.52);

macro_rules! altimeter {
    ($($seg:expr),*) => {{
        let mut __delay = MockNoop::default();
        let mut __basic_seq = NEW.to_vec();
        $(
            __basic_seq.extend_from_slice($seg);
        )*
        HP203B::new(Mock::new(&__basic_seq), OSR, CHAN, &mut __delay).unwrap()
    }};
}

#[test]
fn wait_ready() {
    let mut rng = thread_rng();

    for _ in 0..10 {
        let mut alti = altimeter!(&[Tr::write_read(
            csb::CSBLow::ADDR,
            vec![0x80 + 0x0d],
            vec![rng.gen::<u8>() | 0b0100_0000],
        )]);
        alti.wait_ready().unwrap();
        alti.destroy().done();
    }
}

#[test]
fn create_new() {
    let alti = altimeter!();
    alti.destroy().done();
}

#[test]
fn read_temp() {
    let mut alti = altimeter!(READ_TEMP.deref());
    assert_eq!(alti.read_temp().unwrap(), TEMP_VAL);
    alti.destroy().done();
}

#[test]
fn read_pressure() {
    let mut alti = altimeter!(READ_PRES.deref());
    assert_eq!(alti.read_pres().unwrap(), PRES_VAL);
    alti.destroy().done();
}

#[test]
fn read_alti() {
    let mut alti = altimeter!(TO_ALTI.deref(), READ_ALTI.deref())
        .to_altitude()
        .unwrap();
    assert_eq!(alti.read_alti().unwrap(), ALTI_VAL);
    alti.destroy().done();
}
