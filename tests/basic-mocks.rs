use hp203b::*;

use csb::CSB as _;
use embedded_hal_mock::i2c::{Mock, Transaction};
use once_cell::sync::Lazy;
use rand::prelude::*;
use std::ops::Deref;

// We use CSBLow for all the tests since there's no point changing it in mocks
type CSB = csb::CSBLow;

type HP203B<M> = hp203b::HP203B<Mock, M, CSB>;

macro_rules! w {
    ($($byte:expr),*) => {
        Transaction::write(CSB::ADDR, vec![
            $($byte,)*
        ])
    };
}

macro_rules! wr {
    ($($wbyte:expr),* ; $($rbyte:expr),*) => {
        Transaction::write_read(
            CSB::ADDR,
            vec![
                $($wbyte,)*
            ],
            vec![
                $($rbyte,)*
            ],
        )
    }
}

macro_rules! r {
    ($($rbyte:expr),*) => {
        Transaction::read( CSB::ADDR, vec![ $($rbyte,)* ],)
    }
}

const OSR: hp203b::OSR = hp203b::OSR::OSR1024;
const CHAN: hp203b::Channel = hp203b::Channel::SensorPressureTemperature;
static NEW: Lazy<[Transaction; 4]> = Lazy::new(|| {
    [
        // SOFT_RST
        w!(0x06),
        // Check Ready
        wr!(0x80 + 0x0d; thread_rng().gen::<u8>() | 0b0100_0000),
        // Set OSR and channel
        w!(0x40 + OSR as u8 + CHAN as u8),
        // Disable all interrupts except the *_RDY ones
        w!(0xc0 + 0x0b, 0b0011_0000),
    ]
});

static READ_TEMP: Lazy<[Transaction; 3]> = Lazy::new(|| {
    [
        // READ_T
        w!(0x32),
        // Check temp is ready
        wr!(0x80 + 0x0d; 0b0001_0000),
        // Read value
        r!(0x00, 0x0A, 0x5C),
    ]
});
const TEMP_VAL: Temperature = Temperature(26.52);

static READ_PRES: Lazy<[Transaction; 3]> = Lazy::new(|| {
    [
        // READ_P
        w!(0x30),
        // Check pressure is ready
        wr!(0x80 + 0x0d; 0b0010_0000),
        // Read value
        r!(0x00, 0x0A, 0x5C),
    ]
});
const PRES_VAL: Pressure = Pressure(26.52);

static TO_ALTI: Lazy<[Transaction; 12]> = Lazy::new(|| {
    let mut rng = thread_rng();
    let enabled_flags = rng.gen();
    let pinout_flags = rng.gen();
    [
        // Set lower bound = 0
        w!(0xc0 + 0x06, 0),
        w!(0xc0 + 0x07, 0),
        // Set upper bound = 0
        w!(0xc0 + 0x02, 0),
        w!(0xc0 + 0x03, 0),
        // Set mid bound = 0
        w!(0xc0 + 0x04, 0),
        w!(0xc0 + 0x05, 0),
        // Set offset = 0
        w!(0xc0 + 0x00, 0),
        w!(0xc0 + 0x01, 0),
        // Correct interrupt flags
        wr!(0x80 + 0x0b; enabled_flags),
        w!(0xc0 + 0x0b, enabled_flags & 0b0011_0101),
        // Correct pinout flags
        wr!(0x80 + 0x0c; pinout_flags),
        w!(0xc0 + 0x0c, (pinout_flags | 0b0100_0000) & 0b0111_1111),
    ]
});

static READ_ALTI: Lazy<[Transaction; 3]> = Lazy::new(|| {
    [
        // READ_A
        w!(0x31),
        // Check alti is ready
        wr!(0x80 + 0x0d; 0b0010_0000),
        // Read value
        r!(0x00, 0x0A, 0x5C),
    ]
});
const ALTI_VAL: Altitude = Altitude(26.52);

macro_rules! altimeter {
    ($($seq:expr),*) => {{
        let mut __basic_seq = NEW.to_vec();
        $(
            __basic_seq.extend_from_slice($seq);
        )*
        HP203B::new(Mock::new(&__basic_seq), OSR, CHAN).unwrap()
    }};
}

#[test]
fn wait_ready() {
    let mut rng = thread_rng();

    for _ in 0..10 {
        let mut alti = altimeter!(&[wr!(0x80 + 0x0d; rng.gen::<u8>() | 0b0100_0000)]);
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
    let measurement = alti.read_temp_blocking().unwrap();
    assert_eq!(measurement, TEMP_VAL);
    alti.destroy().done();
}

#[test]
fn read_pressure() {
    let mut alti = altimeter!(READ_PRES.deref());
    let measurement = alti.read_pres_blocking().unwrap();
    assert_eq!(measurement, PRES_VAL);
    alti.destroy().done();
}

#[test]
fn read_alti() {
    let mut alti = altimeter!(TO_ALTI.deref(), READ_ALTI.deref())
        .to_altitude()
        .unwrap();
    let measurement = alti.read_alti_blocking().unwrap();
    assert_eq!(measurement, ALTI_VAL);
    alti.destroy().done();
}
