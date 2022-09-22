use hp203b::*;

use csb::CSB;
use embedded_hal_mock::delay::MockNoop;
use embedded_hal_mock::i2c::{Mock, Transaction as Tr};
use once_cell::sync::Lazy;
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
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0100_0000]),
        // Set OSR and channel
        Tr::write(csb::CSBLow::ADDR, vec![0x40 + OSR as u8 + CHAN as u8]),
        // Disable all interrupts except the *_RDY ones
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x0b, 0b0011_0000]),
    ]
});

static READ_PRES: Lazy<[Tr; 3]> = Lazy::new(|| {
    [
        // READ_P
        Tr::write(csb::CSBLow::ADDR, vec![0x30]),
        // Check pressure is ready
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0010_0000]),
        // Read pressure
        Tr::read(csb::CSBLow::ADDR, vec![0x00, 0x0A, 0x5C]),
    ]
});
const PRES_VAL: Pressure = Pressure(26.52);

static READ_ALTI: Lazy<[Tr; 3]> = Lazy::new(|| {
    [
        // READ_A
        Tr::write(csb::CSBLow::ADDR, vec![0x31]),
        // Check pressure is ready
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0010_0000]),
        // Read pressure
        Tr::read(csb::CSBLow::ADDR, vec![0x00, 0x0A, 0x5C]),
    ]
});

#[test]
fn create_new() {
    let mut delay = MockNoop::default();
    let alti = HP203B::new(Mock::new(NEW.deref()), OSR, CHAN, &mut delay).unwrap();
    alti.destroy().done();
}

#[test]
fn read_pressure() {
    let mut delay = MockNoop::default();

    let mut mock_seq = NEW.to_vec();
    mock_seq.extend_from_slice(READ_PRES.deref());
    let mock = Mock::new(&mock_seq);

    let mut alti = HP203B::new(mock, OSR, CHAN, &mut delay).unwrap();
    assert_eq!(nb::block!(alti.read_pres()).unwrap(), PRES_VAL);

    alti.destroy().done();
}
