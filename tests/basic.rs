use hp203b::*;

use csb::CSB;
use embedded_hal_mock::i2c::{Mock, Transaction as Tr};
use once_cell::sync::Lazy;
use std::ops::Deref;

// We use CSBLow for all the tests since there's no point changing it in mocks
type HP203B<M> = hp203b::HP203B<Mock, M, csb::CSBLow>;

const OSR: hp203b::OSR = hp203b::OSR::OSR1024;
const CHAN: hp203b::Channel = hp203b::Channel::SensorPressureTemperature;
static NEW: Lazy<[Tr; 5]> = Lazy::new(|| {
    [
        Tr::write(csb::CSBLow::ADDR, vec![0x06]), // SOFT_RST
        // Check Ready
        Tr::write_read(csb::CSBLow::ADDR, vec![0x80 + 0x0d], vec![0b0100_0000]),
        // Set OSR and channel
        Tr::write(csb::CSBLow::ADDR, vec![0x40 + OSR as u8 + CHAN as u8]),
        // Disable all interrupts
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x0b, 0b0011_0000]),
        // Disable all interrupt pinout
        Tr::write(csb::CSBLow::ADDR, vec![0xc0 + 0x0c, 0]),
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
const PRES_VAL: f32 = 26.52;

#[test]
fn create_new() {
    let alti = HP203B::new(Mock::new(NEW.deref()), OSR, CHAN).unwrap();
    alti.destroy().done();
}

#[test]
fn read_pressure() {
    let osr = OSR::OSR1024;
    let chan = Channel::SensorPressureTemperature;

    let mut mock_seq = NEW.to_vec();
    mock_seq.extend_from_slice(READ_PRES.deref());
    let mock = Mock::new(&mock_seq);

    let mut alti = HP203B::new(mock, osr, chan).unwrap();
    assert_eq!(nb::block!(alti.read_pres()).unwrap(), PRES_VAL);

    alti.destroy().done();
}