use crate::mutex::Mutex;
use crate::once_cell::OnceCell;
use crate::time::Instant;
use crate::twi::TWI;
use core::time::Duration;

// Register Access
const MAGNETOMETER_ADDR: u8 = 0x1E; // 0b0011110

const REG_CR_A: u8 = 0x00; // Configuration register A
const REG_CR_B: u8 = 0x01; // Configuration register B
const REG_MR: u8 = 0x02; // Mode register

//TODO Double check these
const REG_OUT_X_H: u8 = 0x03; // X axis high byte
const REG_OUT_X_L: u8 = 0x04; // X axis low byte
const REG_OUT_Y_H: u8 = 0x05; // Y axis high byte
const REG_OUT_Y_L: u8 = 0x06; // Y axis low byte
const REG_OUT_Z_H: u8 = 0x07; // Z axis high byte
const REG_OUT_Z_L: u8 = 0x08; // Z axis low byte

const REG_SR: u8 = 0x09; // Status register
const ID_REG_A: u8 = 0x0A; // Identification register A
const ID_REG_B: u8 = 0x0B; // Identification register B
const ID_REG_C: u8 = 0x0C; // Identification register C
#[allow(dead_code)]
struct Hmc5883l {
    x: i16,
    y: i16,
    z: i16,
    last_measurement_time: Instant,
}

static MAGNETOMETER: Mutex<OnceCell<Hmc5883l>> = Mutex::new(OnceCell::uninitialized());

pub(crate) fn initialize() {
    // Safety: The TWI mutex are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };

    // twi.write(MAGNETOMETER_ADDR, REG_CR_A, &[]); // Send a "write" command to move the address pointer to register 0x00
    // twi.write(MAGNETOMETER_ADDR, REG_CR_A, &[0x70]); // 8 samples averaged, 15 Hz output
    // twi.write(MAGNETOMETER_ADDR, REG_CR_B, &[0x20]); // 1.3 gain
    // twi.write(MAGNETOMETER_ADDR, REG_MR, &[0x00]); // Continuous measurement mode

    MAGNETOMETER.modify(|compass| {
        compass.initialize(Hmc5883l {
            x: 0,
            y: 0,
            z: 0,
            last_measurement_time: Instant::now(),
        });
    });
}

fn update() {
    // Safety: The TWI and BAROMETER mutexes are not accessed in an interrupt
    let twi = unsafe { TWI.no_critical_section_lock_mut() };
    let compass = unsafe { MAGNETOMETER.no_critical_section_lock_mut() };

    let now = Instant::now();

    if now - compass.last_measurement_time < Duration::from_millis(100) {
        return;
    }

    compass.last_measurement_time = now;

    let mut data = [0u8; 6];
    _ = twi.read(MAGNETOMETER_ADDR, REG_OUT_X_H, &mut data); // Read all 6 registers at once

    compass.x = ((data[0] as i16) << 8) | data[1] as i16;
    compass.z = ((data[2] as i16) << 8) | data[3] as i16;
    compass.y = ((data[4] as i16) << 8) | data[5] as i16;
}

/// Read the compass field in the x, y, and z directions.
pub fn read_field() -> [i16; 3] {
    update();

    let compass = unsafe { MAGNETOMETER.no_critical_section_lock_mut() };

    [compass.x, compass.y, compass.z]
}
