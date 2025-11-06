#![no_main]
#![no_std]

use core::time::Duration;

use vexide::{prelude::*, time};

const BAUD_RATE: u32 = 115200;
const MOTOR_PACKET_MAGIC: u16 = 0xFEFA;
const MOTOR_POWER_MAX: f64 = 1.0;
const MOTOR_VOLTAGE_MAX: f64 = 12.0;

#[repr(C)]
struct MotorPacket {
    front_left: f64,
    front_right: f64,
    back_left: f64,
    back_right: f64,
}

fn get_motor_packet(rx_port: &mut SerialPort) -> Option<MotorPacket> {
    const TIMEOUT: Duration = Duration::from_secs(1);

    let start_time = time::Instant::now();
    while time::Instant::now().duration_since(start_time) < TIMEOUT {
        // Check for the whole magic, byte by byte
        if MOTOR_PACKET_MAGIC
            .to_le_bytes()
            .iter()
            .all(|x| rx_port.read_byte() == Some(*x))
        {
            let mut packet = [0u8; core::mem::size_of::<MotorPacket>()];
            if rx_port.read_exact(&mut packet).is_ok() {
                return Some(unsafe { core::mem::transmute::<[u8; _], MotorPacket>(packet) });
            }
        }
    }

    None
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let rx_smart_port: SmartPort = peripherals.port_1;
    let tx_smart_port: SmartPort = peripherals.port_2;

    let mut rx_serial = SerialPort::open(rx_smart_port, BAUD_RATE).await;
    let mut _tx_serial = SerialPort::open(tx_smart_port, BAUD_RATE).await;
    let mut front_lefts: [Motor; _] = [
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
    ];
    let mut front_rights: [Motor; 2] = [
        Motor::new(peripherals.port_5, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_6, Gearset::Green, Direction::Forward),
    ];
    let mut back_lefts: [Motor; 2] = [
        Motor::new(peripherals.port_7, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_8, Gearset::Green, Direction::Forward),
    ];
    let mut back_rights: [Motor; 2] = [
        Motor::new(peripherals.port_9, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_10, Gearset::Green, Direction::Forward),
    ];

    loop {
        if let Some(packet) = get_motor_packet(&mut rx_serial) {
            front_lefts.iter_mut().for_each(|m| {
                m.set_voltage(packet.front_left * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
            front_rights.iter_mut().for_each(|m| {
                m.set_voltage(packet.front_right * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
            back_lefts.iter_mut().for_each(|m| {
                m.set_voltage(packet.back_left * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
            back_rights.iter_mut().for_each(|m| {
                m.set_voltage(packet.back_right * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
        }
    }
}
