#![no_main]
#![no_std]

use core::time::Duration;

use bytemuck::{Pod, Zeroable, bytes_of, from_bytes};
use vexide::{io::Result, prelude::*, time};

const BAUD_RATE: u32 = 115200;
const MOTOR_PACKET_MAGIC: u16 = 0xFEFA;
const ENCODER_PACKET_MAGIC: u16 = 0xF23B;
const MOTOR_POWER_MAX: f64 = 1.0;
const MOTOR_VOLTAGE_MAX: f64 = 12.0;

#[derive(Clone, Copy, Pod, Debug)]
#[repr(C)]
struct MotorPacket {
    front_left: f64,
    front_right: f64,
    back_left: f64,
    back_right: f64,
}

unsafe impl Zeroable for MotorPacket {
    fn zeroed() -> Self {
        MotorPacket {
            front_left: 0.,
            front_right: 0.,
            back_left: 0.,
            back_right: 0.,
        }
    }
}

fn get_power_packet(rx_port: &mut SerialPort) -> Option<MotorPacket> {
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
                return Some(*from_bytes(&packet));
            }
        }
    }

    None
}

fn send_encoder_packet(tx_port: &mut SerialPort, packet: &MotorPacket) -> Result<()> {
    tx_port.write_all(&ENCODER_PACKET_MAGIC.to_le_bytes())?;
    tx_port.write_all(bytes_of(packet))?;

    Ok(())
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut rx_serial = SerialPort::open(peripherals.port_1, BAUD_RATE).await;
    let mut tx_serial = SerialPort::open(peripherals.port_2, BAUD_RATE).await;
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
        if let Some(power_packet) = get_power_packet(&mut rx_serial) {
            println!("Got power packet: {:?}", power_packet);
            front_lefts.iter_mut().for_each(|m| {
                m.set_voltage(power_packet.front_left * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
            front_rights.iter_mut().for_each(|m| {
                m.set_voltage(power_packet.front_right * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
            back_lefts.iter_mut().for_each(|m| {
                m.set_voltage(power_packet.back_left * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });
            back_rights.iter_mut().for_each(|m| {
                m.set_voltage(power_packet.back_right * MOTOR_VOLTAGE_MAX / MOTOR_POWER_MAX)
                    .expect("Motor set broke");
            });

            let encoder_packet = MotorPacket {
                front_left: front_lefts[0]
                    .position()
                    .expect("Motor position broke")
                    .as_degrees(),
                front_right: front_rights[0]
                    .position()
                    .expect("Motor position broke")
                    .as_degrees(),
                back_left: back_lefts[0]
                    .position()
                    .expect("Motor position broke")
                    .as_degrees(),
                back_right: back_rights[0]
                    .position()
                    .expect("Motor position broke")
                    .as_degrees(),
            };
            if send_encoder_packet(&mut tx_serial, &encoder_packet).is_ok() {
                println!("Sent encoder packet: {:?}", encoder_packet);
            }
        }
    }
}
