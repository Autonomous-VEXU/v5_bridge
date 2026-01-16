use core::time::Duration;
use std::{
    f64::consts::PI,
    io::{Read, Write},
    time::Instant,
};

use bytemuck::{Pod, Zeroable, bytes_of, bytes_of_mut};
use vexide::prelude::*;

#[allow(unused)] // Unused in USB port
const BAUD_RATE: u32 = 115200;
const MOTOR_PACKET_MAGIC: u16 = 0xFEFA;
const ENCODER_PACKET_MAGIC: u16 = 0xF23B;
const WHEEL_GEAR_RATIO: f64 = 26f64 / 22f64;

#[derive(Clone, Copy, Pod, Debug)]
#[repr(C)]
struct MotorPacket {
    front_left: f64,
    back_left: f64,
    back_right: f64,
    front_right: f64,
}

unsafe impl Zeroable for MotorPacket {
    fn zeroed() -> Self {
        MotorPacket {
            front_left: 0.,
            back_left: 0.,
            back_right: 0.,
            front_right: 0.,
        }
    }
}

fn get_motor_packet(rx_port: &mut impl Read) -> Option<MotorPacket> {
    const TIMEOUT: Duration = Duration::from_secs(1);
    println!("getting packet...");

    let start_time = Instant::now();
    while Instant::now().duration_since(start_time) < TIMEOUT {
        // Check for the whole magic, byte by byte
        if MOTOR_PACKET_MAGIC
            .to_le_bytes()
            .iter()
            .all(|x| rx_port.bytes().next().transpose().unwrap() == Some(*x))
        {
            let mut packet = MotorPacket::zeroed();
            println!("read ROS packet: {:?}", packet);
            if rx_port.read_exact(bytes_of_mut(&mut packet)).is_ok() {
                return Some(packet);
            }
        }
    }

    None
}

fn send_encoder_packet(
    tx_port: &mut impl Write,
    packet: &MotorPacket,
) -> Result<(), std::io::Error> {
    tx_port.write_all(&ENCODER_PACKET_MAGIC.to_le_bytes())?;
    tx_port.write_all(bytes_of(packet))?;

    Ok(())
}

fn packet_to_motor_rpm(packet_value: f64) -> i32 {
    const RAD_PER_REV: f64 = 2.0 * PI;
    const SEC_PER_MIN: f64 = 60.0;

    let rev_per_sec = packet_value / RAD_PER_REV;
    let rev_per_min = rev_per_sec * SEC_PER_MIN;

    let output_rpm = rev_per_min * WHEEL_GEAR_RATIO;
    output_rpm as _
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    // let mut rx_serial = SerialPort::open(peripherals.port_19, BAUD_RATE).await;
    // let mut tx_serial = SerialPort::open(peripherals.port_20, BAUD_RATE).await;
    let mut front_lefts: [Motor; _] = [
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Reverse),
    ];
    let mut front_rights: [Motor; 2] = [
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Reverse),
    ];
    let mut back_lefts: [Motor; 2] = [
        Motor::new(peripherals.port_11, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_12, Gearset::Green, Direction::Reverse),
    ];
    let mut back_rights: [Motor; 2] = [
        Motor::new(peripherals.port_13, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_14, Gearset::Green, Direction::Reverse),
    ];

    loop {
        if let Some(motor_packet) = get_motor_packet(&mut std::io::stdin()) {
            println!("Got power packet: {:?}", motor_packet);
            front_lefts.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_motor_rpm(motor_packet.front_left));
            });
            front_rights.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_motor_rpm(motor_packet.front_right));
            });
            back_lefts.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_motor_rpm(motor_packet.back_left));
            });
            back_rights.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_motor_rpm(motor_packet.back_right));
            });
        }

        let encoder_packet = MotorPacket {
            front_left: front_lefts[0]
                .position()
                .map_or(-f64::INFINITY, |x| x.as_radians() / WHEEL_GEAR_RATIO),
            front_right: front_rights[0]
                .position()
                .map_or(-f64::INFINITY, |x| x.as_radians() / WHEEL_GEAR_RATIO),
            back_left: back_lefts[0]
                .position()
                .map_or(-f64::INFINITY, |x| x.as_radians() / WHEEL_GEAR_RATIO),
            back_right: back_rights[0]
                .position()
                .map_or(-f64::INFINITY, |x| x.as_radians() / WHEEL_GEAR_RATIO),
        };

        if send_encoder_packet(&mut std::io::stdout(), &encoder_packet).is_ok() {
            println!("Sent encoder packet: {:?}", encoder_packet);
        }
    }
}
