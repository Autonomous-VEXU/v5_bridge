use core::{time::Duration, fmt::Write as _};
use std::{
    f64::consts::PI,
    io::{self, Read, Write},
    time::Instant,
};

use bytemuck::{Pod, Zeroable, from_bytes, bytes_of};
use vexide::{color::Color, prelude::*};

#[allow(unused)] // Unused in USB port
const BAUD_RATE: u32 = 115200;

const MOTOR_PACKET_MAGIC: u64     = 0xFEFAABCD1234BEEF;
const ENCODER_POSITION_MAGIC: u64 = 0xF23BDEAD6789ACBD;
const ENCODER_VELOCITY_MAGIC: u64 = 0xF81CB00B1350C0CA;

const WHEEL_GEAR_RATIO: f64 = 22f64 / 26f64;

#[derive(Clone, Copy, Pod, Debug)]
#[repr(C, packed(1))]
struct MotorPacket {
    magic: u64,
    front_left: f32,
    back_left: f32,
    back_right: f32,
    front_right: f32,
    intake1: f32,
    intake2: f32,
    intake3: f32,
}

unsafe impl Zeroable for MotorPacket {
    fn zeroed() -> Self {
        MotorPacket {
            magic: 0,
            front_left: 0.,
            back_left: 0.,
            back_right: 0.,
            front_right: 0.,
            intake1: 0.,
            intake2: 0.,
            intake3: 0.,
        }
    }
}

async fn get_motor_packet(
    rx_port: &mut impl Read,
    persistent_buf: &mut Vec<u8>,
) -> io::Result<MotorPacket> {
    const TIMEOUT: Duration = Duration::from_millis(100);

    let start_time = Instant::now();
    while Instant::now().duration_since(start_time) < TIMEOUT {
        let mut sub_buf = vec![];
        rx_port.read_to_end(&mut sub_buf)?;
        persistent_buf.extend(sub_buf);

        if persistent_buf.len() > size_of::<MotorPacket>() {
            let mut idx = persistent_buf.len() - size_of::<MotorPacket>();
            while idx >= 0 {
                if persistent_buf[idx..(idx + size_of_val(&MOTOR_PACKET_MAGIC))]
                    == MOTOR_PACKET_MAGIC.to_le_bytes()
                {
                    let end_of_packet = idx + size_of::<MotorPacket>();
                    let packet = from_bytes::<MotorPacket>(&persistent_buf[idx..end_of_packet]).clone();
                    persistent_buf.drain(..end_of_packet);

                    return Ok(packet);
                }
                idx -= 1;
            }
        }
        sleep(Duration::from_millis(1)).await;
    }

    persistent_buf.clear();

    Err(io::ErrorKind::TimedOut.into())
}

fn send_position_packet(
    tx_port: &mut impl Write,
    packet: &MotorPacket,
) -> Result<(), std::io::Error> {
    tx_port.write_all(bytes_of(packet))?;

    Ok(())
}

fn send_velocity_packet(
    tx_port: &mut impl Write,
    packet: &MotorPacket,
) -> Result<(), std::io::Error> {
    tx_port.write_all(bytes_of(packet))?;

    Ok(())
}

const RAD_PER_REV: f64 = 2.0 * PI;
const SEC_PER_MIN: f64 = 60.0;

fn packet_to_wheel_motor_rpm(packet_value: f32) -> i32 {
    let rev_per_sec = (packet_value as f64) / RAD_PER_REV;
    let rev_per_min = rev_per_sec * SEC_PER_MIN;

    let output_rpm = rev_per_min / WHEEL_GEAR_RATIO;
    output_rpm as _
}

fn packet_to_intake_motor_rpm(packet_value: f32) -> i32 {
    let rev_per_sec = (packet_value as f64) / RAD_PER_REV;
    let rev_per_min = rev_per_sec * SEC_PER_MIN;

    rev_per_min as _
}

fn rpm_to_intake_rad_per_sec(rpm: f64) -> f32 {
    (rpm * RAD_PER_REV / SEC_PER_MIN) as f32
}

fn rpm_to_wheel_rad_per_sec(rpm: f64) -> f32 {
    ((rpm * RAD_PER_REV / SEC_PER_MIN) / WHEEL_GEAR_RATIO) as f32
}

#[vexide::main]
async fn main(mut peripherals: Peripherals) {
    let mut rx_serial = SerialPort::open(peripherals.port_15, BAUD_RATE).await;
    let mut tx_serial = SerialPort::open(peripherals.port_16, BAUD_RATE).await;
    let mut front_rights: [Motor; _] = [
        Motor::new(peripherals.port_2, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_1, Gearset::Green, Direction::Reverse),
    ];
    let mut front_lefts: [Motor; 2] = [
        Motor::new(peripherals.port_4, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_3, Gearset::Green, Direction::Reverse),
    ];
    let mut back_rights: [Motor; 2] = [
        Motor::new(peripherals.port_14, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_13, Gearset::Green, Direction::Reverse),
    ];
    let mut back_lefts: [Motor; 2] = [
        Motor::new(peripherals.port_12, Gearset::Green, Direction::Forward),
        Motor::new(peripherals.port_11, Gearset::Green, Direction::Reverse),
    ];
    let mut intake1 = Motor::new(peripherals.port_18, Gearset::Green, Direction::Forward);
    let mut intake2 = Motor::new(peripherals.port_19, Gearset::Green, Direction::Forward);
    let mut intake3 = Motor::new(peripherals.port_20, Gearset::Green, Direction::Forward);

    let input = &mut rx_serial;
    let output = &mut tx_serial;

    // let input = &mut std::io::stdin();
    // let output = &mut std::io::stdout();


    let mut i = 0;
    let mut persistent_motor_buf = vec![];
    loop {
        i = (i + 1) % 3;
        let motor_packet = get_motor_packet(input, &mut persistent_motor_buf).await;
        if let Ok(motor_packet) = motor_packet {
            println!("Got power packet: {:?}", motor_packet);
            front_lefts.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_wheel_motor_rpm(motor_packet.front_left));
            });
            front_rights.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_wheel_motor_rpm(motor_packet.front_right));
            });
            back_lefts.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_wheel_motor_rpm(motor_packet.back_left));
            });
            back_rights.iter_mut().for_each(|m| {
                let _ = m.set_velocity(packet_to_wheel_motor_rpm(motor_packet.back_right));
            });
            let _ = intake1.set_velocity(packet_to_intake_motor_rpm(motor_packet.intake1));
            let _ = intake2.set_velocity(packet_to_intake_motor_rpm(motor_packet.intake2));
            let _ = intake3.set_velocity(packet_to_intake_motor_rpm(motor_packet.intake3));
        }

        
        let position_packet = MotorPacket {
            magic: ENCODER_POSITION_MAGIC,
            front_left: front_lefts[0]
                .position()
                .map_or(0f32, |x| (x.as_radians() / WHEEL_GEAR_RATIO) as f32),
            front_right: front_rights[0]
                .position()
                .map_or(0f32, |x| (x.as_radians() / WHEEL_GEAR_RATIO) as f32),
            back_left: back_lefts[0]
                .position()
                .map_or(0f32, |x| (x.as_radians() / WHEEL_GEAR_RATIO) as f32),
            back_right: back_rights[0]
                .position()
                .map_or(0f32, |x| (x.as_radians() / WHEEL_GEAR_RATIO) as f32),
            intake1: intake1
                .position()
                .map_or(0f32, |x| x.as_radians() as f32),
            intake2: intake2
                .position()
                .map_or(0f32, |x| x.as_radians() as f32),
            intake3: intake3
                .position()
                .map_or(0f32, |x| x.as_radians() as f32),
        };
        
        if i == 1 && send_position_packet(output, &position_packet).is_ok() {
            // println!("Sent position packet: {:?}", position_packet);
        }
        peripherals.display.erase(Color::from_raw(0));
        let _ = writeln!(peripherals.display, "--POSITIONS:\n{position_packet:?}");
        
        let velocity_packet = MotorPacket {
            magic: ENCODER_VELOCITY_MAGIC,
            front_left: front_lefts[0]
            .velocity()
            .map_or(-f32::INFINITY, rpm_to_wheel_rad_per_sec),
            front_right: front_rights[0]
                .velocity()
                .map_or(-f32::INFINITY, rpm_to_wheel_rad_per_sec),
            back_left: back_lefts[0]
            .velocity()
            .map_or(-f32::INFINITY, rpm_to_wheel_rad_per_sec),
            back_right: back_rights[0]
            .velocity()
            .map_or(-f32::INFINITY, rpm_to_wheel_rad_per_sec),
            intake1: intake1
            .velocity()
            .map_or(-f32::INFINITY, rpm_to_intake_rad_per_sec),
            intake2: intake2
            .velocity()
            .map_or(-f32::INFINITY, rpm_to_intake_rad_per_sec),
            intake3: intake3
            .velocity()
            .map_or(-f32::INFINITY, rpm_to_intake_rad_per_sec),
        };
        
        if i == 2 && send_velocity_packet(output, &velocity_packet).is_ok() {
            // println!("Sent velocity packet: {:?}", velocity_packet);
        }
        let _ = write!(peripherals.display, "--VELOCITIES:\n{velocity_packet:?}");

    }
}
