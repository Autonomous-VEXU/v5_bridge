use std::io::Write;
use bytemuck::{Pod, Zeroable, bytes_of};
use vexide::{prelude::*};
const BAUD_RATE: u32 = 115200;

#[derive(Clone, Copy, Pod, Debug)]
#[repr(C, packed(1))]
struct CountPacket {
    count: u32,
}

unsafe impl Zeroable for CountPacket {
    fn zeroed() -> Self {
        CountPacket {
            count: 0,
        }
    }
}

#[vexide::main]
async fn main(peripherals: Peripherals) {
    let mut tx_serial = SerialPort::open(peripherals.port_19, BAUD_RATE).await;
    let output = &mut tx_serial;

    let mut count = 1u32;
    loop {
        let count_packet = CountPacket {
            count,
        };
        
        let _ = output.write_all(bytes_of(&count_packet));
        
        count += 1;
        if count > 225 {
            count = 1;
        }
    }
}
