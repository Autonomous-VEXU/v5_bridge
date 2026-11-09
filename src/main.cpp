#include "main.h"

#include <cmath>
#include <cstring>
#include <limits>
#include <vector>

#include "bridge/protocol.hpp"
#include "bridge/transport.hpp"

using namespace bridge;

// Set to 1 to talk over smart ports 20 (rx) / 19 (tx) via the comms board
// instead of the USB cable.
#define BRIDGE_USE_SMART_PORT_SERIAL 0

namespace {

[[maybe_unused]] constexpr int32_t BAUD_RATE = 115200; // Unused on USB
[[maybe_unused]] constexpr uint8_t RX_SERIAL_PORT = 20; // tx on comms board
[[maybe_unused]] constexpr uint8_t TX_SERIAL_PORT = 19; // rx on comms board

// Wheel turns per motor turn.
// NOTE: commands divide by this but feedback also divides by it, so reported
// wheel position/velocity is 4x what was commanded. Kept as-is from the Rust
// version; one direction should multiply instead.
constexpr double WHEEL_GEAR_RATIO = 1.0 / 2.0;

constexpr double RAD_PER_REV = 2.0 * M_PI;
constexpr double SEC_PER_MIN = 60.0;
constexpr double RAD_PER_DEG = M_PI / 180.0;

constexpr uint32_t PACKET_TIMEOUT_MS = 100;
constexpr uint32_t DISPLAY_PERIOD_MS = 100;

#if BRIDGE_USE_SMART_PORT_SERIAL
using Transport = SmartPortTransport;
#else
using Transport = UsbTransport;
#endif

enum class InputPacketType { None, Motor, ResetEncoders };

// Saturating float -> int like Rust's `as i32` (NaN becomes 0).
int32_t to_rpm_int(double rpm) {
    if (std::isnan(rpm)) return 0;
    if (rpm >= std::numeric_limits<int32_t>::max()) return std::numeric_limits<int32_t>::max();
    if (rpm <= std::numeric_limits<int32_t>::min()) return std::numeric_limits<int32_t>::min();
    return static_cast<int32_t>(rpm);
}

int32_t packet_to_wheel_motor_rpm(float packet_value) {
    double rev_per_min = static_cast<double>(packet_value) / RAD_PER_REV * SEC_PER_MIN;
    return to_rpm_int(rev_per_min / WHEEL_GEAR_RATIO);
}

/*
int32_t packet_to_intake_motor_rpm(float packet_value) {
    return to_rpm_int(static_cast<double>(packet_value) / RAD_PER_REV * SEC_PER_MIN);
}
*/

// PROS reports read errors as PROS_ERR_F (infinity); report those as 0.
float or_zero(double value) {
    return std::isfinite(value) ? static_cast<float>(value) : 0.0f;
}

float wheel_position_rad(double motor_degrees) {
    return or_zero(motor_degrees * RAD_PER_DEG / WHEEL_GEAR_RATIO);
}

/*
float intake_position_rad(double motor_degrees) {
    return or_zero(motor_degrees * RAD_PER_DEG);
}
*/

float rpm_to_wheel_rad_per_sec(double rpm) {
    return or_zero((rpm * RAD_PER_REV / SEC_PER_MIN) / WHEEL_GEAR_RATIO);
}

/*
float rpm_to_intake_rad_per_sec(double rpm) {
    return or_zero(rpm * RAD_PER_REV / SEC_PER_MIN);
}
*/

uint64_t magic_at(const std::vector<uint8_t>& buf, size_t idx) {
    uint64_t magic;
    std::memcpy(&magic, buf.data() + idx, sizeof(magic));
    return magic;
}

// Waits up to PACKET_TIMEOUT_MS for a packet. Scans from the newest bytes
// backwards so a backlog of stale commands is skipped in favor of the latest
// one; everything up to the end of the packet found is discarded.
InputPacketType get_packet(Transport& rx, std::vector<uint8_t>& persistent_buf, MotorPacket& out) {
    const uint32_t start_time = pros::millis();
    while (pros::millis() - start_time < PACKET_TIMEOUT_MS) {
        uint8_t chunk[128];
        size_t n;
        while ((n = rx.read_available(chunk, sizeof(chunk))) > 0) {
            persistent_buf.insert(persistent_buf.end(), chunk, chunk + n);
        }

        if (persistent_buf.size() >= sizeof(uint64_t)) {
            for (size_t idx = persistent_buf.size() - sizeof(uint64_t) + 1; idx-- > 0;) {
                const uint64_t magic = magic_at(persistent_buf, idx);
                if (magic == MOTOR_PACKET_MAGIC && idx + sizeof(MotorPacket) <= persistent_buf.size()) {
                    std::memcpy(&out, persistent_buf.data() + idx, sizeof(MotorPacket));
                    persistent_buf.erase(persistent_buf.begin(), persistent_buf.begin() + idx + sizeof(MotorPacket));
                    return InputPacketType::Motor;
                }
                if (magic == RESET_ENCODER_MAGIC) {
                    persistent_buf.erase(persistent_buf.begin(), persistent_buf.begin() + idx + sizeof(uint64_t));
                    return InputPacketType::ResetEncoders;
                }
            }
        }
        pros::delay(1);
    }

    persistent_buf.clear();
    return InputPacketType::None;
}

void send_packet(Transport& tx, const MotorPacket& packet) {
    tx.write_all(reinterpret_cast<const uint8_t*>(&packet), sizeof(packet));
}

void draw_packet(int16_t first_line, const char* title, const MotorPacket& p) {
    pros::screen::print(pros::E_TEXT_SMALL, first_line, "%s", title);
    pros::screen::print(pros::E_TEXT_SMALL, first_line + 1, "FL %8.3f  FR %8.3f  BL %8.3f  BR %8.3f",
                        p.front_left, p.front_right, p.back_left, p.back_right);
    pros::screen::print(pros::E_TEXT_SMALL, first_line + 2, "I1 %8.3f  I2 %8.3f  I3 %8.3f",
                        p.intake1, p.intake2, p.intake3);
}

void bridge_loop() {
    // A negative port reverses that motor.
    pros::MotorGroup front_rights({4, -3}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::MotorGroup front_lefts({2, -1}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::MotorGroup back_rights({14, -13}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    pros::MotorGroup back_lefts({12, -11}, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    //pros::Motor intake1(15, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    //pros::Motor intake2(16, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);
    //pros::Motor intake3(17, pros::v5::MotorGears::green, pros::v5::MotorUnits::degrees);

#if BRIDGE_USE_SMART_PORT_SERIAL
    Transport link(RX_SERIAL_PORT, TX_SERIAL_PORT, BAUD_RATE);
#else
    Transport link;
#endif

    std::vector<uint8_t> persistent_input_buf;
    uint32_t last_draw = 0;
    while (true) {
        MotorPacket cmd;
        switch (get_packet(link, persistent_input_buf, cmd)) {
            case InputPacketType::Motor:
                front_lefts.move_velocity(packet_to_wheel_motor_rpm(cmd.front_left));
                front_rights.move_velocity(packet_to_wheel_motor_rpm(cmd.front_right));
                back_lefts.move_velocity(packet_to_wheel_motor_rpm(cmd.back_left));
                back_rights.move_velocity(packet_to_wheel_motor_rpm(cmd.back_right));
                //intake1.move_velocity(packet_to_intake_motor_rpm(cmd.intake1));
                //intake2.move_velocity(packet_to_intake_motor_rpm(cmd.intake2));
                //intake3.move_velocity(packet_to_intake_motor_rpm(cmd.intake3));
                break;
            case InputPacketType::ResetEncoders:
                front_lefts.tare_position_all();
                back_lefts.tare_position_all();
                front_rights.tare_position_all();
                back_rights.tare_position_all();
                //intake1.tare_position();
                //intake2.tare_position();
                //intake3.tare_position();
                break;
            case InputPacketType::None:
                break;
        }

        // Feedback comes from the first motor of each group only.
        const MotorPacket position_packet{
            ENCODER_POSITION_MAGIC,
            wheel_position_rad(front_lefts.get_position(0)),
            wheel_position_rad(front_rights.get_position(0)),
            wheel_position_rad(back_lefts.get_position(0)),
            wheel_position_rad(back_rights.get_position(0)),
            //intake_position_rad(intake1.get_position()),
            //intake_position_rad(intake2.get_position()),
            //intake_position_rad(intake3.get_position()),
        };
        send_packet(link, position_packet);

        const MotorPacket velocity_packet{
            ENCODER_VELOCITY_MAGIC,
            rpm_to_wheel_rad_per_sec(front_lefts.get_actual_velocity(0)),
            rpm_to_wheel_rad_per_sec(front_rights.get_actual_velocity(0)),
            rpm_to_wheel_rad_per_sec(back_lefts.get_actual_velocity(0)),
            rpm_to_wheel_rad_per_sec(back_rights.get_actual_velocity(0)),
            //rpm_to_intake_rad_per_sec(intake1.get_actual_velocity()),
            //rpm_to_intake_rad_per_sec(intake2.get_actual_velocity()),
            //rpm_to_intake_rad_per_sec(intake3.get_actual_velocity()),
        };
        send_packet(link, velocity_packet);

        // Redrawing every loop would slow the telemetry rate, so throttle it.
        if (pros::millis() - last_draw >= DISPLAY_PERIOD_MS) {
            last_draw = pros::millis();
            pros::screen::erase();
            draw_packet(0, "--POSITIONS:", position_packet);
            draw_packet(4, "--VELOCITIES:", velocity_packet);
        }
    }
}

} // namespace

// Started from initialize() rather than opcontrol() so, like the Rust version,
// the bridge runs regardless of competition mode and survives mode switches.
void initialize() {
    static pros::Task bridge_task(bridge_loop, "bridge");
}

void disabled() {}

void competition_initialize() {}

void autonomous() {}

void opcontrol() {
    while (true) {
        pros::delay(1000);
    }
}
