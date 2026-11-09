#pragma once

#include <cstdint>

// Wire protocol shared with otto_hardware (omni_wheel_hardware.cpp).
// Every packet is a little-endian, unpadded MotorPacket whose first 8 bytes
// are a magic number identifying the packet type. RESET_ENCODER is the one
// exception: it is sent as the bare 8-byte magic with no payload.
namespace bridge {

constexpr uint64_t MOTOR_PACKET_MAGIC     = 0xFEFAABCD1234BEEF; // host -> brain: velocity command (rad/s)
constexpr uint64_t RESET_ENCODER_MAGIC    = 0xABCD5869B0B1CCCC; // host -> brain: zero all encoders
constexpr uint64_t ENCODER_POSITION_MAGIC = 0xF23BDEAD6789ACBD; // brain -> host: positions (rad)
// NOTE: otto_hardware currently expects 0xF81CB00B1350C0CA for velocity
// packets, so the host never decodes these. One side needs to change.
constexpr uint64_t ENCODER_VELOCITY_MAGIC = 0xF81CB3924EA7C0CA; // brain -> host: velocities (rad/s)

#pragma pack(push, 1)
struct MotorPacket {
    uint64_t magic;
    float front_left;
    float front_right;
    float back_left;
    float back_right;
    float intake1;
    float intake2;
    float intake3;
};
#pragma pack(pop)

static_assert(sizeof(MotorPacket) == 36, "MotorPacket must match the host's 36-byte layout");

} // namespace bridge
