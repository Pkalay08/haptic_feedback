// controller_follower_protocol.hpp
#ifndef CONTROLLER_FOLLOWER_PROTOCOL_HPP
#define CONTROLLER_FOLLOWER_PROTOCOL_HPP

#include <cstdint>
#include <cstddef>
#include <string>

namespace arm_control {
namespace protocol {

// ============================================================================
// Constants
// ============================================================================

constexpr uint8_t  START_BYTE          = 0xAA;
constexpr uint16_t CRC_INITIAL         = 0xFFFF;
constexpr uint16_t CRC_POLYNOMIAL      = 0x1021;   // CRC‑16‑CCITT
constexpr size_t   MAX_PAYLOAD_SIZE    = 64;
constexpr size_t   MAX_SERVOS          = 12;

// ============================================================================
// Message Types
// ============================================================================

enum class MessageType : uint8_t {
    COMMAND_SERVO   = 0x01,   // Controller → Follower
    FEEDBACK_HAPTIC = 0x02,   // Follower → Controller
    FEEDBACK_STATE  = 0x03    // Follower → Controller (optional)
};

// ============================================================================
// Packed Payload Structures (no padding)
// ============================================================================

#pragma pack(push, 1)

struct ServoCommandPayload {
    uint16_t num_servos;
    float    positions[MAX_SERVOS];
    uint16_t duration_ms;
    uint8_t  flags;
};

struct HapticFeedbackPayload {
    float    force_magnitude;
    float    frequency_hz;
    uint16_t duration_ms;
    uint8_t  flags;
};

struct FollowerStatePayload {
    float    battery_voltage;
    float    temperature_celsius;
    uint16_t error_code;
    float    current_positions[MAX_SERVOS];
    float    velocities[MAX_SERVOS];
};

struct ProtocolFrame {
    uint8_t  start_byte;
    uint8_t  type;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint16_t payload_length;
    uint8_t  payload[MAX_PAYLOAD_SIZE];
    uint16_t crc;
};

#pragma pack(pop)

// ============================================================================
// CRC Helpers (inline, header‑only)
// ============================================================================

uint16_t crc16(const uint8_t* data, size_t len);
uint16_t computeFrameCRC(const ProtocolFrame& frame);

// ============================================================================
// Serial Transport (Linux syscalls)
// ============================================================================

class SerialTransport {
public:
    SerialTransport();
    ~SerialTransport();

    // Non‑copyable, non‑movable
    SerialTransport(const SerialTransport&) = delete;
    SerialTransport& operator=(const SerialTransport&) = delete;

    bool openPort(const std::string& port, int baudrate, int timeout_ms = 200);
    void close();
    bool isOpen() const;
    void setTimeout(int ms);

    // Controller → Follower
    bool sendServoCommand(uint16_t sequence, uint32_t timestamp_ms,
                          const ServoCommandPayload& cmd);

    // Follower → Controller
    bool receiveHapticFeedback(HapticFeedbackPayload& fb,
                               uint16_t* out_seq = nullptr,
                               uint32_t* out_timestamp = nullptr);

    bool receiveFollowerState(FollowerStatePayload& state,
                              uint16_t* out_seq = nullptr,
                              uint32_t* out_timestamp = nullptr);

private:
    static constexpr size_t RX_BUFFER_SIZE = 4096;

    int     fd_;
    int     timeout_ms_;
    uint8_t rx_buffer_[RX_BUFFER_SIZE];
    size_t  rx_head_;
    size_t  rx_tail_;

    // Private helpers
    size_t availableReadSpace() const;
    size_t availableWriteSpace() const;
    uint8_t getByte(size_t idx) const;
    void consumeBytes(size_t count);
    bool receiveFrame(ProtocolFrame& frame);
    size_t findStartByte() const;
};

} // namespace protocol
} // namespace arm_control

#endif // CONTROLLER_FOLLOWER_PROTOCOL_HPP