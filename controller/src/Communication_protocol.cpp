// controller_follower_protocol.cpp
#include "Communication_protocol.hpp"

#include <cstring>
#include <unistd.h>
#include <fcntl.h>
#include <termios.h>
#include <poll.h>
#include <errno.h>

namespace arm_control {
namespace protocol {

// ============================================================================
// CRC functions
// ============================================================================

uint16_t crc16(const uint8_t* data, size_t len) {
    uint16_t crc = CRC_INITIAL;
    for (size_t i = 0; i < len; ++i) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int bit = 0; bit < 8; ++bit) {
            if (crc & 0x8000)
                crc = (crc << 1) ^ CRC_POLYNOMIAL;
            else
                crc <<= 1;
        }
    }
    return crc;
}

uint16_t computeFrameCRC(const ProtocolFrame& frame) {
    const uint8_t* start = reinterpret_cast<const uint8_t*>(&frame.type);
    size_t len = sizeof(ProtocolFrame) - offsetof(ProtocolFrame, type) - sizeof(uint16_t);
    return crc16(start, len);
}

// ============================================================================
// SerialTransport Implementation
// ============================================================================

SerialTransport::SerialTransport()
    : fd_(-1)
    , timeout_ms_(200)
    , rx_head_(0)
    , rx_tail_(0) {
    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
}

SerialTransport::~SerialTransport() {
    close();
}

bool SerialTransport::openPort(const std::string& port, int baudrate, int timeout_ms) {
    if (fd_ >= 0) close();

    fd_ = ::open(port.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
    if (fd_ < 0) return false;

    struct termios tty;
    if (tcgetattr(fd_, &tty) != 0) goto fail;

    speed_t speed;
    switch (baudrate) {
        case 9600:   speed = B9600;   break;
        case 19200:  speed = B19200;  break;
        case 38400:  speed = B38400;  break;
        case 57600:  speed = B57600;  break;
        case 115200: speed = B115200; break;
        case 230400: speed = B230400; break;
        case 460800: speed = B460800; break;
        case 921600: speed = B921600; break;
        default:     speed = B115200; break;
    }
    cfsetospeed(&tty, speed);
    cfsetispeed(&tty, speed);

    cfmakeraw(&tty);
    tty.c_cflag &= ~(PARENB | CSTOPB | CSIZE | CRTSCTS);
    tty.c_cflag |= CS8 | CREAD | CLOCAL;
    tty.c_cc[VMIN]  = 0;
    tty.c_cc[VTIME] = 0;

    if (tcsetattr(fd_, TCSANOW, &tty) != 0) goto fail;

    timeout_ms_ = timeout_ms;
    rx_head_ = rx_tail_ = 0;
    std::memset(rx_buffer_, 0, sizeof(rx_buffer_));
    return true;

fail:
    ::close(fd_);
    fd_ = -1;
    return false;
}

void SerialTransport::close() {
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
}

bool SerialTransport::isOpen() const {
    return fd_ >= 0;
}

void SerialTransport::setTimeout(int ms) {
    timeout_ms_ = ms;
}

bool SerialTransport::sendServoCommand(uint16_t sequence, uint32_t timestamp_ms,
                                       const ServoCommandPayload& cmd) {
    if (fd_ < 0 || cmd.num_servos == 0 || cmd.num_servos > MAX_SERVOS)
        return false;

    ProtocolFrame frame;
    frame.start_byte     = START_BYTE;
    frame.type           = static_cast<uint8_t>(MessageType::COMMAND_SERVO);
    frame.sequence       = sequence;
    frame.timestamp_ms   = timestamp_ms;
    frame.payload_length = sizeof(ServoCommandPayload);
    std::memcpy(frame.payload, &cmd, sizeof(ServoCommandPayload));
    frame.crc = computeFrameCRC(frame);

    uint8_t tx_buf[sizeof(ProtocolFrame)];
    std::memcpy(tx_buf, &frame, sizeof(frame));
    size_t tx_len = sizeof(frame);

    ssize_t written = ::write(fd_, tx_buf, tx_len);
    return (written >= 0 && static_cast<size_t>(written) == tx_len);
}

bool SerialTransport::receiveHapticFeedback(HapticFeedbackPayload& fb,
                                            uint16_t* out_seq,
                                            uint32_t* out_timestamp) {
    ProtocolFrame frame;
    if (!receiveFrame(frame)) return false;

    if (frame.type != static_cast<uint8_t>(MessageType::FEEDBACK_HAPTIC))
        return false;
    if (frame.payload_length != sizeof(HapticFeedbackPayload))
        return false;

    std::memcpy(&fb, frame.payload, sizeof(HapticFeedbackPayload));
    if (out_seq)       *out_seq       = frame.sequence;
    if (out_timestamp) *out_timestamp = frame.timestamp_ms;
    return true;
}

bool SerialTransport::receiveFollowerState(FollowerStatePayload& state,
                                           uint16_t* out_seq,
                                           uint32_t* out_timestamp) {
    ProtocolFrame frame;
    if (!receiveFrame(frame)) return false;

    if (frame.type != static_cast<uint8_t>(MessageType::FEEDBACK_STATE))
        return false;
    if (frame.payload_length != sizeof(FollowerStatePayload))
        return false;

    std::memcpy(&state, frame.payload, sizeof(frame));
    if (out_seq)       *out_seq       = frame.sequence;
    if (out_timestamp) *out_timestamp = frame.timestamp_ms;
    return true;
}

// ----------------------------------------------------------------------------
// Private helpers
// ----------------------------------------------------------------------------

size_t SerialTransport::availableReadSpace() const {
    if (rx_head_ >= rx_tail_)
        return rx_head_ - rx_tail_;
    else
        return RX_BUFFER_SIZE - rx_tail_ + rx_head_;
}

size_t SerialTransport::availableWriteSpace() const {
    return RX_BUFFER_SIZE - availableReadSpace() - 1;
}

uint8_t SerialTransport::getByte(size_t idx) const {
    return rx_buffer_[idx % RX_BUFFER_SIZE];
}

void SerialTransport::consumeBytes(size_t count) {
    if (count >= availableReadSpace()) {
        rx_tail_ = rx_head_;
    } else {
        rx_tail_ = (rx_tail_ + count) % RX_BUFFER_SIZE;
    }
}

bool SerialTransport::receiveFrame(ProtocolFrame& frame) {
    if (fd_ < 0) return false;

    // Wait for data with timeout
    struct pollfd pfd = {fd_, POLLIN, 0};
    int ret = ::poll(&pfd, 1, timeout_ms_);
    if (ret <= 0) return false;

    // Read into ring buffer
    ssize_t n = ::read(fd_, rx_buffer_ + rx_head_, availableWriteSpace());
    if (n <= 0) return false;
    rx_head_ = (rx_head_ + static_cast<size_t>(n)) % RX_BUFFER_SIZE;

    // Scan for a valid frame
    while (availableReadSpace() >= sizeof(ProtocolFrame)) {
        size_t start_off = findStartByte();
        if (start_off == RX_BUFFER_SIZE) {
            rx_tail_ = rx_head_;
            return false;
        }
        consumeBytes(start_off);

        // Copy candidate into temporary struct
        uint8_t raw[sizeof(ProtocolFrame)];
        for (size_t i = 0; i < sizeof(ProtocolFrame); ++i)
            raw[i] = getByte(rx_tail_ + i);

        ProtocolFrame cand;
        std::memcpy(&cand, raw, sizeof(ProtocolFrame));

        if (cand.start_byte == START_BYTE &&
            cand.payload_length <= MAX_PAYLOAD_SIZE &&
            computeFrameCRC(cand) == cand.crc) {
            consumeBytes(sizeof(ProtocolFrame));
            frame = cand;
            return true;
        } else {
            consumeBytes(1); // false start – skip one byte
        }
    }
    return false;
}

size_t SerialTransport::findStartByte() const {
    size_t avail = availableReadSpace();
    for (size_t off = 0; off < avail; ++off) {
        if (getByte(rx_tail_ + off) == START_BYTE)
            return off;
    }
    return RX_BUFFER_SIZE;
}

} // namespace protocol
} // namespace arm_control