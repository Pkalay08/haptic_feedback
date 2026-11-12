#include "UART.hpp"

#include <string>
#include <chrono>
#include <cstdint>
#include "Logger.hpp"
#include <thread>

#if defined(__linux__)
#include <fcntl.h>
#include <termios.h>
#include <unistd.h>
#endif

namespace haptic_feedback {
namespace {

#if defined(__linux__)
speed_t toLinuxBaudRate(UART::BaudRate baud_rate)
{
    switch (baud_rate) {
    case UART::BaudRate::Rate9600:
        return B9600;
    case UART::BaudRate::Rate19200:
        return B19200;
    case UART::BaudRate::Rate38400:
        return B38400;
    case UART::BaudRate::Rate57600:
        return B57600;
    case UART::BaudRate::Rate115200:
        return B115200;
    case UART::BaudRate::Rate230400:
        return B230400;
    case UART::BaudRate::Rate460800:
        return B460800;
    case UART::BaudRate::Rate921600:
        return B921600;
    default:
        return B115200;
    }
}

tcflag_t toLinuxDataBits(UART::DataBits data_bits)
{
    switch (data_bits) {
    case UART::DataBits::Five:
        return CS5;
    case UART::DataBits::Six:
        return CS6;
    case UART::DataBits::Seven:
        return CS7;
    case UART::DataBits::Eight:
    default:
        return CS8;
    }
}
#endif

} // namespace

UART::LinuxSerialPort::LinuxSerialPort(const std::string& device_path,
                                       const Config& config)
    : device_path_(device_path),
      config_(config),
      fd_(-1)
{
}

UART::LinuxSerialPort::~LinuxSerialPort()
{
    close();
}

bool UART::LinuxSerialPort::open()
{
#if defined(__linux__)
    if (isOpen()) {
        return true;
    }

    int flags = O_RDWR | O_NOCTTY;
    if (config_.non_blocking) {
        flags |= O_NONBLOCK;
    }

    fd_ = ::open(device_path_.c_str(), flags);
    if (fd_ < 0) {
        return false;
    }

    if (!applyConfig(config_)) {
        close();
        return false;
    }

    return true;
#else
    return false;
#endif
}

void UART::LinuxSerialPort::close()
{
#if defined(__linux__)
    if (fd_ >= 0) {
        ::close(fd_);
        fd_ = -1;
    }
#else
    fd_ = -1;
#endif
}

bool UART::LinuxSerialPort::isOpen() const
{
    return fd_ >= 0;
}

int UART::LinuxSerialPort::read(std::uint8_t* data, std::size_t length)
{
#if defined(__linux__)
    if (!isOpen() || data == nullptr || length == 0) {
        return -1;
    }

    return static_cast<int>(::read(fd_, data, length));
#else
    (void)data;
    (void)length;
    return -1;
#endif
}

int UART::LinuxSerialPort:: write(const std::uint8_t* data, std::size_t length)
{
#if defined(__linux__)
    if (!isOpen() || data == nullptr || length == 0) {
        return -1;
    }

    return static_cast<int>(::write(fd_, data, length));
#else
    (void)data;
    (void)length;
    return -1;
#endif
}

bool UART::LinuxSerialPort::flushInput()
{
#if defined(__linux__)
    return isOpen() && ::tcflush(fd_, TCIFLUSH) == 0;
#else
    return false;
#endif
}

bool UART::LinuxSerialPort::flushOutput()
{
#if defined(__linux__)
    return isOpen() && ::tcflush(fd_, TCOFLUSH) == 0;
#else
    return false;
#endif
}

bool UART::LinuxSerialPort::flushBoth()
{
#if defined(__linux__)
    return isOpen() && ::tcflush(fd_, TCIOFLUSH) == 0;
#else
    return false;
#endif
}

bool UART::LinuxSerialPort::drainOutput()
{
#if defined(__linux__)
    return isOpen() && ::tcdrain(fd_) == 0;
#else
    return false;
#endif
}

bool UART::LinuxSerialPort::applyConfig(const Config& config)
{
#if defined(__linux__)
    if (!isOpen()) {
        return false;
    }

    termios options = {};
    if (::tcgetattr(fd_, &options) != 0) {
        return false;
    }

    ::cfmakeraw(&options);

    const speed_t speed = toLinuxBaudRate(config.baud_rate);
    if (::cfsetispeed(&options, speed) != 0 ||
        ::cfsetospeed(&options, speed) != 0) {
        return false;
    }

    options.c_cflag &= static_cast<tcflag_t>(~CSIZE);
    options.c_cflag |= toLinuxDataBits(config.data_bits);
    options.c_cflag |= CLOCAL | CREAD;

    if (config.stop_bits == StopBits::Two) {
        options.c_cflag |= CSTOPB;
    } else {
        options.c_cflag &= static_cast<tcflag_t>(~CSTOPB);
    }

    if (config.parity == Parity::None) {
        options.c_cflag &= static_cast<tcflag_t>(~PARENB);
    } else {
        options.c_cflag |= PARENB;
        if (config.parity == Parity::Odd) {
            options.c_cflag |= PARODD;
        } else {
            options.c_cflag &= static_cast<tcflag_t>(~PARODD);
        }
    }

    options.c_iflag &= static_cast<tcflag_t>(~(IXON | IXOFF | IXANY));
    options.c_cflag &= static_cast<tcflag_t>(~CRTSCTS);

    if (config.flow_control == FlowControl::Software) {
        options.c_iflag |= IXON | IXOFF;
    } else if (config.flow_control == FlowControl::Hardware) {
        options.c_cflag |= CRTSCTS;
    }

    options.c_cc[VTIME] = config.read_timeout_deciseconds;
    options.c_cc[VMIN] = config.minimum_read_bytes;

    if (::tcsetattr(fd_, TCSANOW, &options) != 0) {
        return false;
    }

    config_ = config;
    return true;
#else
    (void)config;
    return false;
#endif
}

const std::string& UART::LinuxSerialPort::devicePath() const
{
    return device_path_;
}

const UART::Config& UART::LinuxSerialPort::config() const
{
    return config_;
}

int UART::LinuxSerialPort::fileDescriptor() const
{
    return fd_;
}

BLE60::BLE60(UART::SerialPort& serial)
    : serial_(serial)
{
}

bool BLE60::initialize()
{
    if (!serial_.isOpen() && !serial_.open()) {
        return false;
    }

    return serial_.flushBoth();
}

bool BLE60::isOpen() const
{
    return serial_.isOpen();
}

int BLE60::send(const std::string& cmd, const std::uint8_t* data, std::size_t length)
{
    if (data == nullptr || length == 0) {
        return -1;
    }
    std::string message = cmd + kCommandTerminator;
    return serial_.write((const std::uint8_t*)message.c_str(), message.length());
}

int BLE60::receive(std::uint8_t* data, std::size_t length)
{
    if (data == nullptr || length == 0) {
        return -1;
    }

    return serial_.read(data, length);
}

bool BLE60::sendText(const std::string& text)
{
    if (text.empty()) {
        return false;
    }

    const int written = serial_.write(
        reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
    return written == static_cast<int>(text.size()) && serial_.drainOutput();
}

bool BLE60::sendCommand(const std::string& command)
{
    if (command.empty()) {
        return false;
    }

    std::string framed_command = command;
    framed_command += kCommandTerminator;
    return sendText(framed_command);
}

bool BLE60::send_and_wait(const std::string& command,
                          const std::string& terminator,
                          std::string&       response,
                          int                safety_timeout_ms)
{
    using clock = std::chrono::steady_clock;
    using ms    = std::chrono::milliseconds;

    response.clear();

    if (!command.empty() && !sendCommand(command)) {
        return false;
    }

    const auto start = clock::now();

    while (std::chrono::duration_cast<ms>(clock::now() - start).count()
           < safety_timeout_ms) {
        std::uint8_t b = 0;
        const int n = serial_.read(&b, 1);

        if (n > 0) {
            response.push_back(static_cast<char>(b));

            if (response.find(terminator) != std::string::npos) {
                // Drain anything already queued, capped at 50 ms
                const auto drain_start = clock::now();
                while (std::chrono::duration_cast<ms>(clock::now() - drain_start).count() < 50) {
                    std::uint8_t extra = 0;
                    if (serial_.read(&extra, 1) <= 0) break;
                    response.push_back(static_cast<char>(extra));
                }
                return true;
            }
        } else {
            std::this_thread::sleep_for(ms(5));
        }
    }
    return false;
}

bool BLE60::readResponse(std::string& out,
                        int quiet_ms,
                        int timeout_ms)
{
    using clock = std::chrono::steady_clock;
    using ms    = std::chrono::milliseconds;

    out.clear();

    const auto start     = clock::now();
    auto       last_byte = start;
    bool       got_byte  = false;

    while (true) {
        const auto now     = clock::now();
        const auto elapsed = std::chrono::duration_cast<ms>(now - start).count();
        const auto quiet   = std::chrono::duration_cast<ms>(now - last_byte).count();

        if (elapsed >= timeout_ms) {
            break;                       // overall timeout reached
        }

        std::uint8_t b = 0;
        const int n = serial_.read(&b, 1);

        if (n > 0) {
            out.push_back(static_cast<char>(b));
            last_byte = clock::now();
            got_byte  = true;
        } else {
            // No byte this iteration.
            if (got_byte && quiet >= quiet_ms) {
                return true;             // quiet period satisfied  reply complete
            }
            std::this_thread::sleep_for(ms(5));
        }
    }

    return got_byte && !out.empty();
}

bool BLE60::flush()
{
    return serial_.flushBoth();
}

UART::SerialPort& BLE60::serialPort()
{
    return serial_;
}

bool BLE60::enterCommandMode()
{
    using ms = std::chrono::milliseconds;

    // Escape sequence has NO terminator.
    // A#A\$% in a C++ string literal:
    static const char kEscape[] = "^#^$^%";
    const std::size_t escape_len = sizeof(kEscape) - 1;

    if (!serial_.flushBoth()) {
        return false;
    }

    const int written = serial_.write(
        reinterpret_cast<const std::uint8_t*>(kEscape), escape_len);
    if (written != static_cast<int>(escape_len) || !serial_.drainOutput()) {
        return false;
    }

    // The module only responds after ~2 seconds of no traffic.
    std::this_thread::sleep_for(ms(2500));

    std::string response;
    if (!readResponse(response, 64)) {
        return false;
    }

    // Look for "CommandMode" in the reply.
    return response.find("CommandMode") != std::string::npos;
}

bool BLE60::enterBypassMode()
{
    std::string response;
    send_and_wait("AT+AB LeBypass", Terminator::BypassMode, response);
    if (response.empty()){
        return -1;
    }
    return response.find("BypassMode") != std::string::npos;
}

bool BLE60::reset()
{
    // Reset does not produce a normal reply; it reboots the BLE60.
    return sendCommand("AT+AB Reset");
}

int BLE60::sendRaw(const std::uint8_t* data, std::size_t length)
{
    if (!isOpen() || data == nullptr || length == 0) {
        return -1;
    }
    const int written = serial_.write(data, length);
    if (written != static_cast<int>(length)) {
        return written;
    }
    if (!serial_.drainOutput()) {
        return -1;
    }
    return written;
}

bool BLE60::transactUntil(const std::string& command,
                          const std::string& expected_token,
                          std::string& accumulated,
                          int timeout_ms) {

    using ms = std::chrono::milliseconds;
    accumulated.clear();
    
    if (!sendCommand(command)) {
        return false;
    }

    auto deadline = std::chrono::steady_clock::now()
                  + std::chrono::milliseconds(timeout_ms);

    while (std::chrono::steady_clock::now() < deadline) {
        std::uint8_t byte = 0;
        int n = serial_.read(&byte, 1);
        if (n > 0) {
            accumulated.push_back(static_cast<char>(byte));
            if (accumulated.find(expected_token) != std::string::npos) {
                return true;
            }
        } else {
            // small sleep so we do not busy-loop on the UART
            std::this_thread::sleep_for(ms(5));
        }
    }
    return false;
}

} // namespace haptic_feedback
