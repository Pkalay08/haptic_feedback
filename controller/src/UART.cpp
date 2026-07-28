#include "UART.hpp"

#include <string>

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
    case UART::BaudRate::B9600:
        return B9600;
    case UART::BaudRate::B19200:
        return B19200;
    case UART::BaudRate::B38400:
        return B38400;
    case UART::BaudRate::B57600:
        return B57600;
    case UART::BaudRate::B115200:
        return B115200;
    case UART::BaudRate::B230400:
        return B230400;
    case UART::BaudRate::B460800:
        return B460800;
    case UART::BaudRate::B921600:
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

int UART::LinuxSerialPort::write(const std::uint8_t* data, std::size_t length)
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

int BLE60::send(const std::uint8_t* data, std::size_t length)
{
    if (data == nullptr || length == 0) {
        return -1;
    }

    return serial_.write(data, length);
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

bool BLE60::transactCommand(const std::string& command,
                            std::string& response,
                            std::size_t max_response_bytes)
{
    response.clear();

    if (!sendCommand(command)) {
        return false;
    }

    return readResponse(response, max_response_bytes);
}

bool BLE60::readResponse(std::string& response, std::size_t max_response_bytes)
{
    response.clear();

    if (max_response_bytes == 0) {
        return false;
    }

    while (response.size() < max_response_bytes) {
        std::uint8_t byte = 0;
        const int bytes_read = serial_.read(&byte, 1);

        if (bytes_read < 0) {
            return false;
        }

        if (bytes_read == 0) {
            break;
        }

        response.push_back(static_cast<char>(byte));
        if (response.size() >= 2 &&
            response[response.size() - 2] == '\r' &&
            response[response.size() - 1] == '\n') {
            break;
        }
    }

    return !response.empty();
}

bool BLE60::flush()
{
    return serial_.flushBoth();
}

UART::SerialPort& BLE60::serialPort()
{
    return serial_;
}

} // namespace haptic_feedback
