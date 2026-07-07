#include "MPU6050.hpp"

#include <vector>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <linux/i2c.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace haptic_feedback {
namespace {

constexpr std::size_t kMaxRegister = 0x75;

std::int16_t makeInt16(std::uint8_t high, std::uint8_t low)
{
    return static_cast<std::int16_t>(
        (static_cast<std::uint16_t>(high) << 8) | low);
}

MPU6050::Vector3Raw parseVector3(const std::uint8_t* data)
{
    MPU6050::Vector3Raw value = {};
    value.x = makeInt16(data[0], data[1]);
    value.y = makeInt16(data[2], data[3]);
    value.z = makeInt16(data[4], data[5]);
    return value;
}

} // namespace

MPU6050::LinuxI2cBus::LinuxI2cBus(const std::string& device_path)
    : device_path_(device_path),
      fd_(-1)
{
}

MPU6050::LinuxI2cBus::~LinuxI2cBus()
{
    close();
}

bool MPU6050::LinuxI2cBus::open()
{
#if defined(__linux__)
    if (isOpen()) {
        return true;
    }

    fd_ = ::open(device_path_.c_str(), O_RDWR);
    return fd_ >= 0;
#else
    return false;
#endif
}

void MPU6050::LinuxI2cBus::close()
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

bool MPU6050::LinuxI2cBus::isOpen() const
{
    return fd_ >= 0;
}

const std::string& MPU6050::LinuxI2cBus::devicePath() const
{
    return device_path_;
}

bool MPU6050::LinuxI2cBus::write(std::uint8_t device_address,
                                 const std::uint8_t* data,
                                 std::size_t length)
{
#if defined(__linux__)
    if (!isOpen() || data == nullptr || length == 0) {
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, device_address) < 0) {
        return false;
    }

    const ssize_t written = ::write(fd_, data, length);
    return written == static_cast<ssize_t>(length);
#else
    (void)device_address;
    (void)data;
    (void)length;
    return false;
#endif
}

bool MPU6050::LinuxI2cBus::read(std::uint8_t device_address,
                                std::uint8_t* data,
                                std::size_t length)
{
#if defined(__linux__)
    if (!isOpen() || data == nullptr || length == 0) {
        return false;
    }

    if (::ioctl(fd_, I2C_SLAVE, device_address) < 0) {
        return false;
    }

    const ssize_t bytes_read = ::read(fd_, data, length);
    return bytes_read == static_cast<ssize_t>(length);
#else
    (void)device_address;
    (void)data;
    (void)length;
    return false;
#endif
}

bool MPU6050::LinuxI2cBus::writeRead(std::uint8_t device_address,
                                     const std::uint8_t* tx,
                                     std::size_t tx_length,
                                     std::uint8_t* rx,
                                     std::size_t rx_length)
{
#if defined(__linux__)
    if (!isOpen() || tx == nullptr || tx_length == 0 ||
        rx == nullptr || rx_length == 0) {
        return false;
    }

    i2c_msg messages[2] = {};
    messages[0].addr = device_address;
    messages[0].flags = 0;
    messages[0].len = static_cast<__u16>(tx_length);
    messages[0].buf = const_cast<std::uint8_t*>(tx);

    messages[1].addr = device_address;
    messages[1].flags = I2C_M_RD;
    messages[1].len = static_cast<__u16>(rx_length);
    messages[1].buf = rx;

    i2c_rdwr_ioctl_data transfer = {};
    transfer.msgs = messages;
    transfer.nmsgs = 2;

    return ::ioctl(fd_, I2C_RDWR, &transfer) >= 0;
#else
    (void)device_address;
    (void)tx;
    (void)tx_length;
    (void)rx;
    (void)rx_length;
    return false;
#endif
}

MPU6050::MPU6050(I2cBus& bus, Address address)
    : bus_(bus),
      address_(address),
      who_am_i_(0)
{
}

bool MPU6050::initialize(const Config& config)
{
    return reset() &&
           wake(config.clock_source) &&
           configure(config) &&
           verifyIdentity();
}

bool MPU6050::reset()
{
    const bool ok = writeRegister(Register::PWR_MGMT_1, kDeviceResetBit);
#if defined(__linux__)
    if (ok) {
        ::usleep(100000);
    }
#endif
    return ok;
}

bool MPU6050::wake(ClockSource clock_source)
{
    const std::uint8_t value =
        static_cast<std::uint8_t>(clock_source) & kClockSourceMask;
    return writeRegister(Register::PWR_MGMT_1, value);
}

bool MPU6050::sleep()
{
    std::uint8_t value = 0;
    if (!readRegister(Register::PWR_MGMT_1, value)) {
        return false;
    }

    value |= kSleepBit;
    return writeRegister(Register::PWR_MGMT_1, value);
}

bool MPU6050::verifyIdentity()
{
    std::uint8_t id = 0;
    if (!readRegister(Register::WHO_AM_I, id)) {
        return false;
    }

    who_am_i_ = id;
    return id == kExpectedWhoAmI;
}

bool MPU6050::configure(const Config& config)
{
    return setClockSource(config.clock_source) &&
           setDigitalLowPassFilter(config.dlpf) &&
           setGyroRange(config.gyro_range) &&
           setAccelRange(config.accel_range) &&
           setSampleRateDivider(config.sample_rate_divider) &&
           enableDataReadyInterrupt(config.data_ready_interrupt_enabled,
                                    config.interrupt_mode);
}

bool MPU6050::setClockSource(ClockSource clock_source)
{
    std::uint8_t value = 0;
    if (!readRegister(Register::PWR_MGMT_1, value)) {
        return false;
    }

    value &= static_cast<std::uint8_t>(~kClockSourceMask);
    value &= static_cast<std::uint8_t>(~kSleepBit);
    value |= static_cast<std::uint8_t>(clock_source) & kClockSourceMask;
    return writeRegister(Register::PWR_MGMT_1, value);
}

bool MPU6050::setDigitalLowPassFilter(DigitalLowPassFilter dlpf)
{
    std::uint8_t value = 0;
    if (!readRegister(Register::CONFIG, value)) {
        return false;
    }

    value &= static_cast<std::uint8_t>(~kDlpfMask);
    value |= static_cast<std::uint8_t>(dlpf) & kDlpfMask;
    return writeRegister(Register::CONFIG, value);
}

bool MPU6050::setGyroRange(GyroRange range)
{
    const std::uint8_t value =
        static_cast<std::uint8_t>(static_cast<std::uint8_t>(range)
                                  << kGyroRangeShift);
    return writeRegister(Register::GYRO_CONFIG, value);
}

bool MPU6050::setAccelRange(AccelRange range)
{
    const std::uint8_t value =
        static_cast<std::uint8_t>(static_cast<std::uint8_t>(range)
                                  << kAccelRangeShift);
    return writeRegister(Register::ACCEL_CONFIG, value);
}

bool MPU6050::setSampleRateDivider(std::uint8_t divider)
{
    return writeRegister(Register::SMPLRT_DIV, divider);
}

bool MPU6050::enableDataReadyInterrupt(bool enabled, InterruptMode mode)
{
    const std::uint8_t int_pin_cfg =
        static_cast<std::uint8_t>(mode) & kInterruptConfigMask;
    const std::uint8_t int_enable =
        enabled ? kDataReadyInterruptBit : static_cast<std::uint8_t>(0);

    return writeRegister(Register::INT_PIN_CFG, int_pin_cfg) &&
           writeRegister(Register::INT_ENABLE, int_enable);
}

bool MPU6050::readAcceleration(Vector3Raw& acceleration)
{
    std::uint8_t data[6] = {};
    if (!readRegisters(Register::ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    acceleration = parseVector3(data);
    return true;
}

bool MPU6050::readGyroscope(Vector3Raw& gyroscope)
{
    std::uint8_t data[6] = {};
    if (!readRegisters(Register::GYRO_XOUT_H, data, sizeof(data))) {
        return false;
    }

    gyroscope = parseVector3(data);
    return true;
}

bool MPU6050::readTemperature(std::int16_t& temperature)
{
    std::uint8_t data[2] = {};
    if (!readRegisters(Register::TEMP_OUT_H, data, sizeof(data))) {
        return false;
    }

    temperature = makeInt16(data[0], data[1]);
    return true;
}

bool MPU6050::readMotionFrame(MotionFrameRaw& frame)
{
    std::uint8_t data[kMotionBurstBytes] = {};
    if (!readRegisters(Register::ACCEL_XOUT_H, data, sizeof(data))) {
        return false;
    }

    frame.acceleration = parseVector3(&data[0]);
    frame.temperature = makeInt16(data[6], data[7]);
    frame.gyroscope = parseVector3(&data[8]);
    return true;
}

bool MPU6050::readInterruptStatus(std::uint8_t& status)
{
    return readRegister(Register::INT_STATUS, status);
}

bool MPU6050::readFifoCount(std::uint16_t& count)
{
    std::uint8_t data[2] = {};
    if (!readRegisters(Register::FIFO_COUNTH, data, sizeof(data))) {
        return false;
    }

    count = static_cast<std::uint16_t>(
        (static_cast<std::uint16_t>(data[0]) << 8) | data[1]);
    return true;
}

bool MPU6050::readFifo(std::uint8_t* data, std::size_t length)
{
    if (data == nullptr || length == 0) {
        return false;
    }

    return readRegisters(Register::FIFO_R_W, data, length);
}

bool MPU6050::readRegister(Register reg, std::uint8_t& value)
{
    return readRegisters(reg, &value, 1);
}

bool MPU6050::writeRegister(Register reg, std::uint8_t value)
{
    return writeRegisters(reg, &value, 1);
}

bool MPU6050::readRegisters(Register start_reg,
                            std::uint8_t* values,
                            std::size_t count)
{
    if (values == nullptr || count == 0) {
        return false;
    }

    const std::uint8_t start = registerAddress(start_reg);
    if (start > kMaxRegister || count > (kMaxRegister - start + 1)) {
        return false;
    }

    return bus_.writeRead(addressByte(), &start, 1, values, count);
}

bool MPU6050::writeRegisters(Register start_reg,
                             const std::uint8_t* values,
                             std::size_t count)
{
    if (values == nullptr || count == 0) {
        return false;
    }

    const std::uint8_t start = registerAddress(start_reg);
    if (start > kMaxRegister || count > (kMaxRegister - start + 1)) {
        return false;
    }

    std::vector<std::uint8_t> tx(count + 1);
    tx[0] = start;
    for (std::size_t i = 0; i < count; ++i) {
        tx[i + 1] = values[i];
    }

    return bus_.write(addressByte(), tx.data(), tx.size());
}

MPU6050::Address MPU6050::address() const
{
    return address_;
}

std::uint8_t MPU6050::addressByte() const
{
    return addressByte(address_);
}

std::uint8_t MPU6050::whoAmI() const
{
    return who_am_i_;
}

} // namespace haptic_feedback
