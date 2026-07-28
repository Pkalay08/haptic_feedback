#include "ADS1299.hpp"

#include <array>

#if defined(__linux__)
#include <fcntl.h>
#include <linux/spi/spidev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#endif

namespace haptic_feedback {
namespace {

constexpr std::size_t kRegisterCount =
    ADS1299::registerAddress(ADS1299::Register::CONFIG4) + 1;
constexpr std::size_t kMaxRegisterTransferBytes = kRegisterCount + 2;

bool validChannel(std::uint8_t channel)
{
    return channel >= 1 && channel <= ADS1299::kMaxChannels;
}

ADS1299::Register channelRegister(std::uint8_t channel)
{
    return static_cast<ADS1299::Register>(
        ADS1299::registerAddress(ADS1299::Register::CH1SET) + channel - 1);
}

std::int32_t convert24BitTwosComplement(const std::uint8_t* data)
{
    std::int32_t value = (static_cast<std::int32_t>(data[0]) << 16) |
                         (static_cast<std::int32_t>(data[1]) << 8) |
                         static_cast<std::int32_t>(data[2]);

    if ((value & 0x00800000) != 0) {
        value |= static_cast<std::int32_t>(0xFF000000);
    }

    return value;
}

void parseSampleFrame(const std::array<std::uint8_t, ADS1299::kFrameBytes>& raw,
                      ADS1299::SampleFrame& frame)
{
    for (std::size_t i = 0; i < ADS1299::kStatusBytes; ++i) {
        frame.status[i] = raw[i];
    }

    for (std::size_t channel = 0; channel < ADS1299::kMaxChannels; ++channel) {
        const std::size_t offset =
            ADS1299::kStatusBytes + (channel * ADS1299::kBytesPerSample);
        frame.channels[channel] = convert24BitTwosComplement(&raw[offset]);
    }
}

} // namespace

ADS1299::LinuxSpiBus::LinuxSpiBus(const std::string& device_path)
    : device_path_(device_path),
      config_{kDefaultSpeedHz, kSpiMode1, kDefaultBitsPerWord, 0},
      fd_(-1)
{
}

ADS1299::LinuxSpiBus::LinuxSpiBus(const std::string& device_path,
                                  const LinuxSpiConfig& config)
    : device_path_(device_path),
      config_(config),
      fd_(-1)
{
}

ADS1299::LinuxSpiBus::~LinuxSpiBus()
{
    close();
}

bool ADS1299::LinuxSpiBus::open()
{
#if defined(__linux__)
    if (isOpen()) {
        return true;
    }

    fd_ = ::open(device_path_.c_str(), O_RDWR);
    if (fd_ < 0) {
        return false;
    }

    if (!configureDevice()) {
        close();
        return false;
    }

    return true;
#else
    return false;
#endif
}

void ADS1299::LinuxSpiBus::close()
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

bool ADS1299::LinuxSpiBus::isOpen() const
{
    return fd_ >= 0;
}

const std::string& ADS1299::LinuxSpiBus::devicePath() const
{
    return device_path_;
}

const ADS1299::LinuxSpiConfig& ADS1299::LinuxSpiBus::config() const
{
    return config_;
}

bool ADS1299::LinuxSpiBus::transfer(const std::uint8_t* tx,
                                    std::uint8_t* rx,
                                    std::size_t length)
{
#if defined(__linux__)
    if (!isOpen() || tx == nullptr || rx == nullptr || length == 0) {
        return false;
    }

    spi_ioc_transfer transfer = {};
    transfer.tx_buf = reinterpret_cast<unsigned long>(tx);
    transfer.rx_buf = reinterpret_cast<unsigned long>(rx);
    transfer.len = static_cast<__u32>(length);
    transfer.speed_hz = config_.speed_hz;
    transfer.delay_usecs = config_.delay_usecs;
    transfer.bits_per_word = config_.bits_per_word;

    return ::ioctl(fd_, SPI_IOC_MESSAGE(1), &transfer) == 1;
#else
    (void)tx;
    (void)rx;
    (void)length;
    return false;
#endif
}

bool ADS1299::LinuxSpiBus::configureDevice()
{
#if defined(__linux__)
    if (!isOpen()) {
        return false;
    }

    std::uint8_t mode = config_.mode;
    std::uint8_t bits = config_.bits_per_word;
    std::uint32_t speed = config_.speed_hz;

    return ::ioctl(fd_, SPI_IOC_WR_MODE, &mode) >= 0 &&
           ::ioctl(fd_, SPI_IOC_RD_MODE, &mode) >= 0 &&
           ::ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits) >= 0 &&
           ::ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits) >= 0 &&
           ::ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed) >= 0 &&
           ::ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed) >= 0;
#else
    return false;
#endif
}

ADS1299::ADS1299(SpiBus& spi)
    : spi_(spi),
      device_id_(0)
{
}

bool ADS1299::reset()
{
    const std::uint8_t tx = commandByte(Command::RESET);
    std::uint8_t rx = 0;

    if (!spi_.transfer(&tx, &rx, 1)) {
        return false;
    }

#if defined(__linux__)
    ::usleep(10000);
#endif

    std::uint8_t id = 0;
    if (readRegister(Register::ID, id)) {
        device_id_ = id;
    }

    return true;
}

bool ADS1299::wakeup()
{
    const std::uint8_t tx = commandByte(Command::WAKEUP);
    std::uint8_t rx = 0;
    return spi_.transfer(&tx, &rx, 1);
}

bool ADS1299::standby()
{
    const std::uint8_t tx = commandByte(Command::STANDBY);
    std::uint8_t rx = 0;
    return spi_.transfer(&tx, &rx, 1);
}

bool ADS1299::start()
{
    const std::uint8_t tx = commandByte(Command::START);
    std::uint8_t rx = 0;
    return spi_.transfer(&tx, &rx, 1);
}

bool ADS1299::stop()
{
    const std::uint8_t tx = commandByte(Command::STOP);
    std::uint8_t rx = 0;
    return spi_.transfer(&tx, &rx, 1);
}

bool ADS1299::enableContinuousRead()
{
    const std::uint8_t tx = commandByte(Command::RDATAC);
    std::uint8_t rx = 0;
    return spi_.transfer(&tx, &rx, 1);
}

bool ADS1299::disableContinuousRead()
{
    const std::uint8_t tx = commandByte(Command::SDATAC);
    std::uint8_t rx = 0;
    return spi_.transfer(&tx, &rx, 1);
}

bool ADS1299::readDataOnce(SampleFrame& frame)
{
    std::array<std::uint8_t, kFrameBytes + 1> tx = {};
    std::array<std::uint8_t, kFrameBytes + 1> rx = {};
    tx[0] = commandByte(Command::RDATA);

    if (!spi_.transfer(tx.data(), rx.data(), tx.size())) {
        return false;
    }

    std::array<std::uint8_t, kFrameBytes> raw = {};
    for (std::size_t i = 0; i < raw.size(); ++i) {
        raw[i] = rx[i + 1];
    }

    parseSampleFrame(raw, frame);
    return true;
}

bool ADS1299::readRegister(Register reg, std::uint8_t& value)
{
    return readRegisters(reg, &value, 1);
}

bool ADS1299::writeRegister(Register reg, std::uint8_t value)
{
    return writeRegisters(reg, &value, 1);
}

bool ADS1299::readRegisters(Register start_reg,
                            std::uint8_t* values,
                            std::size_t count)
{
    if (values == nullptr || count == 0 || count > kRegisterCount) {
        return false;
    }

    const std::uint8_t start = registerAddress(start_reg);
    if (start >= kRegisterCount || (start + count) > kRegisterCount) {
        return false;
    }

    std::array<std::uint8_t, kMaxRegisterTransferBytes> tx = {};
    std::array<std::uint8_t, kMaxRegisterTransferBytes> rx = {};
    const std::size_t transfer_length = count + 2;

    tx[0] = readRegisterCommand(start_reg);
    tx[1] = static_cast<std::uint8_t>(count - 1);

    if (!spi_.transfer(tx.data(), rx.data(), transfer_length)) {
        return false;
    }

    for (std::size_t i = 0; i < count; ++i) {
        values[i] = rx[i + 2];
    }

    if (start_reg == Register::ID && count > 0) {
        device_id_ = values[0];
    }

    return true;
}

bool ADS1299::writeRegisters(Register start_reg,
                             const std::uint8_t* values,
                             std::size_t count)
{
    if (values == nullptr || count == 0 || count > kRegisterCount) {
        return false;
    }

    const std::uint8_t start = registerAddress(start_reg);
    if (start >= kRegisterCount || (start + count) > kRegisterCount) {
        return false;
    }

    std::array<std::uint8_t, kMaxRegisterTransferBytes> tx = {};
    std::array<std::uint8_t, kMaxRegisterTransferBytes> rx = {};
    const std::size_t transfer_length = count + 2;

    tx[0] = writeRegisterCommand(start_reg);
    tx[1] = static_cast<std::uint8_t>(count - 1);
    for (std::size_t i = 0; i < count; ++i) {
        tx[i + 2] = values[i];
    }

    return spi_.transfer(tx.data(), rx.data(), transfer_length);
}

bool ADS1299::configureDevice(const DeviceConfig& config)
{
    std::uint8_t config1 = 0;
    std::uint8_t config3 = 0;

    if (!readRegister(Register::CONFIG1, config1) ||
        !readRegister(Register::CONFIG3, config3)) {
        return false;
    }

    config1 &= static_cast<std::uint8_t>(~kConfig1DataRateMask);
    config1 |= static_cast<std::uint8_t>(config.data_rate) & kConfig1DataRateMask;

    if (config.daisy_chain_enabled) {
        config1 &= static_cast<std::uint8_t>(~kConfig1DaisyChainBit);
    } else {
        config1 |= kConfig1DaisyChainBit;
    }

    if (config.clock_output_enabled) {
        config1 |= kConfig1ClockEnableBit;
    } else {
        config1 &= static_cast<std::uint8_t>(~kConfig1ClockEnableBit);
    }

    if (config.internal_reference_enabled) {
        config3 |= kConfig3ReferenceBufferBit;
    } else {
        config3 &= static_cast<std::uint8_t>(~kConfig3ReferenceBufferBit);
    }

    if (config.bias_buffer_enabled) {
        config3 |= kConfig3BiasBufferBit;
    } else {
        config3 &= static_cast<std::uint8_t>(~kConfig3BiasBufferBit);
    }

    return writeRegister(Register::CONFIG1, config1) &&
           writeRegister(Register::CONFIG3, config3);
}

bool ADS1299::configureChannel(std::uint8_t channel,
                               const ChannelConfig& config)
{
    if (!validChannel(channel)) {
        return false;
    }

    std::uint8_t value = 0;

    if (config.power_down) {
        value |= kChannelPowerDownBit;
    }

    value |= static_cast<std::uint8_t>(
        (static_cast<std::uint8_t>(config.gain) << kChannelGainShift) & 0x70);

    if (config.srb2_connected) {
        value |= kChannelSrb2Bit;
    }

    value |= static_cast<std::uint8_t>(
        static_cast<std::uint8_t>(config.input) & kChannelMuxMask);

    return writeRegister(channelRegister(channel), value);
}

bool ADS1299::setChannelPower(std::uint8_t channel, bool power_down)
{
    if (!validChannel(channel)) {
        return false;
    }

    const Register reg = channelRegister(channel);
    std::uint8_t value = 0;
    if (!readRegister(reg, value)) {
        return false;
    }

    if (power_down) {
        value |= kChannelPowerDownBit;
    } else {
        value &= static_cast<std::uint8_t>(~kChannelPowerDownBit);
    }

    return writeRegister(reg, value);
}

bool ADS1299::setBiasSensePositive(std::uint8_t channel_mask)
{
    return writeRegister(Register::BIAS_SENSP, channel_mask);
}

bool ADS1299::setBiasSenseNegative(std::uint8_t channel_mask)
{
    return writeRegister(Register::BIAS_SENSN, channel_mask);
}

bool ADS1299::setLeadOffSensePositive(std::uint8_t channel_mask)
{
    return writeRegister(Register::LOFF_SENSP, channel_mask);
}

bool ADS1299::setLeadOffSenseNegative(std::uint8_t channel_mask)
{
    return writeRegister(Register::LOFF_SENSN, channel_mask);
}

std::uint8_t ADS1299::deviceId() const
{
    return device_id_;
}

} // namespace haptic_feedback
