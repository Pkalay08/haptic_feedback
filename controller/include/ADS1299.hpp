#ifndef HAPTIC_FEEDBACK_ADS1299_HPP
#define HAPTIC_FEEDBACK_ADS1299_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace haptic_feedback {

class ADS1299 {
public:
    static constexpr std::size_t kMaxChannels = 8;
    static constexpr std::size_t kStatusBytes = 3;
    static constexpr std::size_t kBytesPerSample = 3;
    static constexpr std::size_t kFrameBytes =
        kStatusBytes + (kMaxChannels * kBytesPerSample);

    enum class Command : std::uint8_t {
        WAKEUP = 0x02,
        STANDBY = 0x04,
        RESET = 0x06,
        START = 0x08,
        STOP = 0x0A,
        RDATAC = 0x10,
        SDATAC = 0x11,
        RDATA = 0x12,
    };

    enum class Register : std::uint8_t {
        ID = 0x00,
        CONFIG1 = 0x96,
        CONFIG2 = 0xD1,
        CONFIG3 = 0x6E,
        LOFF = 0x00,
        CH1SET = 0x60,
        CH2SET = 0x61,
        CH3SET = 0x61,
        CH4SET = 0x61,
        CH5SET = 0x61,
        CH6SET = 0x61,
        CH7SET = 0x61,
        CH8SET = 0x61,
        BIAS_SENSP = 0x01,
        BIAS_SENSN = 0x01,
        LOFF_SENSP = 0x00,
        LOFF_SENSN = 0x00,
        LOFF_FLIP = 0x00,
        LOFF_STATP = 0x00,
        LOFF_STATN = 0x00,
        GPIO = 0x00,
        MISC1 = 0x00,
        MISC2 = 0x00,
        CONFIG4 = 0x00,
    };

    enum class DataRate : std::uint8_t {
        SPS_16000 = 0x00,
        SPS_8000 = 0x01,
        SPS_4000 = 0x02,
        SPS_2000 = 0x03,
        SPS_1000 = 0x04,
        SPS_500 = 0x05,
        SPS_250 = 0x06,
    };

    enum class Gain : std::uint8_t {
        X1 = 0x01,
        X2 = 0x02,
        X4 = 0x04,
        X6 = 0x00,
        X8 = 0x05,
        X12 = 0x06,
        X24 = 0x07,
    };

    enum class ChannelInput : std::uint8_t {
        NormalElectrode = 0x00,
        InputShorted = 0x01,
        BiasMeasurement = 0x02,
        Mvdd = 0x03,
        Temperature = 0x04,
        TestSignal = 0x05,
        BiasDrivePositive = 0x06,
        BiasDriveNegative = 0x07,
    };

    enum class TestSignalAmplitude : std::uint8_t {
        OneTimes = 0x00,
        TwoTimes = 0x04,
    };

    enum class TestSignalFrequency : std::uint8_t {
        PulsedAtFclkDiv21 = 0x00,
        PulsedAtFclkDiv20 = 0x01,
        Dc = 0x03,
    };

    struct ChannelConfig {
        bool power_down;
        Gain gain;
        bool srb2_connected;
        ChannelInput input;
    };

    struct DeviceConfig {
        DataRate data_rate;
        bool daisy_chain_enabled;
        bool clock_output_enabled;
        bool internal_reference_enabled;
        bool bias_buffer_enabled;
    };

    struct SampleFrame {
        std::array<std::uint8_t, kStatusBytes> status;
        std::array<std::int32_t, kMaxChannels> channels;
    };

    class SpiBus {
    public:
        virtual ~SpiBus() {}
        virtual bool transfer(const std::uint8_t* tx,
                              std::uint8_t* rx,
                              std::size_t length) = 0;
    };

    struct LinuxSpiConfig {
        std::uint32_t speed_hz;
        std::uint8_t mode;
        std::uint8_t bits_per_word;
        std::uint16_t delay_usecs;
    };

    class LinuxSpiBus : public SpiBus {
    public:
        static constexpr std::uint8_t kSpiMode1 = 0x01;
        static constexpr std::uint8_t kDefaultBitsPerWord = 8;
        static constexpr std::uint32_t kDefaultSpeedHz = 1000000;

        explicit LinuxSpiBus(const std::string& device_path);
        LinuxSpiBus(const std::string& device_path, const LinuxSpiConfig& config);
        ~LinuxSpiBus();

        LinuxSpiBus(const LinuxSpiBus&) = delete;
        LinuxSpiBus& operator=(const LinuxSpiBus&) = delete;

        bool open();
        void close();
        bool isOpen() const;
        const std::string& devicePath() const;
        const LinuxSpiConfig& config() const;

        bool transfer(const std::uint8_t* tx,
                      std::uint8_t* rx,
                      std::size_t length) override;

    private:
        bool configureDevice();

        std::string device_path_;
        LinuxSpiConfig config_;
        int fd_;
    };

    explicit ADS1299(SpiBus& spi);

    bool reset();
    bool wakeup();
    bool standby();
    bool start();
    bool stop();

    bool enableContinuousRead();
    bool disableContinuousRead();
    bool readDataOnce(SampleFrame& frame);

    bool readRegister(Register reg, std::uint8_t& value);
    bool writeRegister(Register reg, std::uint8_t value);
    bool readRegisters(Register start_reg, std::uint8_t* values, std::size_t count);
    bool writeRegisters(Register start_reg, const std::uint8_t* values, std::size_t count);

    bool configureDevice(const DeviceConfig& config);
    bool configureChannel(std::uint8_t channel, const ChannelConfig& config);
    bool setChannelPower(std::uint8_t channel, bool power_down);
    bool setBiasSensePositive(std::uint8_t channel_mask);
    bool setBiasSenseNegative(std::uint8_t channel_mask);
    bool setLeadOffSensePositive(std::uint8_t channel_mask);
    bool setLeadOffSenseNegative(std::uint8_t channel_mask);

    std::uint8_t deviceId() const;

    static constexpr std::uint8_t registerAddress(Register reg)
    {
        return static_cast<std::uint8_t>(reg);
    }

    static constexpr std::uint8_t commandByte(Command command)
    {
        return static_cast<std::uint8_t>(command);
    }

    static constexpr std::uint8_t readRegisterCommand(Register reg)
    {
        return static_cast<std::uint8_t>(0x20 | registerAddress(reg));
    }

    static constexpr std::uint8_t writeRegisterCommand(Register reg)
    {
        return static_cast<std::uint8_t>(0x40 | registerAddress(reg));
    }

private:
    static constexpr std::uint8_t kConfig1DataRateMask = 0x07;
    static constexpr std::uint8_t kConfig1DaisyChainBit = 0x40;
    static constexpr std::uint8_t kConfig1ClockEnableBit = 0x20;
    static constexpr std::uint8_t kConfig2TestAmplitudeMask = 0x04;
    static constexpr std::uint8_t kConfig2TestFrequencyMask = 0x03;
    static constexpr std::uint8_t kConfig3ReferenceBufferBit = 0x80;
    static constexpr std::uint8_t kConfig3BiasBufferBit = 0x04;
    static constexpr std::uint8_t kChannelPowerDownBit = 0x80;
    static constexpr std::uint8_t kChannelGainShift = 4;
    static constexpr std::uint8_t kChannelSrb2Bit = 0x08;
    static constexpr std::uint8_t kChannelMuxMask = 0x07;

    SpiBus& spi_;
    std::uint8_t device_id_;
};

} // namespace haptic_feedback

#endif // HAPTIC_FEEDBACK_ADS1299_HPP
