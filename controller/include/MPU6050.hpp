#ifndef HAPTIC_FEEDBACK_MPU6050_HPP
#define HAPTIC_FEEDBACK_MPU6050_HPP

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>

namespace haptic_feedback {

class MPU6050 {
public:
    static constexpr std::uint8_t kAddressAd0Low = 0x68;
    static constexpr std::uint8_t kAddressAd0High = 0x69;
    static constexpr std::uint8_t kExpectedWhoAmI = 0x68;
    static constexpr std::size_t kAxisCount = 3;
    static constexpr std::size_t kMotionBurstBytes = 14;

    enum class Address : std::uint8_t {
        AD0_LOW = kAddressAd0Low,
        AD0_HIGH = kAddressAd0High,
    };

    enum class ConnectorPin : std::uint8_t {
        VCC = 1,
        GND = 2,
        SCL = 3,
        SDA = 4,
        INT = 5,
        AD0 = 6,
    };

    enum class Register : std::uint8_t {
        SELF_TEST_X = 0x0D,
        SELF_TEST_Y = 0x0E,
        SELF_TEST_Z = 0x0F,
        SELF_TEST_A = 0x10,
        SMPLRT_DIV = 0x19,
        CONFIG = 0x1A,
        GYRO_CONFIG = 0x1B,
        ACCEL_CONFIG = 0x1C,
        FIFO_EN = 0x23,
        I2C_MST_CTRL = 0x24,
        I2C_SLV0_ADDR = 0x25,
        I2C_SLV0_REG = 0x26,
        I2C_SLV0_CTRL = 0x27,
        I2C_SLV1_ADDR = 0x28,
        I2C_SLV1_REG = 0x29,
        I2C_SLV1_CTRL = 0x2A,
        I2C_SLV2_ADDR = 0x2B,
        I2C_SLV2_REG = 0x2C,
        I2C_SLV2_CTRL = 0x2D,
        I2C_SLV3_ADDR = 0x2E,
        I2C_SLV3_REG = 0x2F,
        I2C_SLV3_CTRL = 0x30,
        I2C_SLV4_ADDR = 0x31,
        I2C_SLV4_REG = 0x32,
        I2C_SLV4_DO = 0x33,
        I2C_SLV4_CTRL = 0x34,
        I2C_SLV4_DI = 0x35,
        I2C_MST_STATUS = 0x36,
        INT_PIN_CFG = 0x37,
        INT_ENABLE = 0x38,
        INT_STATUS = 0x3A,
        ACCEL_XOUT_H = 0x3B,
        ACCEL_XOUT_L = 0x3C,
        ACCEL_YOUT_H = 0x3D,
        ACCEL_YOUT_L = 0x3E,
        ACCEL_ZOUT_H = 0x3F,
        ACCEL_ZOUT_L = 0x40,
        TEMP_OUT_H = 0x41,
        TEMP_OUT_L = 0x42,
        GYRO_XOUT_H = 0x43,
        GYRO_XOUT_L = 0x44,
        GYRO_YOUT_H = 0x45,
        GYRO_YOUT_L = 0x46,
        GYRO_ZOUT_H = 0x47,
        GYRO_ZOUT_L = 0x48,
        SIGNAL_PATH_RESET = 0x68,
        USER_CTRL = 0x6A,
        PWR_MGMT_1 = 0x6B,
        PWR_MGMT_2 = 0x6C,
        FIFO_COUNTH = 0x72,
        FIFO_COUNTL = 0x73,
        FIFO_R_W = 0x74,
        WHO_AM_I = 0x75,
    };

    enum class ClockSource : std::uint8_t {
        Internal8MHz = 0x00,
        PllGyroX = 0x01,
        PllGyroY = 0x02,
        PllGyroZ = 0x03,
        PllExternal32kHz = 0x04,
        PllExternal19MHz = 0x05,
        Stop = 0x07,
    };

    enum class DigitalLowPassFilter : std::uint8_t {
        Bandwidth260Hz = 0x00,
        Bandwidth184Hz = 0x01,
        Bandwidth94Hz = 0x02,
        Bandwidth44Hz = 0x03,
        Bandwidth21Hz = 0x04,
        Bandwidth10Hz = 0x05,
        Bandwidth5Hz = 0x06,
    };

    enum class GyroRange : std::uint8_t {
        DPS_250 = 0x00,
        DPS_500 = 0x01,
        DPS_1000 = 0x02,
        DPS_2000 = 0x03,
    };

    enum class AccelRange : std::uint8_t {
        G_2 = 0x00,
        G_4 = 0x01,
        G_8 = 0x02,
        G_16 = 0x03,
    };

    enum class InterruptMode : std::uint8_t {
        ActiveHighPushPull = 0x00,
        ActiveLowOpenDrain = 0xC0,
    };

    struct Vector3Raw {
        std::int16_t x;
        std::int16_t y;
        std::int16_t z;
    };

    struct MotionFrameRaw {
        Vector3Raw acceleration;
        std::int16_t temperature;
        Vector3Raw gyroscope;
    };

    struct Config {
        ClockSource clock_source;
        DigitalLowPassFilter dlpf;
        GyroRange gyro_range;
        AccelRange accel_range;
        std::uint8_t sample_rate_divider;
        bool data_ready_interrupt_enabled;
        InterruptMode interrupt_mode;
    };

    class I2cBus {
    public:
        virtual ~I2cBus() {}
        virtual bool write(std::uint8_t device_address,
                           const std::uint8_t* data,
                           std::size_t length) = 0;
        virtual bool read(std::uint8_t device_address,
                          std::uint8_t* data,
                          std::size_t length) = 0;
        virtual bool writeRead(std::uint8_t device_address,
                               const std::uint8_t* tx,
                               std::size_t tx_length,
                               std::uint8_t* rx,
                               std::size_t rx_length) = 0;
    };

    class LinuxI2cBus : public I2cBus {
    public:
        explicit LinuxI2cBus(const std::string& device_path);
        ~LinuxI2cBus();

        LinuxI2cBus(const LinuxI2cBus&) = delete;
        LinuxI2cBus& operator=(const LinuxI2cBus&) = delete;

        bool open();
        void close();
        bool isOpen() const;
        const std::string& devicePath() const;

        bool write(std::uint8_t device_address,
                   const std::uint8_t* data,
                   std::size_t length) override;
        bool read(std::uint8_t device_address,
                  std::uint8_t* data,
                  std::size_t length) override;
        bool writeRead(std::uint8_t device_address,
                       const std::uint8_t* tx,
                       std::size_t tx_length,
                       std::uint8_t* rx,
                       std::size_t rx_length) override;

    private:
        std::string device_path_;
        int fd_;
    };

    MPU6050(I2cBus& bus, Address address);

    bool initialize(const Config& config);
    bool reset();
    bool wake(ClockSource clock_source);
    bool sleep();
    bool verifyIdentity();

    bool configure(const Config& config);
    bool setClockSource(ClockSource clock_source);
    bool setDigitalLowPassFilter(DigitalLowPassFilter dlpf);
    bool setGyroRange(GyroRange range);
    bool setAccelRange(AccelRange range);
    bool setSampleRateDivider(std::uint8_t divider);
    bool enableDataReadyInterrupt(bool enabled, InterruptMode mode);

    bool readAcceleration(Vector3Raw& acceleration);
    bool readGyroscope(Vector3Raw& gyroscope);
    bool readTemperature(std::int16_t& temperature);
    bool readMotionFrame(MotionFrameRaw& frame);
    bool readInterruptStatus(std::uint8_t& status);
    bool readFifoCount(std::uint16_t& count);
    bool readFifo(std::uint8_t* data, std::size_t length);

    bool readRegister(Register reg, std::uint8_t& value);
    bool writeRegister(Register reg, std::uint8_t value);
    bool readRegisters(Register start_reg, std::uint8_t* values, std::size_t count);
    bool writeRegisters(Register start_reg, const std::uint8_t* values, std::size_t count);

    Address address() const;
    std::uint8_t addressByte() const;
    std::uint8_t whoAmI() const;

    static constexpr std::uint8_t registerAddress(Register reg)
    {
        return static_cast<std::uint8_t>(reg);
    }

    static constexpr std::uint8_t addressByte(Address address)
    {
        return static_cast<std::uint8_t>(address);
    }

private:
    static constexpr std::uint8_t kClockSourceMask = 0x07;
    static constexpr std::uint8_t kSleepBit = 0x40;
    static constexpr std::uint8_t kDeviceResetBit = 0x80;
    static constexpr std::uint8_t kDlpfMask = 0x07;
    static constexpr std::uint8_t kGyroRangeShift = 3;
    static constexpr std::uint8_t kAccelRangeShift = 3;
    static constexpr std::uint8_t kDataReadyInterruptBit = 0x01;
    static constexpr std::uint8_t kInterruptConfigMask = 0xF0;

    I2cBus& bus_;
    Address address_;
    std::uint8_t who_am_i_;
};

using MPU6050Pair = std::array<MPU6050*, 2>;

} // namespace haptic_feedback

#endif // HAPTIC_FEEDBACK_MPU6050_HPP
