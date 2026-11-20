#ifndef HAPTIC_FEEDBACK_UART_HPP
#define HAPTIC_FEEDBACK_UART_HPP

#include <cstddef>
#include <cstdint>
#include <string>

namespace Terminator {
    constexpr const char* BuildVersion    = "BuildVersion";
    constexpr const char* AdvOk           = "AdvOk";
    constexpr const char* BypassMode      = "BypassMode";
    constexpr const char* ConnectionUp    = "ConnectionUp";
    constexpr const char* ConnectionDown  = "ConnectionDown";
    constexpr const char* DiscoveryComplete = "DiscoveryComplete";
    constexpr const char* CommandMode     = "-CommandMode-";
    constexpr const char* BDAddress       = "BDAddress";
    constexpr const char* ResetPending    = "ResetPending";
    constexpr const char* ProfileRole     = "ProfileRole";
    constexpr const char* AttValeInfo     = "AttValeInfo";
    constexpr const char* LeWriteRspDone  = "LeWriteRspDone";
}
namespace haptic_feedback {

class UART {
public:
    enum class BaudRate : std::uint32_t {
        Rate9600 = 9600,
        Rate19200 = 19200,
        Rate38400 = 38400,
        Rate57600 = 57600,
        Rate115200 = 115200,
        Rate230400 = 230400,
        Rate460800 = 460800,
        Rate921600 = 921600,
    };

    enum class DataBits : std::uint8_t {
        Five = 5,
        Six = 6,
        Seven = 7,
        Eight = 8,
    };

    enum class Parity : std::uint8_t {
        None,
        Even,
        Odd,
    };

    enum class StopBits : std::uint8_t {
        One = 1,
        Two = 2,
    };

    enum class FlowControl : std::uint8_t {
        None,
        Software,
        Hardware,
    };

    struct Config {
        BaudRate baud_rate;
        DataBits data_bits;
        Parity parity;
        StopBits stop_bits;
        FlowControl flow_control;
        bool non_blocking;
        std::uint8_t read_timeout_deciseconds;
        std::uint8_t minimum_read_bytes;
    };

    class SerialPort {
    public:
        virtual ~SerialPort() {}
        virtual bool open() = 0;
        virtual void close() = 0;
        virtual bool isOpen() const = 0;
        virtual int read(std::uint8_t* data, std::size_t length) = 0;
        virtual int write(const std::uint8_t* data, std::size_t length) = 0;
        virtual bool flushInput() = 0;
        virtual bool flushOutput() = 0;
        virtual bool flushBoth() = 0;
        virtual bool drainOutput() = 0;
    };

    class LinuxSerialPort : public SerialPort {
    public:
        LinuxSerialPort(const std::string& device_path, const Config& config);
        ~LinuxSerialPort();

        LinuxSerialPort(const LinuxSerialPort&) = delete;
        LinuxSerialPort& operator=(const LinuxSerialPort&) = delete;

        bool open() override;
        void close() override;
        bool isOpen() const override;
        int read(std::uint8_t* data, std::size_t length) override;
        int write(const std::uint8_t* data, std::size_t length) override;
        bool flushInput() override;
        bool flushOutput() override;
        bool flushBoth() override;
        bool drainOutput() override;

        bool applyConfig(const Config& config);
        const std::string& devicePath() const;
        const Config& config() const;
        int fileDescriptor() const;

    private:
        std::string device_path_;
        Config config_;
        int fd_;
    };

    static Config defaultConfig()
    {
        Config config = {};
        config.baud_rate = BaudRate::Rate115200;
        config.data_bits = DataBits::Eight;
        config.parity = Parity::None;
        config.stop_bits = StopBits::One;
        config.flow_control = FlowControl::None;
        config.non_blocking = false;
        config.read_timeout_deciseconds = 1;
        config.minimum_read_bytes = 0;
        return config;
    }

    static Config ble60DefaultConfig()
    {
        Config config = defaultConfig();
        config.baud_rate = BaudRate::Rate115200;
        config.data_bits = DataBits::Eight;
        config.parity = Parity::None;
        config.stop_bits = StopBits::One;
        config.flow_control = FlowControl::None;
        config.non_blocking = false;
        config.read_timeout_deciseconds = 2;
        config.minimum_read_bytes = 0;
        return config;
    }
};

class BLE60 {
public:
    static constexpr const char* kCommandTerminator = "\r\n";
    static constexpr std::size_t kDefaultResponseBytes = 256;

    enum class Role : std::uint8_t {
        Peripheral,
        Central,
        Observer,
        Broadcaster,
    };

    enum class LinkState : std::uint8_t {
        Unknown,
        Advertising,
        Connected,
        Disconnected,
    };

    struct Status {
        LinkState link_state;
        std::int8_t rssi_dbm;
        bool notifications_enabled;
    };

    explicit BLE60(UART::SerialPort& serial);

    bool initialize();
    bool isOpen() const;

    int send(const std::string& cmd, const std::uint8_t* data, std::size_t length);
    int receive(std::uint8_t* data, std::size_t length);
    bool sendText(const std::string& text);

    // ---- Mode control ----
    bool enterCommandMode();          // sends A#A\$% (no CRLF), waits 2s, reads "AT-AB-CommandMode"
    bool enterBypassMode();           // sends AT+AB LeBypass and waits for "AT-AB-BypassMode"
    bool reset();                     // sends AT+AB Reset

    // ---- Raw data (bypass mode) ----
    int sendRaw(const std::uint8_t* data, std::size_t length);   // no terminator, no framing
    int sendRaw(const std::string& text) {
        return sendRaw(reinterpret_cast<const std::uint8_t*>(text.data()), text.size());
    }

    bool sendCommand(const std::string&    command);
    bool send_and_wait(const std::string&  command,
                          std::string&     response,
                          int              gap_ms = 200,
                          int              safety_timeout_ms = 30000);
    bool readResponse(std::string& out, int quiet_ms = 200, int timeout_ms = 30000);
    bool transactUntil(const std::string& command,
                          const std::string& expected_token,
                          std::string& accumulated,
                          int timeout_ms = 5000);

    bool flush();
    UART::SerialPort& serialPort();

private:
    UART::SerialPort& serial_;
};

} // namespace haptic_feedback

#endif // HAPTIC_FEEDBACK_UART_HPP
