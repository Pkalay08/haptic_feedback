// controller/src/main.cpp
// Complete Controller Firmware for EMG + IMU + CNN + BLE

// === INCLUDES ===
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>

// Your custom headers
#include "ADS1299.hpp"
/*#include "CNNInference.hpp"*/
#include "Communication_protocol.hpp"
#include "EMGSignalProcessor.hpp"
#include "MPU6050.hpp"
#include "UART.hpp"
#include "Logger.hpp"        

std::string nRF_bd_adr = "cab7e153689c";

// === GLOBALS ===
std::atomic<bool> running{true};

// === SIGNAL HANDLER (for graceful shutdown) ===
void signalHandler(int signal) {
    (void)signal;
    running = false;
}

std::string extract_bd_address(const std::string& discovery_log) {
    // Find the line that contains "Name=Follower"
    std::size_t name_pos = discovery_log.find("Follower");
    if (name_pos == std::string::npos) return "";

    // Walk backwards to find the start of the line
    std::size_t line_start = discovery_log.rfind('\n', name_pos);
    if (line_start == std::string::npos) line_start = 0;
    else ++line_start;

    // Line format: AT-AB Discovered XX:XX:XX:XX:XX:XX RSSI=... Name=Follower
    const std::string prefix = "AT-AB Discovered ";
    std::size_t addr_pos = discovery_log.find(prefix, line_start);
    if (addr_pos == std::string::npos) return "";

    addr_pos += prefix.size();
    // Address is 17 chars: XX:XX:XX:XX:XX:XX
    return discovery_log.substr(addr_pos, 17);
}

// === MAIN ENTRY POINT ===
int main() {
    using namespace haptic_feedback;
    using ms = std::chrono::milliseconds;
    
    std::string resp;
    std::string adr;

    logInit("/home/controller/run_log.log");

    auto cfg = UART::ble60DefaultConfig();
    UART::LinuxSerialPort uart("/dev/serial0", cfg);

    if (!uart.open()) {
        logError("Failed to open UART");
        return 1;
    }

    BLE60 ble(uart);
    if (!ble.initialize()) {
        logError("Failed to initialize BLE60");
        return 1;
    }

    ble.send_and_wait("AT+AB Reset", Terminator::ResetPending, resp);
    logError("response: " + resp);
    std::this_thread::sleep_for(ms(1500));

    // --- Enter Command mode ---
    if (!ble.enterCommandMode()) {
        logError("Could not enter command mode (still in bypass?)");
        // Not fatal — try to continue anyway
    }
    logError("Entered Command mode");
    std::this_thread::sleep_for(ms(5));
    ble.flush();

    // reset role to central for connecting
    ble.send_and_wait("at+ab config ProfileRole=c", Terminator::ProfileRole, resp);
    if (resp.empty()){
        return -1;
    };
    std::this_thread::sleep_for(ms(5));

    ble.send_and_wait("at+ab config ProfileRole", Terminator::ProfileRole, resp);
    if (resp.empty()){
        return -1;
    };
    logError("response: " + resp);
    std::this_thread::sleep_for(ms(5));


    // Connect to scanned device
    bool ok = ble.send_and_wait("AT+AB LeConnect " + nRF_bd_adr + " rand", Terminator::ConnectionUp, resp);
    if (resp.empty()){
        logError("Could not connect to device");
        return -1;
    };
    logError("Connection: " + resp);
    std::this_thread::sleep_for(ms(5));
        

    // --- 5. Send a known test frame ---
    // 5 servo angles as uint16_t little-endian + 0xAA start byte
    std::uint8_t frame[11] = {0xAA,
        0x00, 0x64,   // servo1 = 100
        0x00, 0x64,   // servo2 = 100
        0x00, 0x64,   // servo3 = 100
        0x00, 0x64,   // servo4 = 100
        0x00, 0x64};  // servo5 = 100

    for (int i = 0; i < 20; ++i) {
        int n = ble.sendRaw(frame, sizeof(frame));
        std::string msg = std::to_string(n);
        logError("Sent " + msg + " bytes");
        std::this_thread::sleep_for(ms(100));
    }

    return 0;
}