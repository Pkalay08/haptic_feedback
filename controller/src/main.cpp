// controller/src/main.cpp
// Complete Controller Firmware for EMG + IMU + CNN + BLE

// === INCLUDES ===
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>

// Your custom headers
#include "ADS1299.hpp"
/*#include "CNNInference.hpp"*/
#include "Communication_protocol.hpp"
#include "EMGSignalProcessor.hpp"
#include "MPU6050.hpp"
#include "UART.hpp"
// #include "Logger.hpp"          // Optional: for logging to file

// === GLOBALS ===
std::atomic<bool> running{true};

// === SIGNAL HANDLER (for graceful shutdown) ===
void signalHandler(int signal) {
    (void)signal;
    running = false;
}

// === MAIN ENTRY POINT ===
int main() {
    using namespace haptic_feedback;

    auto cfg = UART::ble60DefaultConfig();
    UART::LinuxSerialPort uart("/dev/serial0", cfg);  // or "/dev/ttyAMA0"

    if (!uart.open()) {
        std::cerr << "Failed to open UART\n";
        return 1;
    }

    BLE60 ble(uart);
    if (!ble.initialize()) {
        std::cerr << "Failed to initialize BLE60\n";
        return 1;
    }

    // --- 1. Confirm AT-command mode works ---
    std::string resp;
    if (!ble.transactCommand("at+ab config BuildVersion", resp, 128)) {
        std::cerr << "No response to BuildVersion\n";
        return 1;
    }
    std::cout << "BuildVersion: " << resp << std::endl;

    // --- 2. Make sure we are in command mode ---
    //     (If the module is already in command mode, this is a no-op;
    //      if it is stuck in bypass, it will exit.)
    if (!ble.enterCommandMode()) {
        std::cerr << "Could not enter command mode (still in bypass?)\n";
        // Not fatal — try to continue anyway
    }

    // --- 3. Configure the module ---
    ble.transactCommand("AT+AB LeAdv Enable", resp, 64);
    std::cout << "Adv: " << resp << std::endl;

    // --- 4. Enter bypass mode for binary data ---
    if (!ble.enterBypassMode()) {
        std::cerr << "Failed to enter bypass mode\n";
        return 1;
    }
    std::cout << "In bypass mode\n";

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
        std::cout << "Sent " << n << " bytes\n";
        ::sleep(1);
    }

    return 0;
}