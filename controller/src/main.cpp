// controller/src/main.cpp
// Complete Controller Firmware for EMG + IMU + CNN + BLE

// === INCLUDES ===
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>
#include <iostream>
#include <thread>
#include <string>
#include <vector>
#include <cmath>
#include <numbers>
#include <stdint.h>
#include <stddef.h>
#include <stdio.h>

// Your custom headers
#include "ADS1299.hpp"
/*#include "CNNInference.hpp"*/
#include "controller_follower_protocol.h"
#include "EMGSignalProcessor.hpp"
#include "MPU6050.hpp"
#include "UART.hpp"
#include "Logger.hpp"        

#define NUM_SERVOS  5
#define MAX_PAYLOAD 128
#define HEX_BUF_SIZE (MAX_PAYLOAD * 2 + 1)

std::string nRF_bd_adr = "cab7e153689c";
std::string nRF_handle = "0011";

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

std::vector<std::uint16_t> sin_gen(std::uint8_t n,
                                float t,
                                float phase = static_cast<float>(std::numbers::pi / 4.0)){

    std::vector<std::uint16_t> out;
    out.reserve(n);

    for (std::uint8_t i = 0; i < n; ++i) {
        float s = std::sin(t + phase * static_cast<float>(i));   // in [-1, 1]
        int   a = 500 + static_cast<int>(450.0f * s);            
        if (a < 0)    a = 0;
        if (a > 1000) a = 1000;
        out.push_back(static_cast<std::uint16_t>(a));
        }   
    return out;
}

/* Convert `len` bytes at `data` into `2*len` hex characters + NUL.
 * `out` must be at least (2*len + 1) bytes.
 * Returns out on success, NULL if the buffer is too small. */
char *bytes_to_hex(const uint8_t *data, size_t len, char *out, size_t out_size)
{
    if (!data || !out || out_size < (2 * len + 1)) return NULL;

    static const char hex[] = "0123456789ABCDEF";

    for (size_t i = 0; i < len; i++) {
        out[i * 2]     = hex[(data[i] >> 4) & 0x0F];
        out[i * 2 + 1] = hex[ data[i]       & 0x0F];
    }
    out[len * 2] = '\0';
    return out;
}

/* Format the length as 2 ASCII hex chars (no prefix, no spaces).
 * `out` must be at least 3 bytes. */
char *len_to_hex4(uint16_t len, char *out, size_t out_size)
{
    if (!out || out_size < 5) return NULL;
    static const char hex[] = "0123456789ABCDEF";
    out[0] = hex[(len >> 12) & 0x0F];
    out[1] = hex[(len >>  8) & 0x0F];
    out[2] = hex[(len >>  4) & 0x0F];
    out[3] = hex[ len        & 0x0F];
    out[4] = '\0';
    return out;
}


// === MAIN ENTRY POINT ===
int main() {
    using namespace haptic_feedback;
    using ms = std::chrono::milliseconds;
    using namespace std;
    
    std::string resp;
    std::string adr;

    logInit("/home/controller/ble.log");

    auto cfg = UART::ble60DefaultConfig();
    UART::LinuxSerialPort uart("/dev/serial0", cfg);

    if (!uart.open()) {
        logError("Failed to open UART");
        return 1;
    }
    std::this_thread::sleep_for(ms(5));

    BLE60 ble(uart);
    if (!ble.initialize()) {
        logError("Failed to initialize BLE60");
        return 1;
    }
    std::this_thread::sleep_for(ms(2000));

    ble.send_and_wait("AT+AB Reset", resp);
    log("reset response: \n" + resp);
    std::this_thread::sleep_for(ms(5));

    // --- Enter Command mode ---
    if (!ble.enterCommandMode()) {
        logError("Could not enter command mode");
        // Not fatal — try to continue anyway
    }
    ble.flush();
    std::this_thread::sleep_for(ms(5));


    // reset role to central for connecting
    ble.send_and_wait("at+ab config ProfileRole=c", resp);
    if (resp.empty()){
        return -1;
    };
    std::this_thread::sleep_for(ms(5));

    ble.send_and_wait("at+ab config ConnectMode=1", resp);
    if(resp.empty()){
        log("Could not set ConnectMode");
    }
    std::this_thread::sleep_for(ms(5));
    log("ConnectMode: \n" + resp);

    ble.send_and_wait("at+ab config TxPower=7", resp);
    if(resp.empty()){
        log("Could not set TxPower");
    }
    std::this_thread::sleep_for(ms(5));
    log("TxMode: \n" + resp);

    // Connect to scanned device
    ble.send_and_wait("AT+AB LeConnect " + nRF_bd_adr + " rand", resp);
    if (resp.empty()){
        logError("Could not connect to device");
        return -1;
    };
    std::this_thread::sleep_for(ms(5));
    logError("Connection: \n" + resp);
    
    ble.send_and_wait("AT+AB LeReadByHandle 000d", resp);
    if(resp.empty()){
        logError("Sending data did not work as intended");
    }
    log("BLE servo bytes: \n" + resp);
    std::this_thread::sleep_for(ms(5));


    float t = 0.0f;
    const float dt = 0.1f;
    uint16_t sequence = 0;

    while(running){

        std::vector<uint16_t> angles = sin_gen(5, t);
        t += dt;

        for (uint8_t i = 0; i < angles.size(); i++){
            log("angle: " + std::to_string(angles[i]));
        }

        protocol_frame_t frame = {};
        servo_command_payload_t cmd = {};

        cmd.num_servos = NUM_SERVOS;
        cmd.duration_ms = 1000;

        for (int i = 0; i < NUM_SERVOS; i++){
            cmd.positions[i] = angles[i];
        }
        std::this_thread::sleep_for(ms(5));

        frame_build(&frame, MSG_COMMAND_SERVO, sequence++, &cmd, sizeof(cmd));
        std::this_thread::sleep_for(ms(5));

        size_t total = frame_total_size(&frame);

        if (frame.payload_length > PROTO_MAX_PAYLOAD_SIZE) {
            logError("payload too large");
            return 1;
        }  
        
        char hex_payload[HEX_BUF_SIZE];
        char hex_len[5];

        bytes_to_hex(reinterpret_cast<const uint8_t*>(&frame), total, hex_payload, sizeof(hex_payload));
        if (sizeof(hex_payload) == 0) {
            log("Byte payload conversion failed");
            return false;
        }
        log(std::string("converted: ") + hex_payload);
        std::this_thread::sleep_for(ms(5));

        len_to_hex4((uint16_t)total, hex_len, sizeof(hex_len));
        if (sizeof(hex_len) == 0) {
            log("Byte length conversion failed");
            return false;
        }
        log(std::string("length: ") + hex_len);
        std::this_thread::sleep_for(ms(5));
    
        char command[300];
        int n = snprintf(command, sizeof(command),
                        "AT+AB LeWrite %s %s", hex_len, hex_payload);
        if (n < 0 || (size_t)n >= sizeof(command)) {
            logError("command truncated");
            return 1;
        }
        log(std::string("command: ") + command);

        std::string cmd_str = "AT+AB LeWrite " + nRF_handle + " " + hex_len + " " + hex_payload;
        ble.send_and_wait(cmd_str, resp);
        if(resp.empty()){
            logError("Sending data did not work as intended");
        }
        log("BLE servo bytes: \n" + resp);
        std::this_thread::sleep_for(ms(5));

    }

    return 0;
}