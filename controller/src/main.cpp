// controller/src/main.cpp
// Complete Controller Firmware for EMG + IMU + CNN + BLE

// === INCLUDES ===
#include <chrono>
#include <thread>
#include <atomic>
#include <csignal>

// Your custom headers
#include "ADS1299.hpp"
#include "CNNInference.hpp"
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
int main(int argc, char* argv[]) {
    // Set up signal handling (Ctrl+C to stop)
    std::signal(SIGINT, signalHandler);
    std::signal(SIGTERM, signalHandler);

    // --- 1. INSTANTIATE HARDWARE DRIVERS ---
    // **FILL IN THE CONSTRUCTOR PARAMETERS FOR YOUR SETUP**
    
    // Example: ADS1299 on /dev/spidev0.0 at 4MHz
    ADS1299 ads1299("/dev/spidev0.0", 4000000);
    
    // Example: MPU6050 on I2C bus 1, address 0x68 (upper arm)
    MPU6050 imuUpper("/dev/i2c-1", 0x68);
    
    // Example: MPU6050 on I2C bus 1, address 0x69 (lower arm)
    MPU6050 imuLower("/dev/i2c-1", 0x69);  // Uncomment if using two IMUs
    
    // Example: BLE over UART (BLE60) on /dev/ttyS0 at 115200 baud
    UART bleUart("/dev/ttyS0", 115200);
    
    // --- 2. INSTANTIATE PROCESSING PIPELINE ---
    // EMG Signal Processor (sampling rate = 250 Hz)
    EMGSignalProcessor emgProcessor(250.0f);
    
    // CNN Inference (quantized TFLite model)
    EMGIMUInference cnn("models/emg_model_quantized.tflite");
    
    // Communication Protocol (packet formatting)
    SerialTransport protocol;
    
    // --- 3. INSTANTIATE LOGGER (optional) ---
    // Logger logger("telemetry.csv");  // Uncomment for file logging

    // --- 4. INITIALIZE HARDWARE ---
    std::cout << "Initializing ADS1299..." << std::endl;
    if (!ads1299.init()) {
        std::cerr << "ADS1299 initialization failed!" << std::endl;
        return -1;
    }
    
    std::cout << "Initializing IMU..." << std::endl;
    if (!imuUpper.init()) {
        std::cerr << "IMU initialization failed!" << std::endl;
        return -1;
    }
    
    std::cout << "Initializing BLE UART..." << std::endl;
    if (!bleUart.init()) {
        std::cerr << "BLE UART initialization failed!" << std::endl;
        return -1;
    }
    
    std::cout << "Loading CNN model..." << std::endl;
    if (!cnn.loadModel()) {
        std::cerr << "CNN model loading failed!" << std::endl;
        return -1;
    }

    // --- 5. MAIN CONTROL LOOP (10 Hz) ---
    std::cout << "Starting main control loop..." << std::endl;
    const int loopRateHz = 10;  // 10 Hz = 100 ms per cycle
    const std::chrono::milliseconds loopDuration(1000 / loopRateHz);
    
    int gripClass = 0;
    float confidence = 0.0f;
    int servoPosition = 0;
    
    while (running) {
        auto loopStart = std::chrono::steady_clock::now();

        // --- Step 1: Read EMG Data ---
        int16_t emgSample = ads1299.readChannel(1);  // Read from ADS1299 Channel 1
        
        // --- Step 2: Read IMU Data (optional gating) ---
        float accelX = imuUpper.readAccelX();
        float accelY = imuUpper.readAccelY();
        float accelZ = imuUpper.readAccelZ();
        
        // --- Step 3: Process EMG Signal ---
        float envelope = emgProcessor.processSample(emgSample);
        
        // --- Step 4: Run CNN Inference (every 100 ms) ---
        // Collect a window of EMG samples (e.g., 25 samples at 250 SPS = 100 ms)
        float emgWindow[25];
        // This is a placeholder—your EMGSignalProcessor class should provide a getWindow() method
        // emgProcessor.getWindow(emgWindow, 25);
        // gripClass = cnn.predict(emgWindow, 25);
        // confidence = cnn.getConfidence();
        
        // For now, use a dummy value (replace with actual inference)
        gripClass = 0;  // Replace with cnn.predict(emgWindow)
        confidence = 0.95f;
        
        // --- Step 5: Map Grip Class to Servo Position ---
        servoPosition = protocol.mapGripToPosition(gripClass);
        
        // --- Step 6: Send Servo Position over BLE ---
        uint8_t blePacket[4];
        int packetLen = protocol.encodePosition(servoPosition, blePacket);
        bleUart.send(blePacket, packetLen);
        
        // --- Step 7: Send Telemetry (optional) ---
        // logger.log(gripClass, confidence, servoPosition, accelX, accelY, accelZ);
        
        // --- Step 8: Print Debug Information ---
        std::cout << "Grip: " << gripClass 
                  << " | Conf: " << confidence 
                  << " | Pos: " << servoPosition 
                  << std::endl;

        // --- Step 9: Wait for next cycle (non-blocking) ---
        auto loopEnd = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(loopEnd - loopStart);
        
        if (elapsed < loopDuration) {
            std::this_thread::sleep_for(loopDuration - elapsed);
        }
    }

    std::cout << "Shutting down gracefully..." << std::endl;
    
    // --- 6. CLEANUP ---
    // (Destructors will handle cleanup)
    
    return 0;
}