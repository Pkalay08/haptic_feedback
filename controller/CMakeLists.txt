# ============================================================
# CMakeLists.txt — CM3 Controller Firmware
# ============================================================

# --- 1. Project declaration ---
cmake_minimum_required(VERSION 3.16)
set(CMAKE_SYSROOT /usr/arm-linux-gnueabihf)
project(haptic_controller LANGUAGES CXX)

# --- 2. C++ standard ---
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(TENSORFLOW_REPO "$ENV{HOME}/tensorflow/")

# --- 3. Source files ---
set(SOURCES
    src/main.cpp
    src/ADS1299.cpp
    #src/CNNInference.cpp
    src/Communication_protocol.cpp
    src/EMGSignalProcessor.cpp
    src/MPU6050.cpp
    src/UART.cpp
    src/Logger.cpp
)

# --- 4. Include directories ---
include_directories(
    ${CMAKE_SOURCE_DIR}/include
    ${TENSORFLOW_REPO}
)

# --- 5. Create the executable ---
add_executable(${PROJECT_NAME} ${SOURCES})

# --- 6. Compiler flags ---
target_compile_options(${PROJECT_NAME} PRIVATE
    -Wall
    -Wextra
    -O2
)

# --- 8. Threading (required for LiteRT) ---
find_package(Threads REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE
#    ${CMAKE_SOURCE_DIR}/lib/libLiteRt.so
    Threads::Threads
)