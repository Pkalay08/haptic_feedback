# ============================================================
# Toolchain file for cross-compiling to the Raspberry Pi CM3
# ============================================================

set(CMAKE_SYSTEM_NAME Linux)
set(CMAKE_SYSTEM_PROCESSOR arm)

# --- Compilers ---
set(CMAKE_C_COMPILER   arm-linux-gnueabihf-gcc)
set(CMAKE_CXX_COMPILER arm-linux-gnueabihf-g++)

# --- Sysroot (where the target libraries live) ---
set(CMAKE_SYSROOT /usr/arm-linux-gnueabihf)

# --- Search paths ---
set(CMAKE_FIND_ROOT_PATH /usr/arm-linux-gnueabihf)
set(CMAKE_FIND_ROOT_PATH_MODE_PROGRAM NEVER)
set(CMAKE_FIND_ROOT_PATH_MODE_LIBRARY ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_INCLUDE ONLY)
set(CMAKE_FIND_ROOT_PATH_MODE_PACKAGE ONLY)