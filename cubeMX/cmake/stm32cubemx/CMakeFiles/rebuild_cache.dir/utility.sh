set -e

cd /c/Users/PK/Posidon/STUFF/haptic_feedback/cubeMX/cmake/stm32cubemx
/usr/bin/cmake.exe --regenerate-during-build -S$(CMAKE_SOURCE_DIR) -B$(CMAKE_BINARY_DIR)
