#include "Logger.hpp"

#include <fstream>
#include <mutex>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <sstream>

namespace haptic_feedback {

namespace {
std::ofstream g_file;
std::mutex    g_mutex;

std::string timestamp() {
    auto now = std::chrono::system_clock::now();
    auto t   = std::chrono::system_clock::to_time_t(now);
    std::ostringstream ss;
    ss << std::put_time(std::localtime(&t), "%Y-%m-%d %H:%M:%S");
    return ss.str();
}
} // namespace

void logInit(const std::string& path) {
    std::lock_guard<std::mutex> lock(g_mutex);
    g_file.open(path, std::ios::out | std::ios::app);
    if (!g_file.is_open()) {
        // stderr is the only fallback
        std::fprintf(stderr, "logInit: failed to open %s\n", path.c_str());
    }
}

void log(const std::string& message) {
    std::lock_guard<std::mutex> lock(g_mutex);
    if (!g_file.is_open()) return;
    g_file << "[" << timestamp() << "] " << message << "\n";
    g_file.flush();
}

void logError(const std::string& message) {
    log("ERROR: " + message);
}

} // namespace haptic_feedback