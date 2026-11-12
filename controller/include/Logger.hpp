#ifndef LOG_HPP
#define LOG_HPP

#include <string>

namespace haptic_feedback {

// Initialize the log file once at startup.
void logInit(const std::string& path);

// Write a line to the log. Timestamps each entry.
void log(const std::string& message);

// Same, but adds "ERROR" prefix.
void logError(const std::string& message);

} // namespace haptic_feedback

#endif