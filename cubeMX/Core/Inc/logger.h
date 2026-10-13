#ifndef LOGGER_H
#define LOGGER_H

#include <stdint.h>
#include <stdarg.h>
#include "usbd_cdc_if.h"

// ===== C++ COMPATIBILITY =====
#ifdef __cplusplus
extern "C" {
#endif

// Log levels
typedef enum {
    LOG_LEVEL_DEBUG = 0,
    LOG_LEVEL_INFO  = 1,
    LOG_LEVEL_WARN  = 2,
    LOG_LEVEL_ERROR = 3,
    LOG_LEVEL_FATAL = 4
} LogLevel;

// Initialize the logger (set the UART handle)
void logger_init(void);

// Set the minimum log level (messages below this level are ignored)
void logger_setLevel(LogLevel level);

// Log a formatted message with a specific level
void logger_log(LogLevel level, const char* format, ...);

// Convenience macros
#define LOG_DEBUG(...) logger_log(LOG_LEVEL_DEBUG, __VA_ARGS__)
#define LOG_INFO(...)  logger_log(LOG_LEVEL_INFO,  __VA_ARGS__)
#define LOG_WARN(...)  logger_log(LOG_LEVEL_WARN,  __VA_ARGS__)
#define LOG_ERROR(...) logger_log(LOG_LEVEL_ERROR, __VA_ARGS__)
#define LOG_FATAL(...) logger_log(LOG_LEVEL_FATAL, __VA_ARGS__)

#ifdef __cplusplus
}
#endif

#endif // LOGGER_H