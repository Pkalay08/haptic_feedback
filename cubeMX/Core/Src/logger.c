#include "logger.h"
#include "main.h"          // For huart2 (your debug UART handle)
#include <string.h>
#include <stdio.h>         // For vsnprintf
#include <stdlib.h>        // For abs()
#include <time.h>          // For time functions

// ===== CONFIGURATION =====
#define LOG_BUFFER_SIZE 128      // Max message length
#define LOG_TIMESTAMP_ENABLED 1  // 1 = include timestamp, 0 = no timestamp
#define LOG_LEVEL_ENABLED   1    // 1 = include log level, 0 = no level

// ===== INTERNAL STATE =====
static UART_HandleTypeDef* g_huart = NULL;
static LogLevel g_minLevel = LOG_LEVEL_DEBUG;
static uint32_t g_startTimeMs = 0;  // For relative timestamps

// Forward declarations
static void logger_send_string(const char* str);
static void logger_send_char(char c);
static uint32_t logger_get_tick_ms(void);
static const char* logger_level_to_string(LogLevel level);

// ===== PUBLIC FUNCTIONS =====

void logger_init(void) {
    // Use USART2 (or whichever UART you configured for debug)
    g_huart = &huart2;
    g_startTimeMs = logger_get_tick_ms();
}

void logger_setLevel(LogLevel level) {
    g_minLevel = level;
}

void logger_log(LogLevel level, const char* format, ...) {
    // Filter by log level
    if (level < g_minLevel) return;
    if (g_huart == NULL) return;

    char buffer[LOG_BUFFER_SIZE];
    int offset = 0;

    // --- 1. Timestamp (if enabled) ---
    #if LOG_TIMESTAMP_ENABLED
    {
        uint32_t elapsedMs = logger_get_tick_ms() - g_startTimeMs;
        uint32_t seconds = elapsedMs / 1000;
        uint32_t millis = elapsedMs % 1000;
        offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset,
                          "[%04lu.%03lu] ", seconds, millis);
    }
    #endif

    // --- 2. Log level (if enabled) ---
    #if LOG_LEVEL_ENABLED
    {
        const char* levelStr = logger_level_to_string(level);
        offset += snprintf(buffer + offset, LOG_BUFFER_SIZE - offset,
                          "[%s] ", levelStr);
    }
    #endif

    // --- 3. User message (varargs) ---
    va_list args;
    va_start(args, format);
    offset += vsnprintf(buffer + offset, LOG_BUFFER_SIZE - offset, format, args);
    va_end(args);

    // Ensure null termination
    buffer[LOG_BUFFER_SIZE - 1] = '\0';

    // --- 4. Send over UART ---
    logger_send_string(buffer);
    logger_send_string("\r\n");  // Newline
}

// ===== PRIVATE HELPERS =====

static void logger_send_string(const char* str) {
    if (g_huart == NULL) return;
    CDC_Transmit_FS((uint8_t*)str, strlen(str));
}

static void logger_send_char(char c) {
    if (g_huart == NULL) return;
    HAL_UART_Transmit(g_huart, (uint8_t*)&c, 1, 100);
}

static uint32_t logger_get_tick_ms(void) {
    // Returns the current system tick (milliseconds)
    // Requires HAL_GetTick() to be available (it is, from HAL)
    return HAL_GetTick();
}

static const char* logger_level_to_string(LogLevel level) {
    switch (level) {
        case LOG_LEVEL_DEBUG: return "DBG";
        case LOG_LEVEL_INFO:  return "INF";
        case LOG_LEVEL_WARN:  return "WRN";
        case LOG_LEVEL_ERROR: return "ERR";
        case LOG_LEVEL_FATAL: return "FTL";
        default:              return "???";
    }
}