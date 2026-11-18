#ifndef CONTROLLER_FOLLOWER_PROTOCOL_H
#define CONTROLLER_FOLLOWER_PROTOCOL_H

#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/* ── Constants ──────────────────────────────────────────────────────────── */

#define PROTO_START_BYTE        0xAA
#define PROTO_CRC_INITIAL       0xFFFF
#define PROTO_CRC_POLYNOMIAL    0x1021
#define PROTO_MAX_PAYLOAD_SIZE  128
#define PROTO_MAX_SERVOS        12

/* ── Message types ──────────────────────────────────────────────────────── */

typedef enum {
    MSG_COMMAND_SERVO   = 0x01,   /* controller → follower */
    MSG_FEEDBACK_HAPTIC = 0x02,   /* follower → controller */
    MSG_FEEDBACK_STATE  = 0x03    /* follower → controller */
} message_type_t;

/* ── Packed payloads ────────────────────────────────────────────────────── */

#pragma pack(push, 1)

typedef struct {
    uint8_t  num_servos;
    uint16_t positions[PROTO_MAX_SERVOS];
    uint16_t duration_ms;
    uint8_t  flags;
} servo_command_payload_t;

typedef struct {
    float    force_magnitude;
    float    frequency_hz;
    uint16_t duration_ms;
    uint8_t  flags;
} haptic_feedback_payload_t;

typedef struct {
    float    battery_voltage;
    float    temperature_celsius;
    uint16_t error_code;
    float    current_positions[PROTO_MAX_SERVOS];
    float    velocities[PROTO_MAX_SERVOS];
} follower_state_payload_t;

typedef struct {
    uint8_t  start_byte;
    uint8_t  type;
    uint16_t sequence;
    uint32_t timestamp_ms;
    uint16_t payload_length;
    uint8_t  payload[PROTO_MAX_PAYLOAD_SIZE];
    uint16_t crc;
} protocol_frame_t;

#pragma pack(pop)

/* ── Compile-time size checks ───────────────────────────────────────────── */

#if defined(__STDC_VERSION__) && __STDC_VERSION__ >= 201112L
_Static_assert(sizeof(servo_command_payload_t)   <= PROTO_MAX_PAYLOAD_SIZE,
               "servo_command_payload_t too large");
_Static_assert(sizeof(haptic_feedback_payload_t) <= PROTO_MAX_PAYLOAD_SIZE,
               "haptic_feedback_payload_t too large");
_Static_assert(sizeof(follower_state_payload_t)  <= PROTO_MAX_PAYLOAD_SIZE,
               "follower_state_payload_t too large");
#endif

/* ── CRC helpers (platform-independent) ─────────────────────────────────── */

uint16_t crc16(const uint8_t *data, size_t len);
uint16_t compute_frame_crc(const protocol_frame_t *frame);

/* ── Frame helpers ──────────────────────────────────────────────────────── */

/* Returns the total number of bytes a frame occupies on the wire. */
size_t frame_total_size(const protocol_frame_t *frame);

/* Fill header + copy payload + compute CRC.
 * Returns true on success. */
int frame_build(protocol_frame_t *frame,
                uint8_t type,
                uint16_t sequence,
                uint32_t timestamp_ms,
                const void *payload,
                uint16_t payload_length);

/* Validate CRC. Returns true if valid. */
int frame_verify_crc(const protocol_frame_t *frame);

#ifdef __cplusplus
}
#endif

#endif /* CONTROLLER_FOLLOWER_PROTOCOL_H */