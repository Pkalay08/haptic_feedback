#include "controller_follower_protocol.h"

#include <string.h>

uint16_t crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = PROTO_CRC_INITIAL;
    for (size_t i = 0; i < len; ++i) {
        crc ^= (uint16_t)data[i] << 8;
        for (int b = 0; b < 8; ++b) {
            if (crc & 0x8000) {
                crc = (uint16_t)((crc << 1) ^ PROTO_CRC_POLYNOMIAL);
            } else {
                crc = (uint16_t)(crc << 1);
            }
        }
    }
    return crc;
}

size_t frame_total_size(const protocol_frame_t *frame)
{
    return offsetof(protocol_frame_t, payload)
         + frame->payload_length
         + sizeof(frame->crc);
}

uint16_t compute_frame_crc(const protocol_frame_t *frame)
{
    const uint8_t *p = (const uint8_t *)frame;

    /* header bytes after start_byte */
    uint16_t crc = crc16(p + 1, offsetof(protocol_frame_t, payload) - 1);

    /* payload bytes, continue the same CRC state */
    for (size_t i = 0; i < frame->payload_length; ++i) {
        crc ^= (uint16_t)frame->payload[i] << 8;
        for (int b = 0; b < 8; ++b) {
            crc = (crc & 0x8000) ? (uint16_t)((crc << 1) ^ PROTO_CRC_POLYNOMIAL)
                                 : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

int frame_build(protocol_frame_t *frame,
                uint8_t type,
                uint16_t sequence,
                uint32_t timestamp_ms,
                const void *payload,
                uint16_t payload_length)
{
    if (!frame || payload_length > PROTO_MAX_PAYLOAD_SIZE) {
        return 0;
    }
    memset(frame, 0, sizeof(*frame));
    frame->start_byte     = PROTO_START_BYTE;
    frame->type           = type;
    frame->sequence       = sequence;
    frame->timestamp_ms   = timestamp_ms;
    frame->payload_length = payload_length;
    if (payload_length && payload) {
        memcpy(frame->payload, payload, payload_length);
    }
    frame->crc = compute_frame_crc(frame);
    return 1;
}

int frame_verify_crc(const protocol_frame_t *frame)
{
    if (frame->start_byte != PROTO_START_BYTE) {
        return 0;
    }
    if (frame->payload_length > PROTO_MAX_PAYLOAD_SIZE) {
        return 0;
    }
    return compute_frame_crc(frame) == frame->crc;
}