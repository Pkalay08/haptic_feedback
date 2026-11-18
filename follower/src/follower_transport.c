#include "follower_transport.h"

#include <string.h>
#include <zephyr/kernel.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gatt.h>
#include <bluetooth/services/nus.h>
#include <zephyr/bluetooth/gap.h>

static struct bt_conn *current_conn;

/* ── Send ──────────────────────────────────────────────────────────────── */

bool follower_transport_send(uint8_t type,
                             uint16_t sequence,
                             const void *payload,
                             uint16_t payload_length)
{
    if (current_conn == NULL) {
        return false;
    }
    if (payload_length > PROTO_MAX_PAYLOAD_SIZE) {
        return false;
    }

    protocol_frame_t frame;
    if (!frame_build(&frame, type, sequence,
                     (uint32_t)k_uptime_get(),
                     payload, payload_length)) {
        return false;
    }

    size_t total = frame_total_size(&frame);

    /* bt_nus_send handles CCCD subscription checks and the notify call. */
    int err = bt_nus_send(current_conn, (const uint8_t *)&frame, total);
    return err == 0;
}

bool follower_send_haptic(const haptic_feedback_payload_t *fb, uint16_t sequence)
{
    return follower_transport_send(MSG_FEEDBACK_HAPTIC, sequence, fb, sizeof(*fb));
}

bool follower_send_state(const follower_state_payload_t *state, uint16_t sequence)
{
    return follower_transport_send(MSG_FEEDBACK_STATE, sequence, state, sizeof(*state));
}
