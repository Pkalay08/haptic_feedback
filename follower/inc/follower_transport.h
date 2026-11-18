#ifndef FOLLOWER_TRANSPORT_H
#define FOLLOWER_TRANSPORT_H

#include "controller_follower_protocol.h"
#include <zephyr/bluetooth/conn.h>

#ifdef __cplusplus
extern "C" {
#endif

static struct bt_conn *current_conn;

/* Must be called once, before advertising starts. Registers connection and NUS callbacks. */
int follower_transport_init(void);

/* Notify a payload back to the controller over the NUS TX characteristic.
 * Returns true if queued successfully. */
bool follower_transport_send(uint8_t type,
                             uint16_t sequence,
                             const void *payload,
                             uint16_t payload_length);

bool follower_send_haptic(const haptic_feedback_payload_t *fb,
                          uint16_t sequence);

bool follower_send_state(const follower_state_payload_t *state,
                         uint16_t sequence);


#ifdef __cplusplus
}
#endif

#endif /* FOLLOWER_TRANSPORT_H */