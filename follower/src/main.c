#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/services/nus.h>

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ---------------- Hardware UART ---------------- */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)   /* or a custom uartN */
static const struct device *uart_dev = DEVICE_DT_GET(UART_DEVICE_NODE);

#define UART_BUF_SIZE 256
static uint8_t uart_rx_buf[UART_BUF_SIZE];

/* Semaphore to signal "BLE stack ready to send" */
static K_SEM_DEFINE(nus_tx_sem, 0, 1);

/* Forward declarations */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);
static void bt_ready(int err);
static void nus_send_cb(struct bt_conn *conn, const void *data, uint16_t len);

/* ---------------- UART callback ---------------- */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        /* Data arrived from the physical UART — forward to BLE */
        LOG_INF("UART RX %d bytes", evt->data.rx.len);
        bt_nus_send(NULL, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
        break;

    case UART_RX_BUF_REQUEST:
        /* Driver asks for the next buffer (double-buffering) */
        uart_rx_buf_request(dev);
        break;

    case UART_RX_BUF_RELEASED:
        /* Driver is done with a buffer; give it back */
        uart_rx_buf_rsp(dev, uart_rx_buf, sizeof(uart_rx_buf));
        break;

    case UART_TX_DONE:
        break;

    case UART_TX_ABORTED:
        LOG_WRN("UART TX aborted");
        break;

    default:
        break;
    }
}

/* ---------------- BLE NUS callbacks ---------------- */
/* Called when the phone writes to the NUS RX characteristic */
static void nus_receive_cb(struct bt_conn *conn, const uint8_t *const data, uint16_t len)
{
    LOG_INF("BLE RX %d bytes -> UART", len);
    for (uint16_t i = 0; i < len; i++) {
        uart_poll_out(uart_dev, data[i]);
    }
}

/* Called when NUS TX notification is sent (or fails) */
static void nus_send_cb(struct bt_conn *conn, const void *data, uint16_t len)
{
    ARG_UNUSED(conn);
    ARG_UNUSED(data);
    ARG_UNUSED(len);
}

static struct bt_nus_cb nus_cbs = {
    .received = nus_receive_cb,
    .sent     = nus_send_cb,
};

/* ---------------- Bluetooth connection callbacks ---------------- */
static void connected(struct bt_conn *conn, uint8_t err)
{
    if (err) {
        LOG_ERR("Connection failed (err %u)", err);
        return;
    }
    LOG_INF("BLE connected");
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
    LOG_INF("BLE disconnected (reason %u)", reason);
}

BT_CONN_CB_DEFINE(conn_callbacks) = {
    .connected    = connected,
    .disconnected = disconnected,
};

/* ---------------- Advertising data ---------------- */
static const struct bt_data ad[] = {
    BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
    BT_DATA(BT_DATA_NAME_COMPLETE, CONFIG_BT_DEVICE_NAME,
            sizeof(CONFIG_BT_DEVICE_NAME) - 1),
};

static const struct bt_data sd[] = {
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* ---------------- BT ready callback ---------------- */
static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    /* Register our NUS callbacks */
    err = bt_nus_init(&nus_cbs);
    if (err) {
        LOG_ERR("NUS init failed (err %d)", err);
        return;
    }

    /* Start advertising */
    err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started as \"%s\"", CONFIG_BT_DEVICE_NAME);
}

/* ---------------- main() ---------------- */
int main(void)
{
    int err;

    /* 1. Set up UART */
    if (!device_is_ready(uart_dev)) {
        LOG_ERR("UART device not ready");
        return -1;
    }
    uart_callback_set(uart_dev, uart_cb, NULL);
    uart_rx_enable(uart_dev, uart_rx_buf, sizeof(uart_rx_buf), 100);  /* 100us timeout */

    /* 2. Bring up Bluetooth */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return -1;
    }

    LOG_INF("Ready. UART<->BLE bridge running.");

    /* 3. Idle loop — everything happens in callbacks */
    for (;;) {
        k_sleep(K_FOREVER);
    }
}