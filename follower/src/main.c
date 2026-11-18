#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/gpio.h>
#include <zephyr/logging/log.h>
#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/gap.h>
#include <bluetooth/services/nus.h>
#include <stdio.h>
#include "i2c_devices.h"
#include "math.h"
#include "controller_follower_protocol.h"
#include "follower_transport.h"

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

/* ---------------  Prototype declarations ------- */
static void lx16a_send_packet(uint8_t id, uint8_t len, uint8_t cmd, uint8_t ahb, uint8_t alb, uint16_t time);

/* ---------------- Hardware UART ---------------- */
#define UART_DEVICE_NODE DT_CHOSEN(zephyr_console)
static const struct device *uart_dev = DEVICE_DT_GET(DT_ALIAS(servo_uart));

#define UART_BUF_SIZE 256
static uint8_t uart_rx_buf[UART_BUF_SIZE];

static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data);
#define NUM_SERVOS    5

/* ---------------- UART callback ---------------- */
static void uart_cb(const struct device *dev, struct uart_event *evt, void *user_data)
{
    switch (evt->type) {
    case UART_RX_RDY:
        if(bt_is_ready()){
            LOG_INF("UART RX %d bytes", evt->data.rx.len);
            bt_nus_send(NULL, &evt->data.rx.buf[evt->data.rx.offset], evt->data.rx.len);
        }
        break;

    case UART_RX_BUF_REQUEST:
        uart_rx_buf_rsp(dev, uart_rx_buf, sizeof(uart_rx_buf));
        break;

    case UART_RX_BUF_RELEASED:
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

/* ---------------- QT1070 CHANGE PIN ---------------- */
static const struct gpio_dt_spec qt_change = GPIO_DT_SPEC_GET(DT_ALIAS(qt1070_change), gpios);

static struct gpio_callback qt_cb;

static void qt_change_handler(const struct device *port,
                              struct gpio_callback *cb, uint32_t pins)
{
    uint8_t detection = 0;
    qt1070_read_status(&detection, NULL);
    LOG_INF("Touch interrupt: 0x%02X", detection);
}

/* ---------------- BLE NUS callbacks ---------------- */
static void nus_receive_cb(struct bt_conn *conn,
                            const uint8_t *data,
                            uint16_t len)
{
    ARG_UNUSED(conn);
    if (len < offsetof(protocol_frame_t, payload)) {
        printk("NUS RX: short frame (%u)\n", len);
        return;
    }

    protocol_frame_t frame;
    memset(&frame, 0, sizeof(frame));

    /* Copy the header */
    memcpy(&frame, data, offsetof(protocol_frame_t, payload));

    if (frame.start_byte != PROTO_START_BYTE) {
        printk("NUS RX: bad start byte 0x%02X\n", frame.start_byte);
        return;
    }
    if (frame.payload_length > PROTO_MAX_PAYLOAD_SIZE) {
        printk("NUS RX: payload too long (%u)\n", frame.payload_length);
        return;
    }

    size_t expected = frame_total_size(&frame);
    if (len < expected) {
        printk("NUS RX: truncated frame (got %u, need %u)\n",
               len, (unsigned)expected);
        return;
    }

    /* Copy payload and CRC */
    memcpy(frame.payload, data + offsetof(protocol_frame_t, payload),
           frame.payload_length);
    memcpy(&frame.crc,
           data + offsetof(protocol_frame_t, payload) + frame.payload_length,
           sizeof(frame.crc));

    if (!frame_verify_crc(&frame)) {
        printk("NUS RX: CRC mismatch\n");
        return;
    }

    if (frame.type != MSG_COMMAND_SERVO) {
        printk("NUS RX: unexpected type 0x%02X\n", frame.type);
        return;
    }
    if (frame.payload_length < sizeof(servo_command_payload_t)) {
        printk("NUS RX: short servo payload\n");
        return;
    }

    for (int i = 0; i < NUM_SERVOS; i++){
        printk("Sending 0x%02x", frame.payload[i]);
        lx16a_send_packet(i+1, 7, 1, frame.payload[i*2], frame.payload[(i*2)+1], 1000);
    }
}



static struct bt_nus_cb nus_cbs = {
    .received = nus_receive_cb,
};

/* ---------------- Connection callbacks ---------------- */
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
    /* Uses the raw-byte UUID macro */
    BT_DATA_BYTES(BT_DATA_UUID128_ALL, BT_UUID_NUS_VAL),
};

/* Explicit adv parameters */
static const struct bt_le_adv_param adv_param =
    BT_LE_ADV_PARAM_INIT(BT_LE_ADV_OPT_CONN,
                         BT_GAP_ADV_FAST_INT_MIN_2,
                         BT_GAP_ADV_FAST_INT_MAX_2,
                         NULL);

/* ---------------- BT ready callback ---------------- */
static void bt_ready(int err)
{
    if (err) {
        LOG_ERR("Bluetooth init failed (err %d)", err);
        return;
    }
    LOG_INF("Bluetooth initialized");

    err = bt_nus_init(&nus_cbs);
    if (err) {
        LOG_ERR("NUS init failed (err %d)", err);
        return;
    }

    err = bt_le_adv_start(&adv_param, ad, ARRAY_SIZE(ad),
                          sd, ARRAY_SIZE(sd));
    if (err) {
        LOG_ERR("Advertising failed to start (err %d)", err);
        return;
    }
    LOG_INF("Advertising started as \"%s\"", CONFIG_BT_DEVICE_NAME);
}

static void sensor_thread(void *a, void *b, void *c)
{
    LOG_INF("Sensor Thread started");
    uint8_t detection = 0;
    uint16_t angle = 0;

    k_sleep(K_MSEC(200));
    for (;;) {
        if (qt1070_read_status(&detection, NULL) == 0 && detection) {
            LOG_INF("Touch: 0x%02X", detection);
        }
        if (as5600_read_angle(AS5600_I2C_ADDR, &angle) == 0) {
            /* use angle */
        }
        k_sleep(K_MSEC(20));
    }
}

K_THREAD_STACK_DEFINE(sensor_stack, 4096);
static struct k_thread sensor_thread_data;

/* ----------- SERVO PACKET FUNCTION ------------- */
static void lx16a_send_packet(uint8_t id, uint8_t len, uint8_t cmd, uint8_t ahb, uint8_t alb, uint16_t time) {
    // Packet format: 0x55 0x55 [ID] [LEN=7] CMD [PARAM_1] ... [PARAM_N]  [CHECKSUM]
    uint8_t servo_packet[10];

    servo_packet[0] = 0x55;
    servo_packet[1] = 0x55;
    servo_packet[2] = id;
    servo_packet[3] = len;
    servo_packet[4] = cmd;
    servo_packet[5] = alb;
    servo_packet[6] = ahb;
    servo_packet[7] = time & 0xFF;
    servo_packet[8] = (time >> 8) & 0xFF;
    servo_packet[9] = 0x00;

    // Calculate checksum (summation of bytes 2-8)
    uint8_t checksum = 0;
    for (int i = 2; i < 9; i++) {
        checksum += servo_packet[i];
    }
    servo_packet[9] = ~checksum; 

    printk("packet contents: %02X %02X %02X %02X %02X %02X %02X %02X %02X %02X\n", servo_packet[0], servo_packet[1],
                          servo_packet[2], servo_packet[3], servo_packet[4], servo_packet[5], servo_packet[6], servo_packet[7], servo_packet[8],
                        servo_packet[9]);
    
    /* Send over UART */
    for (int i = 0; i < 10; i++) {
        uart_poll_out(uart_dev, servo_packet[i]);
    }

    /* Inter-packet delay */
    k_msleep(5);

    return;

}

/**
  * @brief Home LX-16A servos upon startup 
  * @param channel Channel number (0-4 for servos 1-5)
  * @return 12-bit angle value (0-4095)
  */
void lx16a_home(void) {
  // home servos when loop starts
    printk("Homing servos...\n");
    lx16a_send_packet(254, 7, 1, 0x00, 0x00, 1000);
    k_msleep(1000);

    lx16a_send_packet(254, 7, 1, 0x00, 0x64, 1000);
    k_msleep(1000);

    lx16a_send_packet(254, 7, 1, 0x00, 0x00, 2000);
    k_msleep(1000);

    return;
  
}



/* ---------------- main() ---------------- */
int main(void)
{
    printk("PRINTK: main() reached\n");

    int err;

    printk("STEP 1: i2c readiness check\n");
    /* 1. Check peripherals are ready */
    if (!device_is_ready(DEVICE_DT_GET(DT_NODELABEL(i2c0)))) {
        LOG_ERR("I2C not ready");
        return -1;
    }
    
    printk("STEP 2: uart readiness check\n");
    if (!device_is_ready(DEVICE_DT_GET(DT_ALIAS(servo_uart)))) {
        LOG_ERR("UART not ready");
        return -1;
    }

    /* 2. Initialize I2C devices */
    printk("STEP 3: qt1070_init\n");
    err = qt1070_init();
    if (err) {
        LOG_ERR("QT1070 init failed (%d)", err);
        /* keep going — other devices may work */
    }
    printk("STEP 3 done: err=%d\n", err);

    printk("STEP 4: as5600_init\n");
    err = as5600_init(AS5600_I2C_ADDR);
    if (err) {
        LOG_WRN("AS5600 not found (%d)", err);
    }
    printk("STEP 4 done: err=%d\n", err);

    printk("STEP 5: uart_callback_set\n");
    uart_callback_set(DEVICE_DT_GET(DT_ALIAS(servo_uart)), uart_cb, NULL);

    printk("STEP 6: uart_rx_enable\n");
    uart_irq_rx_enable(DEVICE_DT_GET(DT_ALIAS(servo_uart)));

    printk("STEP 7: bt_enable\n");
    /* 4. Bring up Bluetooth */
    err = bt_enable(bt_ready);
    if (err) {
        LOG_ERR("bt_enable failed (err %d)", err);
        return -1;
    }
    printk("STEP 7 done: err=%d\n", err);

    LOG_INF("Ready. UART<->BLE bridge running.");

    printk("STEP 8: home servos\n");
    lx16a_home();
    k_sleep(K_MSEC(20));

    printk("STEP 9: create sensor thread\n");

    /* sensor thread*/
    k_thread_create(&sensor_thread_data, sensor_stack,
        K_THREAD_STACK_SIZEOF(sensor_stack),
        sensor_thread, NULL, NULL, NULL,
        5, 0, K_FOREVER);

    for (;;) {
        k_sleep(K_FOREVER);
    }
}