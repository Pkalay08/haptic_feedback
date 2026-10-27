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

LOG_MODULE_REGISTER(main, LOG_LEVEL_INF);

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
    LOG_INF("BLE RX %d bytes -> UART\n", len);
    for (uint16_t i = 0; i < len; i++) {
        printk("%02X ", data[i] & 0xFF);
        uart_poll_out(uart_dev, data[i] & 0xFF);
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
static void lx16a_send_packet(uint8_t id, uint8_t len, uint8_t cmd, uint16_t angle, uint16_t time) {
    // Packet format: 0x55 0x55 [ID] [LEN=7] CMD [PARAM_1] ... [PARAM_N]  [CHECKSUM]
    uint8_t servo_packet[10];

    servo_packet[0] = 0x55;
    servo_packet[1] = 0x55;
    servo_packet[2] = id;
    servo_packet[3] = len;
    servo_packet[4] = cmd;
    servo_packet[5] = angle & 0xFF;
    servo_packet[6] = (angle >> 8) & 0xFF;
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
    lx16a_send_packet(254, 7, 1, 0, 1000);
    k_msleep(1000);

    lx16a_send_packet(254, 7, 1, 100, 1000);
    k_msleep(1000);

    lx16a_send_packet(254, 7, 1, 0, 2000);
    k_msleep(1000);

    return;
  
}

uint16_t angle;
float t = 0.0f;

void sin_gen(void){

    int32_t raw = 500 + (int32_t)(500.0f * sinf(t));
    if (raw < 0) raw = 0;
    if (raw > 1000) raw = 1000;
    uint16_t angle = (uint16_t)raw;
    
    for(uint8_t i; i<)
    lx16a_send_packet(254, 7, 1, angle, 20);

    k_msleep(20);
    
    t += 0.1f;

    printk("Sine value: %d time: %.2f\n", angle, (double)t);
}

static void sine_thread(void *a, void *b, void *c)
{
    printk("Sine generator started\n");

    while (1) {
        sin_gen();
        k_sleep(K_MSEC(2));
    }
}

K_THREAD_STACK_DEFINE(sine_stack, 4096);
static struct k_thread sine_thread_data;


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

    printk("STEP 9: create sensor and sine motion thread\n");

    /* sine motion thread*/
    k_thread_create(&sine_thread_data, sine_stack,
                K_THREAD_STACK_SIZEOF(sine_stack),
                sine_thread, NULL, NULL, NULL,
                5, 0, K_FOREVER);

    /* sensor thread*/
    k_thread_create(&sensor_thread_data, sensor_stack,
        K_THREAD_STACK_SIZEOF(sensor_stack),
        sensor_thread, NULL, NULL, NULL,
        5, 0, K_FOREVER);

    for (;;) {
        k_sleep(K_FOREVER);
    }
}