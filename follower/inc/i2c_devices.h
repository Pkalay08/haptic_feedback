#ifndef I2C_DEVICES_H_
#define I2C_DEVICES_H_

#include <zephyr/kernel.h>
#include <stdint.h>

/* ---------------- AT42QT1070 ---------------- */
#define QT1070_I2C_ADDR              0x1B   /* 7-bit address */

/* Register map (AT42QT1070 datasheet §8) */
#define QT1070_REG_CHIP_ID           0x00   /* RO: reads 0x2E */
#define QT1070_REG_FW_VERSION        0x01   /* RO */
#define QT1070_REG_DETECTION_STATUS  0x02   /* RO: bit per key */
#define QT1070_REG_KEY_STATUS        0x03   /* RO: bit per key (filtered) */
#define QT1070_REG_KEY_SIGNAL_0      0x04   /* RO: 16-bit per key, big-endian */
#define QT1070_REG_KEY_REF_0         0x06   /* RO: 16-bit per key */
#define QT1070_REG_NTHR              0x10   /* RW: detection threshold */
#define QT1070_REG_CTRL              0x11   /* RW: control */
#define QT1070_REG_DATA0             0x12   /* RW: data byte 0 */
#define QT1070_REG_DATA1             0x13   /* RW: data byte 1 */
#define QT1070_REG_DATA2             0x14   /* RW: data byte 2 */
#define QT1070_REG_DATA3             0x15   /* RW: data byte 3 */
#define QT1070_REG_DATA4             0x16   /* RW: data byte 4 */
#define QT1070_REG_DATA5             0x17   /* RW: data byte 5 */
#define QT1070_REG_DI                0x18   /* RW: detection integrator */
#define QT1070_REG_GUARD_CHANNEL     0x19   /* RW: 0=off, 1..7 = channel X */

#define QT1070_CHIP_ID_VALUE         0x2E
#define QT1070_NUM_KEYS              7

int  qt1070_init(void);
int  qt1070_read_chip_id(uint8_t *id);
int  qt1070_read_status(uint8_t *detection, uint8_t *key_status);
bool qt1070_key_pressed(uint8_t key, uint8_t detection_byte);
int  qt1070_read_signal(uint8_t key, uint16_t *signal);
int  qt1070_set_nthr(uint8_t value);
int  qt1070_set_di(uint8_t value);
int  qt1070_set_guard_channel(uint8_t ch);

/* ---------------- Magnetic encoders ---------------- */
/* Example: AS5600 (12-bit magnetic rotary encoder) */
#define AS5600_I2C_ADDR              0x36
#define AS5600_REG_STATUS            0x0B
#define AS5600_REG_RAW_ANGLE_HI      0x0C   /* RO: 12-bit angle, big-endian */
#define AS5600_REG_RAW_ANGLE_LO      0x0D
#define AS5600_REG_ANGLE_HI          0x0E   /* RO: scaled angle */
#define AS5600_REG_ANGLE_LO          0x0F
#define AS5600_REG_AGC               0x1A   /* RO: automatic gain control */
#define AS5600_REG_MAGNITUDE_HI      0x1B   /* RO: 12-bit magnitude */
#define AS5600_REG_MAGNITUDE_LO      0x1C

int as5600_init(uint8_t i2c_addr);
int as5600_read_angle(uint8_t i2c_addr, uint16_t *angle);
int as5600_read_raw_angle(uint8_t i2c_addr, uint16_t *raw_angle);
int as5600_read_magnitude(uint8_t i2c_addr, uint16_t *magnitude);

#endif /* I2C_DEVICES_H_ */