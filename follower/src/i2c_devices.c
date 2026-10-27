#include "i2c_devices.h"
#include <zephyr/device.h>
#include <zephyr/drivers/i2c.h>
#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(i2c_dev, LOG_LEVEL_INF);

/* Get I2C bus 0 from Devicetree (resolved at compile time) */
static const struct device *i2c_bus = DEVICE_DT_GET(DT_NODELABEL(i2c0));

/* ---------------- Helpers ---------------- */
static int i2c_write_reg(uint8_t dev_addr, uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = { reg, value };
    return i2c_write(i2c_bus, buf, sizeof(buf), dev_addr);
}

static int i2c_read_reg(uint8_t dev_addr, uint8_t reg, uint8_t *value)
{
    return i2c_write_read(i2c_bus, dev_addr, &reg, 1, value, 1);
}

static int i2c_read_regs(uint8_t dev_addr, uint8_t reg, uint8_t *buf, size_t len)
{
    return i2c_write_read(i2c_bus, dev_addr, &reg, 1, buf, len);
}

/* ---------------- AT42QT1070 ---------------- */
int qt1070_init(void)
{
    uint8_t id = 0;
    int err = qt1070_read_chip_id(&id);
    if (err) {
        LOG_ERR("QT1070 not responding (err %d)", err);
        return err;
    }
    if (id != QT1070_CHIP_ID_VALUE) {
        LOG_ERR("QT1070 wrong chip ID: 0x%02X (expected 0x%02X)",
                id, QT1070_CHIP_ID_VALUE);
        return -EIO;
    }
    LOG_INF("QT1070 initialized, chip ID 0x%02X", id);
    return 0;
}

int qt1070_read_chip_id(uint8_t *id)
{
    return i2c_read_reg(QT1070_I2C_ADDR, QT1070_REG_CHIP_ID, id);
}

int qt1070_read_status(uint8_t *detection, uint8_t *key_status)
{
    /* The QT1070 supports multi-byte reads: read both registers in one shot */
    uint8_t buf[2];
    int err = i2c_read_regs(QT1070_I2C_ADDR,
                            QT1070_REG_DETECTION_STATUS,
                            buf, sizeof(buf));
    if (err) return err;
    if (detection)  *detection  = buf[0];
    if (key_status) *key_status = buf[1];
    return 0;
}

bool qt1070_key_pressed(uint8_t key, uint8_t detection_byte)
{
    if (key >= QT1070_NUM_KEYS) return false;
    return (detection_byte & (1 << key)) != 0;
}

int qt1070_read_signal(uint8_t key, uint16_t *signal)
{
    if (key >= QT1070_NUM_KEYS) return -EINVAL;
    uint8_t reg = QT1070_REG_KEY_SIGNAL_0 + (key * 2);
    uint8_t buf[2];
    int err = i2c_read_regs(QT1070_I2C_ADDR, reg, buf, 2);
    if (err) return err;
    /* Signal is a 16-bit big-endian value, only low 10 bits meaningful */
    *signal = ((uint16_t)buf[0] << 8) | buf[1];
    *signal &= 0x03FF;   /* 10-bit */
    return 0;
}

int qt1070_set_nthr(uint8_t value)
{
    return i2c_write_reg(QT1070_I2C_ADDR, QT1070_REG_NTHR, value);
}

int qt1070_set_di(uint8_t value)
{
    return i2c_write_reg(QT1070_I2C_ADDR, QT1070_REG_DI, value);
}

int qt1070_set_guard_channel(uint8_t ch)
{
    if (ch > 7) return -EINVAL;
    return i2c_write_reg(QT1070_I2C_ADDR, QT1070_REG_GUARD_CHANNEL, ch);
}

/* ---------------- AS5600 magnetic encoder ---------------- */
int as5600_init(uint8_t i2c_addr)
{
    uint8_t status = 0;
    int err = i2c_read_reg(i2c_addr, AS5600_REG_STATUS, &status);
    if (err) {
        LOG_ERR("AS5600 @0x%02X not responding (err %d)", i2c_addr, err);
        return err;
    }
    LOG_INF("AS5600 @0x%02X status=0x%02X", i2c_addr, status);
    return 0;
}

int as5600_read_angle(uint8_t i2c_addr, uint16_t *angle)
{
    uint8_t buf[2];
    int err = i2c_read_regs(i2c_addr, AS5600_REG_ANGLE_HI, buf, 2);
    if (err) return err;
    *angle = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;  /* 12-bit */
    return 0;
}

int as5600_read_raw_angle(uint8_t i2c_addr, uint16_t *raw_angle)
{
    uint8_t buf[2];
    int err = i2c_read_regs(i2c_addr, AS5600_REG_RAW_ANGLE_HI, buf, 2);
    if (err) return err;
    *raw_angle = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    return 0;
}

int as5600_read_magnitude(uint8_t i2c_addr, uint16_t *magnitude)
{
    uint8_t buf[2];
    int err = i2c_read_regs(i2c_addr, AS5600_REG_MAGNITUDE_HI, buf, 2);
    if (err) return err;
    *magnitude = (((uint16_t)buf[0] << 8) | buf[1]) & 0x0FFF;
    return 0;
}