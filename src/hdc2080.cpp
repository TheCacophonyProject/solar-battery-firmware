#include "hdc2080.h"
#include "log_codes.h"
#include "util.h"

// Datasheet: HDC2080 Low-Power Humidity and Temperature Digital Sensor, TI SNAS678C

bool HDC2080::begin() {
    uint8_t manufId[2];
    if (!i2c_.read(HDC2080_ADDR, HDC2080_REG_MANUF_ID_LO, manufId, 2)) {
        // logCode(LOG_HDC_MANUF_ID_FAIL);
        return false;
    }
    if (manufId[0] != HDC2080_MANUF_ID_LO || manufId[1] != HDC2080_MANUF_ID_HI) {
        // logCodeU16(LOG_HDC_BAD_MANUF_ID, (uint16_t)manufId[0] | ((uint16_t)manufId[1] << 8));
        return false;
    }

    // 14-bit temperature + humidity, auto measurement mode disabled (manual trigger via I2C),
    // heater and DRDY/INT pin left off. This matches the register reset defaults, but we write
    // it explicitly so behaviour doesn't depend on power-up state.
    if (!i2c_.writeReg(HDC2080_ADDR, HDC2080_REG_MEAS_CONFIG, 0x00)) {
        // logCode(LOG_HDC_CONFIG_FAIL);
        return false;
    }

    return true;
}

bool HDC2080::trigger() {
    if (!i2c_.writeReg(HDC2080_ADDR, HDC2080_REG_MEAS_CONFIG, HDC2080_MEAS_TRIG)) {
        // logCode(LOG_HDC_TRIGGER_FAIL);
        return false;
    }
    newReadingTriggered = true;
    return true;
}

bool HDC2080::readResult() {
    if (!newReadingTriggered) {
        // logCode(LOG_HDC_NO_TRIGGER);
    }
    newReadingTriggered = false;

    uint8_t status;
    if (!i2c_.readReg(HDC2080_ADDR, HDC2080_REG_DRDY, &status)) {
        // logCode(LOG_HDC_STATUS_FAIL);
        return false;
    }
    if (!(status & HDC2080_DRDY_STATUS)) {
        // logCode(LOG_HDC_BUSY);
        return false;
    }

    // Registers 0x00-0x03: TEMP_LOW, TEMP_HIGH, HUMIDITY_LOW, HUMIDITY_HIGH.
    uint8_t buf[4];
    if (!i2c_.read(HDC2080_ADDR, HDC2080_REG_TEMP_LOW, buf, 4)) {
        // logCode(LOG_HDC_DATA_FAIL);
        return false;
    }

    uint16_t rawTemp = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    uint16_t rawHum = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
    temperature_ = (float)rawTemp / 65536.0f * 165.0f - 40.5f; // Equation 1
    humidity_ = (float)rawHum / 65536.0f * 100.0f;             // Equation 3
    return true;
}
