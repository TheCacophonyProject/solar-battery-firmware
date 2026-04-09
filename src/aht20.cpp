#include "aht20.h"
#include "log_codes.h"
#include "util.h"

// Datasheet: AHT20 Humidity and Temperature Sensor, ASAIR V1.0 May 2021

// readData reads 7 bytes directly from the AHT20 with no preceding write.
// The AHT20 is not register-addressed: writing 0x71 before a read returns a
// status-only response (data bytes all zero). A plain requestFrom returns the
// full status + measurement payload.
bool AHT20::readData(uint8_t buf[7]) { return i2c_.readDirect(AHT20_ADDR, buf, 7); }

// crc8 calculates CRC-8/MAXIM: polynomial 0x31, init 0xFF (section 7.4).
uint8_t AHT20::crc8(uint8_t *data, uint8_t len) {
    uint8_t crc = 0xFF;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t j = 0; j < 8; j++) {
            crc = (crc & 0x80) ? (crc << 1) ^ 0x31 : (crc << 1);
        }
    }
    return crc;
}

// begin checks calibration and initialises the sensor if needed.
bool AHT20::begin() {
    delay(100); // Section 7.1: ≥100ms after power-on before any command

    uint8_t buf[7];
    if (!readData(buf)) {
        logCode(LOG_AHT_STATUS_FAIL);
        return false;
    }

    logCodeU8(LOG_AHT_STATUS, buf[0]);

    // From section 7.4 of datasheet: Check status register & 0x18 == 0x18
    if ((buf[0] & AHT20_READY) == AHT20_READY) {
        delay(10);
        return true;
    }

    logCodeU8(LOG_AHT_STATUS, buf[0]);

    // Not calibrated: send 0xBE 0x08 0x00 to load calibration coefficients.
    uint8_t initParams[] = {0x08, 0x00};
    if (!i2c_.write(AHT20_ADDR, AHT20_CMD_INIT, initParams, 2)) {
        logCode(LOG_AHT_INIT_FAIL);
        return false;
    }
    delay(100);

    if (!readData(buf)) {
        logCode(LOG_AHT_STATUS_AFTER_INIT_FAIL);
        return false;
    }
    if ((buf[0] & AHT20_READY) != AHT20_READY) {
        logCode(LOG_AHT_NOT_CALIBRATED);
        return false;
    }
    delay(10);
    return true;
}

bool AHT20::trigger() {
    uint8_t triggerParams[] = {0x33, 0x00};
    if (!i2c_.write(AHT20_ADDR, AHT20_CMD_TRIGGER, triggerParams, 2)) {
        logCode(LOG_AHT_TRIGGER_FAIL);
        return false;
    }
    newReadingTriggered = true;
    return true;
}

bool AHT20::readResult() {
    if (!newReadingTriggered) {
        logCode(LOG_AHT_NO_TRIGGER);
    }
    newReadingTriggered = false;

    uint8_t buf[7];
    if (!readData(buf)) {
        return false;
    }
    if (buf[0] & AHT20_BUSY) {
        logCode(LOG_AHT_BUSY);
        return false;
    }

    uint8_t readCRC = buf[6];
    if (readCRC != 0xFF && readCRC != crc8(buf, 6)) {
        logCode(LOG_AHT_CRC_ERR);
        return false;
    }

    uint32_t rawHum = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t rawTemp = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    humidity_ = (float)rawHum / 1048576.0f * 100.0f;
    temperature_ = (float)rawTemp / 1048576.0f * 200.0f - 50.0f;
    return true;
}

// measure triggers a measurement, polls until complete, then parses the result.
bool AHT20::measure() {
    if (!trigger()) {
        return false;
    }

    // Datasheet says ≥80ms; poll busy bit, retry 3 times with 100ms delay.
    for (uint8_t i = 0; i < 3; i++) {
        delay(100);
        if (readResult()) {
            return true;
        }
    }

    logCode(LOG_AHT_TIMEOUT);
    return false;
}
