#include "aht20.h"
#include "util.h"

// Datasheet: AHT20 Humidity and Temperature Sensor, ASAIR V1.0 May 2021

// readData writes AHT20_STATUS_REG (0x71) then reads 7 bytes.
// This matches the Go driver pattern: Tx(addr, {0x71}, 7).
bool AHT20::readData(uint8_t buf[7]) {
    return i2c_.read(AHT20_ADDR, AHT20_STATUS_REG, buf, 7);
}

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
        println("AHT20: status read failed");
        return false;
    }

    if ((buf[0] & AHT20_CALIBRATED) == AHT20_CALIBRATED) {
        return true;
    }

    // Not calibrated: send 0xBE 0x08 0x00 to load calibration coefficients.
    uint8_t initParams[] = {0x08, 0x00};
    if (!i2c_.write(AHT20_ADDR, AHT20_CMD_INIT, initParams, 2)) {
        println("AHT20: init command failed");
        return false;
    }
    delay(100);

    if (!readData(buf)) {
        println("AHT20: status read after init failed");
        return false;
    }
    if ((buf[0] & AHT20_CALIBRATED) != AHT20_CALIBRATED) {
        println("AHT20: not calibrated after init");
        return false;
    }
    return true;
}

bool AHT20::trigger() {
    uint8_t triggerParams[] = {0x33, 0x00};
    if (!i2c_.write(AHT20_ADDR, AHT20_CMD_TRIGGER, triggerParams, 2)) {
        println("AHT20: trigger failed");
        return false;
    }
    newReadingTriggered = true;
    return true;
}

bool AHT20::readResult() {
    if (!newReadingTriggered) {
        println("AHT20: no trigger");
    }
    newReadingTriggered = false;

    uint8_t buf[7];
    if (!readData(buf)) {
        return false;
    }
    if (buf[0] & AHT20_BUSY) {
        println("AHT20: busy");
        return false;
    }

    uint8_t readCRC = buf[6];
    if (readCRC != 0xFF && readCRC != crc8(buf, 6)) {
        println("AHT20: CRC mismatch");
        return false;
    }

    uint32_t rawHum  = ((uint32_t)buf[1] << 12) | ((uint32_t)buf[2] << 4) | (buf[3] >> 4);
    uint32_t rawTemp = ((uint32_t)(buf[3] & 0x0F) << 16) | ((uint32_t)buf[4] << 8) | buf[5];
    humidity_    = (float)rawHum / 1048576.0f * 100.0f;
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

    println("AHT20: measurement timeout");
    return false;
}
