#ifndef AHT20_H
#define AHT20_H

#include "Arduino.h"
#include "i2c.h"

// Datasheet: AHT20 Humidity and Temperature Sensor, ASAIR V1.0 May 2021

#define AHT20_ADDR          0x38 // Fixed I2C address (section 7.3)
#define AHT20_STATUS_REG    0x71 // Written before every read to get status/data
#define AHT20_BUSY          (1u << 7)
#define AHT20_CALIBRATED    (1u << 3)
#define AHT20_READY         (0x18u) // bits 3 and 4 must both be set (section 7.4)
#define AHT20_CMD_TRIGGER   0xAC // Trigger measurement (params: 0x33 0x00)
#define AHT20_CMD_INIT      0xBE // Initialise calibration (params: 0x08 0x00)

class AHT20 {
  public:
    // begin checks calibration status and initialises if needed.
    // Call after Wire.begin() with ≥100ms elapsed since power-on (section 7.1).
    bool begin();

    // measure triggers a single measurement and waits for it to complete.
    // Returns true on success. Call temperature() and humidity() for results.
    bool measure();

    // trigger sends the measurement command and returns immediately.
    // Call readResult() after at least 80ms (e.g. after the ATtiny wakes from sleep).
    bool trigger();

    // readResult reads and parses a previously triggered measurement.
    // Returns false if the sensor is still busy or a read error occurs.
    bool readResult();

    float temperature() const { return temperature_; }
    float humidity() const { return humidity_; }

    // Set by trigger(), cleared after the result is consumed. Used to detect
    // cases where readResult() is called without a prior trigger.
    bool newReadingTriggered = false;

  private:
    // readData writes AHT20_STATUS_REG then reads 7 bytes (status + 5 data + CRC).
    bool readData(uint8_t buf[7]);
    void resetCalReg(uint8_t reg);
    // crc8 calculates CRC-8 with polynomial 0x31, init 0xFF (section 7.4).
    uint8_t crc8(uint8_t *data, uint8_t len);

    I2C i2c_;
    float temperature_ = 0.0f;
    float humidity_    = 0.0f;
};

#endif
