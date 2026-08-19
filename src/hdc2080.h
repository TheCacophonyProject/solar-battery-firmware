#ifndef HDC2080_H
#define HDC2080_H

#include "Arduino.h"
#include "i2c.h"

// Datasheet: HDC2080 Low-Power Humidity and Temperature Digital Sensor, TI SNAS678C

// ADDR pin tied to GND (or left unconnected) on this PCB -> slave address 1000000.
#define HDC2080_ADDR 0x40

#define HDC2080_REG_TEMP_LOW    0x00
#define HDC2080_REG_DRDY        0x04 // Interrupt/DRDY status
#define HDC2080_REG_CONFIG      0x0E // Soft reset & DRDY/INT configuration
#define HDC2080_REG_MEAS_CONFIG 0x0F // Measurement configuration / trigger
#define HDC2080_REG_MANUF_ID_LO 0xFC

#define HDC2080_DRDY_STATUS (1u << 7) // Set in HDC2080_REG_DRDY when a conversion has completed
#define HDC2080_MEAS_TRIG   (1u << 0) // Written to HDC2080_REG_MEAS_CONFIG to start a measurement

#define HDC2080_MANUF_ID_LO 0x49
#define HDC2080_MANUF_ID_HI 0x54

class HDC2080 {
  public:
    // begin verifies the manufacturer ID and puts the sensor into a known
    // configuration (14-bit humidity + temperature, manual trigger, no auto
    // measurement, heater/interrupt off).
    bool begin();

    // trigger starts a humidity + temperature conversion and returns immediately.
    // Conversion completes well within a second (sub-ms per the datasheet), so
    // it's safe to call readResult() on the next loop iteration.
    bool trigger();

    // readResult reads and parses a previously triggered measurement.
    // Returns false if the conversion hasn't completed yet or a read error occurs.
    bool readResult();

    float temperature() const { return temperature_; }
    float humidity() const { return humidity_; }

    // Set by trigger(), cleared after the result is consumed. Used to detect
    // cases where readResult() is called without a prior trigger.
    bool newReadingTriggered = false;

  private:
    I2C i2c_;
    float temperature_ = 0.0f;
    float humidity_    = 0.0f;
};

#endif
