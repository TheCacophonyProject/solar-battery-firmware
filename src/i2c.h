#ifndef I2C_H
#define I2C_H

#include "Arduino.h"
#include <Wire.h>

class I2C {
  public:
    void setup();
    bool write(uint8_t address, uint8_t register, uint8_t data[], uint8_t len);
    bool writeReg(uint8_t address, uint8_t register, uint8_t data);
    bool read(uint8_t address, uint8_t register, uint8_t data[], uint8_t len);
    bool readReg(uint8_t address, uint8_t register, uint8_t *data);

  private:
    bool endTransmission(bool sendStop = true);
};

#endif
