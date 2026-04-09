#include "i2c.h"
#include "log_codes.h"
#include "util.h"

void I2C::setup() { Wire.begin(); }

bool I2C::write(uint8_t address, uint8_t reg, uint8_t data[], uint8_t len) {
    Wire.beginTransmission(address);
    // TODO, do we need to check that these were written or will that error get caught in endTransmission?
    Wire.write(reg);
    Wire.write(data, len);
    return endTransmission();
}

// writeReg will write data to a single register
bool I2C::writeReg(uint8_t address, uint8_t reg, uint8_t data) {
    uint8_t dataArray[] = {data};
    return write(address, reg, dataArray, 1);
}

bool I2C::read(uint8_t address, uint8_t reg, uint8_t data[], uint8_t len) {
    // First we write the register that we are wanting to start the read from.
    Wire.beginTransmission(address);
    if (Wire.write(reg) != 1) {
        logCode(LOG_I2C_QUEUE_ERR);
        return false;
    }
    // End the transmission but don't send the stop bit as we want to do the read.
    uint8_t err = endTransmission(false);

    // Begin the read
    // Wire.beginTransmission(address);
    Wire.requestFrom(address, len);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return endTransmission();
}

// readDirect reads len bytes from address with no preceding register write.
// Use for peripherals like AHT20 that don't use a register-addressed read.
bool I2C::readDirect(uint8_t address, uint8_t data[], uint8_t len) {
    Wire.requestFrom(address, len);
    for (uint8_t i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return true;
}

// readReg will read data from a single register
bool I2C::readReg(uint8_t address, uint8_t reg, uint8_t *data) {
    uint8_t dataArray[1];
    // TODO, check if we can just pass in the data pointer directly
    bool success = read(address, reg, dataArray, 1);
    *data = dataArray[0];
    return success;
}

bool I2C::endTransmission(bool sendStop) {
    // Wire.flush();
    uint8_t err = Wire.endTransmission(sendStop);
    ;
    if (err == 0) {
        return true;
    }

    logCodeU8(LOG_I2C_ERR, err);
    return false;
}