#include "i2c.h"


void I2C::setup() {
    Wire.begin();
}

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
        Serial.println("Wire: failed to queue register byte");
        return false;
    }
    // End the transmission but don't send the stop bit as we want to do the read.
    uint8_t err = endTransmission(false);
    
    // Begin the read
    Wire.beginTransmission(address);
    Wire.requestFrom(address, len);
    for (int i = 0; i < len; i++) {
        data[i] = Wire.read();
    }
    return endTransmission();
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
    uint8_t err = Wire.endTransmission();
    if (err == 0) {
        return true;
    }

    // Something went wrong..
    switch (err) {
        case 0x00:  Serial.println("Wire transmit was successful"); break;
        case 0x02:  Serial.println("Address was NACK'd"); break;
        case 0x03:  Serial.println("Data was NACK'd"); break;
        case 0x04:  Serial.println("Unknown error occurred"); break;
        case 0x05:  Serial.println("Transmission time-outed"); break;
        // The library also supports some extended errors that might give a hint on what is failing.
        case 0x10:  Serial.println("Wire is uninitialized"); break;
        case 0x11:  Serial.println("Pullups are missing"); break;
        case 0x12:  Serial.println("Arbitration lost"); break;
    }
    return false;
}