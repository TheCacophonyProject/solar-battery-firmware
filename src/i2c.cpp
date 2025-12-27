#include "i2c.h"
#include "util.h"


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
        println("Wire: failed to queue register byte");
        return false;
    }
    // End the transmission but don't send the stop bit as we want to do the read.
    uint8_t err = endTransmission(false);
    
    // Begin the read
    //Wire.beginTransmission(address);
    Wire.requestFrom(address, len);
    for (uint8_t i = 0; i < len; i++) {
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
    //Wire.flush();
    uint8_t err = Wire.endTransmission(sendStop);;
    if (err == 0) {
        return true;
    }

    // Something went wrong..
    switch (err) {
        print("Wire error: 0x"); 
        println(err, HEX);

        /*
        case 0x00:  println("Wire transmit was successful"); break;
        case 0x02:  println("Address was NACK'd"); break;
        case 0x03:  println("Data was NACK'd"); break;
        case 0x04:  println("Unknown error occurred"); break;
        case 0x05:  println("Transmission time-outed"); break;
        // The library also supports some extended errors that might give a hint on what is failing.
        case 0x10:  println("Wire is uninitialized"); break;
        case 0x11:  println("Pullups are missing"); break;
        case 0x12:  println("Arbitration lost"); break;
        */
    }
    return false;
}