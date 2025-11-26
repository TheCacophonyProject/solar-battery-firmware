#ifndef BQ25798_H
#define BQ25798_H

#include "Arduino.h"
#include "i2c.h"

#define BQ25798_DEFAULT_ADDR 0x6B


class BQ25798 {
    public:
        bool begin();
        void sourceRetry();
        void enableMPPT();
        void setVOCRate();
        void setChargeVoltageLimit();
        void setInputAndChargeLimits();
        void checkStatus();
        void init();
        //void SetBatteryChargeCurrentLimit();
        
    private:
        bool writeReg(uint8_t reg, uint8_t data);
        bool readBlock(uint8_t reg, uint8_t data[], size_t len);
        bool writeBlock(uint8_t reg, uint8_t data[], size_t len);
        bool readReg(uint8_t reg, uint8_t *data);
        bool setBit(uint8_t reg, uint8_t bit, bool value);
        bool setBits(uint8_t reg, uint8_t val, uint8_t offset, uint8_t len);
        bool writeWord(uint8_t reg, uint16_t data, bool check=false);
        bool readWord(uint8_t reg, uint16_t *data);
        I2C i2c_; 
        uint8_t chargeStatus;
        uint8_t vbusStatus;
        int16_t chargingCurrent;
};

#endif
