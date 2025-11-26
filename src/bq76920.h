#ifndef BQ76920_H
#define BQ76920_H

#include "Arduino.h"
#include "i2c.h"

#define BQ76920_ADDRESS 0x08
// Register addresses
#define CELL_BALANCE_REG 0x01
#define CC_CFG_REG 0x0B


#define CELL_BALANCE_THRESHOLD_START 30
#define CELL_BALANCE_THRESHOLD_END 20
#define CELL_OV_TARGET 4200 // 4.2V         # Target of the cell over voltage in mV
#define CELL_UV_TARGET 3000 // 3.0V         # Target of the cell under voltage in mV

class BQ76920 {
    public:
        bool begin();
        void updateBalanceRoutine();
        void readChargeDischargeBits();
        void readStatus();
        
        void clearOVandUVTrip();
        void enableChargeAndDischarge();
        float ReadTemp();
        uint16_t CalculateADC(uint8_t msb, uint8_t lsb);
    private:
        bool getCellVoltages();
        bool cellShouldBePopulated(int cell);
        I2C i2c_; 
        bool writeReg(uint8_t reg, uint8_t data);
        bool writeBlock(uint8_t reg, uint8_t data[], uint8_t len);
        bool readReg(uint8_t reg, uint8_t *data);
        bool readBlock(uint8_t reg, uint8_t data[], size_t len);
        uint8_t crc8_atm(uint8_t *data, size_t len);
        void getADCGainAndOffset();
        bool setBit(uint8_t reg, uint8_t bit, bool value);
        uint8_t maxVoltageCell = 6;
        void writeOVandUVTripVoltages();
        void disableChargeAndDischarge();


        int32_t adcOffset = 0;   // ADC offset in mV
        uint32_t adcGain = 0;   // ADC gain in uV/LSB

        uint16_t cellMilliVoltages[5] = {0, 0, 0, 0, 0};

        uint16_t cellOVMilliVoltage = 0;
        uint16_t cellUVMilliVoltage = 0;
        bool balancing = false;
};

float ntc_temp_from_resistance(float r_ohms);

#endif
