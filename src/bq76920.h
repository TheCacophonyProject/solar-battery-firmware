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

#define BQ76920_SYS_STAT_OV (1 << 2)
#define BQ76920_SYS_STAT_UV (1 << 3)

enum class BQ76920_REG {
    SYS_STAT = 0x00
};

enum class BQ76920_OV_UV_STATE {
    HEALTHY,
    OVER_VOLTAGE,
    UNDER_VOLTAGE,
    UNDER_VOLTAGE_AND_OVER_VOLTAGE,
};

class BQ76920 {
    public:
        bool begin();
        void updateBalanceRoutine();
        void readChargeDischargeBits();
        void readStatus();
        
        void clearOVandUVTrip();
        void enableChargeAndDischarge();
        void disableChargeAndDischarge();
        void stopCellBalancing();
        float ReadTemp();
        uint16_t CalculateADC(uint8_t msb, uint8_t lsb);

        uint8_t getReg(BQ76920_REG reg);

        BQ76920_OV_UV_STATE getOVUVState();

        void enableCharging();
        void disableCharging();
        void enableDischarging();
        void disableDischarging();

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


        int32_t adcOffset = 0;   // ADC offset in mV
        uint32_t adcGain = 0;   // ADC gain in uV/LSB

        uint16_t cellMilliVoltages[5] = {0, 0, 0, 0, 0};

        uint16_t cellOVMilliVoltage = 0;
        uint16_t cellUVMilliVoltage = 0;
        bool balancing = false;
};

#endif
