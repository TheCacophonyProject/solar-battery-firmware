#ifndef BQ76920_H
#define BQ76920_H

#include "Arduino.h"
#include "i2c.h"

// ======= TESTING PARAMETERS ======= //

// This will set the minimum cell voltage to the highest possible value to make it easier testing 
// the low cell voltage threshold.
#define TEST_MINIMUM_CELL_VOLTAGE false

// This will set the maximum cell voltage to the lowest possible value to make it easier testing 
// the high cell voltage threshold.
#define TEST_MAXIMUM_CELL_VOLTAGE false

// ======== BQ76920 ADDRESSES  ======== //

#define BQ76920_ADDRESS 0x08
#define CELL_BALANCE_REG 0x01
#define CC_CFG_REG 0x0B


#define CELL_BALANCE_THRESHOLD_START 30 // Voltage difference between the highest and lowest cell where the balance routine will start (in mV)
#define CELL_BALANCE_THRESHOLD_END 20   // Voltage difference between the highest and lowest cell where the balance routine will stop (in mV)
#define CELL_OV_TARGET 4200             // Voltage where OV protection will trigger in mV
#define CELL_UV_TARGET 3000             // Voltage where UV protection will trigger in mV

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
