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

#define CELL_BALANCE_THRESHOLD_START 30 // Voltage difference between the highest and lowest cell where the balance routine will start (in mV)
#define CELL_BALANCE_THRESHOLD_END 20   // Voltage difference between the highest and lowest cell where the balance routine will stop (in mV)
#define CELL_OV_TARGET 4200             // Voltage where OV protection will trigger in mV
#define CELL_UV_TARGET 3000             // Voltage where UV protection will trigger in mV

#define BQ76920_SYS_STAT_OV (1 << 2)
#define BQ76920_SYS_STAT_UV (1 << 3)

typedef enum {
    // ---- Top-level status + control registers ----
    BQ76920_REG00_SYS_STAT       = 0x00, // SYS_STAT
    BQ76920_REG01_CELL_BAL1      = 0x01, // CELLBAL1
    BQ76920_REG02_CELL_BAL2      = 0x02, // CELLBAL2
    BQ76920_REG03_CELL_BAL3      = 0x03, // CELLBAL3
    BQ76920_REG04_SYS_CTRL1      = 0x04, // SYS_CTRL1
    BQ76920_REG05_SYS_CTRL2      = 0x05, // SYS_CTRL2
    BQ76920_REG06_PROTECT1       = 0x06, // PROTECT1
    BQ76920_REG07_PROTECT2       = 0x07, // PROTECT2
    BQ76920_REG08_PROTECT3       = 0x08, // PROTECT3
    BQ76920_REG09_OV_TRIP        = 0x09, // OV_TRIP
    BQ76920_REG0A_UV_TRIP        = 0x0A, // UV_TRIP
    BQ76920_REG0B_CC_CFG         = 0x0B, // CC_CFG

    // ---- Cell voltage registers ----
    BQ76920_REG0C_VC1_HI          = 0x0C, // VC1_HI
    BQ76920_REG0D_VC1_LO          = 0x0D, // VC1_LO
    BQ76920_REG0E_VC2_HI          = 0x0E, // VC2_HI
    BQ76920_REG0F_VC2_LO          = 0x0F, // VC2_LO
    BQ76920_REG10_VC3_HI          = 0x10, // VC3_HI
    BQ76920_REG11_VC3_LO          = 0x11, // VC3_LO
    BQ76920_REG12_VC4_HI          = 0x12, // VC4_HI
    BQ76920_REG13_VC4_LO          = 0x13, // VC4_LO
    BQ76920_REG14_VC5_HI          = 0x14, // VC5_HI
    BQ76920_REG15_VC5_LO          = 0x15, // VC5_LO

    // ---- Pack voltage ----
    BQ76920_REG2A_BAT_HI          = 0x2A, // BAT_HI
    BQ76920_REG2B_BAT_LO          = 0x2B, // BAT_LO

    // ---- TS (temperature sense) registers ----
    BQ76920_REG2C_TS1_HI          = 0x2C, // TS1_HI
    BQ76920_REG2D_TS1_LO          = 0x2D, // TS1_LO

    // ---- Coulomb counter ----
    BQ76920_REG32_CC_HI           = 0x32, // CC_HI
    BQ76920_REG33_CC_LO           = 0x33, // CC_LO

    // ---- ADC calibration ----
    BQ76920_REG50_ADC_GAIN1       = 0x50, // ADC_GAIN1
    BQ76920_REG51_ADC_OFFSET      = 0x51, // ADC_OFFSET
    BQ76920_REG59_ADC_GAIN2       = 0x59  // ADC_GAIN2

} bq76920_reg_t;

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
        void stopCellBalancing();
        float readTemp();
        BQ76920_OV_UV_STATE getOVUVState();

        void enableCharging();
        void disableCharging();
        void enableDischarging();
        void disableDischarging();
        void shipMode();
        bool inHighInputImpedance();

    private:  
        uint16_t calculateADC(uint8_t msb, uint8_t lsb);
        bool getCellVoltages();
        bool cellShouldBePopulated(int cell);
        bool writeReg(bq76920_reg_t reg, uint8_t data);
        bool writeBlock(bq76920_reg_t reg, uint8_t data[], uint8_t len);
        bool readReg(bq76920_reg_t reg, uint8_t *data);
        bool readBlock(bq76920_reg_t reg, uint8_t data[], size_t len);
        uint8_t crc8_atm(uint8_t *data, size_t len);
        void getADCGainAndOffset();
        bool setBit(bq76920_reg_t reg, uint8_t bit, bool value);
        void writeOVandUVTripVoltages();

        I2C i2c_;  
        uint8_t maxVoltageCell = 6;
        int32_t adcOffset = 0;   // ADC offset in mV
        uint32_t adcGain = 0;   // ADC gain in uV/LSB
        uint16_t cellMilliVoltages[5] = {0, 0, 0, 0, 0};
        uint16_t cellOVMilliVoltage = 0;
        uint16_t cellUVMilliVoltage = 0;
        bool balancing = false;
};

#endif
