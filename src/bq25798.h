#ifndef BQ25798_H
#define BQ25798_H

#include "Arduino.h"
#include "i2c.h"
#include "ntcTemp.h"

#define BQ25798_DEFAULT_ADDR 0x6B

enum class BQ25798_TEMP { COLD = 0, COOL, GOOD, WARM, HOT };

#define BQ25798_VBAT_OVP_STAT (1u << 5)
#define BQ25798_TS_COLD_STAT (1u << 3)
#define BQ25798_TS_COOL_STAT (1u << 2)
#define BQ25798_TS_WARM_STAT (1u << 1)
#define BQ25798_TS_HOT_STAT (1u << 0)
#define BQ25798_VBUS_PRESENT_STAT (1u << 0)
#define BQ25798_EN_HIZ (1u << 2)
#define BQ25798_EN_MPPT (1u << 0)

#define VBUS_PRESENT_FLAG (1u << 0)
#define VAC1_PRESENT_FLAG (1u << 1)
#define VAC2_PRESENT_FLAG (1u << 2)
#define POWER_GOOD_FLAG (1u << 3)
#define POORSRC_FLAG (1u << 4)
#define ADC_DONE_FLAG (1u << 5)

#define ICO_DONE_FLAG (1u << 0)
#define ICO_FAIL_FLAG (1u << 1)
#define IINDPM_FLAG (1u << 2)
#define VINDPM_FLAG (1u << 3)
#define TREG_FLAG (1u << 4)

#define CHARGE_DONE_FLAG (1u << 0)
#define RECHARGE_FLAG (1u << 1)
#define PRECHARGE_FLAG (1u << 2)
#define FASTCHARGE_FLAG (1u << 3)
#define TOPOFF_FLAG (1u << 4)
#define TERMINATION_FLAG (1u << 5)

#define TS_COLD_FLAG (1u << 3)
#define TS_COOL_FLAG (1u << 2)
#define TS_WARM_FLAG (1u << 1)
#define TS_HOT_FLAG (1u << 0)

#define VBUS_OVP_FLAG (1u << 0)
#define IBUS_OCP_FLAG (1u << 1)
#define IBAT_OCP_FLAG (1u << 2)
#define VSYS_OVP_FLAG (1u << 3)
#define VBAT_OVP_FLAG (1u << 4)
#define VBAT_UVP_FLAG (1u << 5)

#define TS_FAULT_FLAG (1u << 0)
#define TSHUT_FLAG (1u << 1)
#define WATCHDOG_FLAG (1u << 2)
#define SAFETY_TIMER_FLAG (1u << 3)

typedef enum {
    BQ25798_REG00_MIN_SYS_VOLTAGE = 0x00,      // REG00_Minimal_System_Voltage
    BQ25798_REG01_CHARGE_VOLTAGE_LIMIT = 0x01, // REG01_Charge_Voltage_Limit
    BQ25798_REG03_CHARGE_CURRENT_LIMIT = 0x03, // REG03_Charge_Current_Limit
    BQ25798_REG05_INPUT_VOLTAGE_LIMIT = 0x05,  // REG05_Input_Voltage_Limit
    BQ25798_REG06_INPUT_CURRENT_LIMIT = 0x06,  // REG06_Input_Current_Limit
    BQ25798_REG08_PRECHARGE_CONTROL = 0x08,    // REG08_Precharge_Control
    BQ25798_REG09_TERMINATION_CONTROL = 0x09,  // REG09_Termination_Control
    BQ25798_REG0A_RECHARGE_CONTROL = 0x0A,     // REG0A_Re-charge_Control
    BQ25798_REG0B_VOTG_REGULATION = 0x0B,      // REG0B_VOTG_regulation
    BQ25798_REG0D_IOTG_REGULATION = 0x0D,      // REG0D_IOTG_regulation
    BQ25798_REG0E_TIMER_CONTROL = 0x0E,        // REG0E_Timer_Control
    BQ25798_REG0F_CHARGER_CONTROL_0 = 0x0F,    // REG0F_Charger_Control_0
    BQ25798_REG10_CHARGER_CONTROL_1 = 0x10,    // REG10_Charger_Control_1
    BQ25798_REG11_CHARGER_CONTROL_2 = 0x11,    // REG11_Charger_Control_2
    BQ25798_REG12_CHARGER_CONTROL_3 = 0x12,    // REG12_Charger_Control_3
    BQ25798_REG13_CHARGER_CONTROL_4 = 0x13,    // REG13_Charger_Control_4
    BQ25798_REG14_CHARGER_CONTROL_5 = 0x14,    // REG14_Charger_Control_5
    BQ25798_REG15_MPPT_CONTROL = 0x15,         // REG15_MPPT_Control
    BQ25798_REG16_TEMPERATURE_CONTROL = 0x16,  // REG16_Temperature_Control
    BQ25798_REG17_NTC_CONTROL_0 = 0x17,        // REG17_NTC_Control_0
    BQ25798_REG18_NTC_CONTROL_1 = 0x18,        // REG18_NTC_Control_1
    BQ25798_REG19_ICO_CURRENT_LIMIT = 0x19,    // REG19_ICO_Current_Limit

    BQ25798_REG1B_CHARGER_STATUS_0 = 0x1B, // REG1B_Charger_Status_0
    BQ25798_REG1C_CHARGER_STATUS_1 = 0x1C, // REG1C_Charger_Status_1
    BQ25798_REG1D_CHARGER_STATUS_2 = 0x1D, // REG1D_Charger_Status_2
    BQ25798_REG1E_CHARGER_STATUS_3 = 0x1E, // REG1E_Charger_Status_3
    BQ25798_REG1F_CHARGER_STATUS_4 = 0x1F, // REG1F_Charger_Status_4

    BQ25798_REG20_FAULT_STATUS_0 = 0x20, // REG20_FAULT_Status_0
    BQ25798_REG21_FAULT_STATUS_1 = 0x21, // REG21_FAULT_Status_1

    BQ25798_REG22_CHARGER_FLAG_0 = 0x22, // REG22_Charger_Flag_0
    BQ25798_REG23_CHARGER_FLAG_1 = 0x23, // REG23_Charger_Flag_1
    BQ25798_REG24_CHARGER_FLAG_2 = 0x24, // REG24_Charger_Flag_2
    BQ25798_REG25_CHARGER_FLAG_3 = 0x25, // REG25_Charger_Flag_3

    BQ25798_REG26_FAULT_FLAG_0 = 0x26, // REG26_FAULT_Flag_0
    BQ25798_REG27_FAULT_FLAG_1 = 0x27, // REG27_FAULT_Flag_1

    BQ25798_REG28_CHARGER_MASK_0 = 0x28, // REG28_Charger_Mask_0
    BQ25798_REG29_CHARGER_MASK_1 = 0x29, // REG29_Charger_Mask_1
    BQ25798_REG2A_CHARGER_MASK_2 = 0x2A, // REG2A_Charger_Mask_2
    BQ25798_REG2B_CHARGER_MASK_3 = 0x2B, // REG2B_Charger_Mask_3

    BQ25798_REG2C_FAULT_MASK_0 = 0x2C, // REG2C_FAULT_Mask_0
    BQ25798_REG2D_FAULT_MASK_1 = 0x2D, // REG2D_FAULT_Mask_1

    BQ25798_REG2E_ADC_CONTROL = 0x2E,            // REG2E_ADC_Control
    BQ25798_REG2F_ADC_FUNCTION_DISABLE_0 = 0x2F, // REG2F_ADC_Function_Disable_0
    BQ25798_REG30_ADC_FUNCTION_DISABLE_1 = 0x30, // REG30_ADC_Function_Disable_1

    BQ25798_REG31_IBUS_ADC = 0x31,         // REG31_IBUS_ADC
    BQ25798_REG33_IBAT_ADC = 0x33,         // REG33_IBAT_ADC
    BQ25798_REG35_VBUS_ADC = 0x35,         // REG35_VBUS_ADC
    BQ25798_REG37_VAC1_ADC = 0x37,         // REG37_VAC1_ADC
    BQ25798_REG39_VAC2_ADC = 0x39,         // REG39_VAC2_ADC
    BQ25798_REG3B_VBAT_ADC = 0x3B,         // REG3B_VBAT_ADC
    BQ25798_REG3D_VSYS_ADC = 0x3D,         // REG3D_VSYS_ADC
    BQ25798_REG3F_TS_ADC = 0x3F,           // REG3F_TS_ADC
    BQ25798_REG41_TDIE_ADC = 0x41,         // REG41_TDIE_ADC
    BQ25798_REG43_DP_ADC = 0x43,           // REG43_D+_ADC
    BQ25798_REG45_DM_ADC = 0x45,           // REG45_D-_ADC
    BQ25798_REG47_DPDM_DRIVER = 0x47,      // REG47_DPDM_Driver
    BQ25798_REG48_PART_INFORMATION = 0x48, // REG48_Part_Information
} bq25798_reg_t;

class BQ25798 {
  public:
    bool begin(int);
    void sourceRetry();
    void checkStatus();
    void init();
    void enable();
    void disable();
    void checkSourceAndMPPT();
    void sleepMode();
    bool haveInputSource();
    bool vbatOvpStat();
    BQ25798_TEMP getTemperatureStatus();
    uint32_t poorSourceTime = 0;
    void readFlags();
    bool inHighInputImpedance();

    bool poorSourceFlag;
    bool vbusPresentFlag;

    void clearFlags();

    void disableVBUSWakeup();
    void enableVBUSWakeup();
    void resetWatchdog();
    bool vbatPresent();

  private:
    bool writeReg(bq25798_reg_t reg, uint8_t data);
    bool writeBlock(bq25798_reg_t reg, uint8_t data[], size_t len);
    bool readReg(bq25798_reg_t reg, uint8_t *data);
    bool readBlock(bq25798_reg_t reg, uint8_t data[], size_t len);
    bool setBit(bq25798_reg_t reg, uint8_t bit, bool value);
    bool setBits(bq25798_reg_t reg, uint8_t val, uint8_t offset, uint8_t len);
    bool writeWord(bq25798_reg_t reg, uint16_t data, bool check = false);
    bool readWord(bq25798_reg_t reg, uint16_t *data);
    I2C i2c_;
    uint8_t chargeStatus;
    uint8_t vbusStatus;
    int enablePin;
    void readADC();
    uint16_t readTSADC();
};

#endif
