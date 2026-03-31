#include "bq25798.h"
#include "log_codes.h"
#include "util.h"

// Datasheet: https://www.ti.com/lit/ds/symlink/bq25798.pdf

// Check that we can find the BQ76920 on the I2C bus.
// This is done by reading the Part Information Register (0x48).
// Returning false if it can not be found.
bool BQ25798::begin(int enablePin) {
    // Set enable/disable pin as an output.
    this->enablePin = enablePin;
    pinMode(enablePin, OUTPUT);
    // Set it to high to disable the chip.
    // The main loop will enable it when ready.
    // disable();

    // Make sure we are talking to the right chip.
    // From section 9.5.1.57
    // Check that Device Part Number is 0x03
    uint8_t regData;
    readReg(BQ25798_REG48_PART_INFORMATION, &regData);
    if ((regData & 0x38) != 0x18) {
        logCodeU8(LOG_CHG_BAD_PART, regData);
        return false;
    }

    // Read from the CELL register (REG0x0A[7:6])
    // We should be configured for 3 cells (both for LiFePO4 4S and Li-ion 3S as
    // LiFePO4 4S have a similar voltage range). The actual voltage range is
    // configured at a different register.
    readReg(BQ25798_REG0A_RECHARGE_CONTROL, &regData);
    if ((regData & 0xC0) != 0x80) {
        logCode(LOG_CHG_BAD_CELL_CNT);
        return false;
    }

    // Read frequency from PWM_FREQ
    // This is set by a resistor (same one that sets the cell count) to 750MHz,
    // if it is not 750MHz then something is wrong.
    readReg(BQ25798_REG13_CHARGER_CONTROL_4, &regData);
    if ((regData & 0x20) != 0x20) {
        logCode(LOG_CHG_BAD_PWM_FREQ);
        return false;
    }

    init();

    checkStatus();
    return true;
}

void BQ25798::enable() {
    _sleeping = false;
    digitalWrite(enablePin, LOW);
}

void BQ25798::disable() { digitalWrite(enablePin, HIGH); }

void BQ25798::sleepMode() {
    // Enable a low power sleep mode by setting it to ship mode
    _sleeping = true;
    writeReg(BQ25798_REG11_CHARGER_CONTROL_2, 0x44);
}

bool BQ25798::isSleeping() { return _sleeping; }

bool BQ25798::haveInputSource() {

    uint8_t chargerStatus0;
    readReg(BQ25798_REG1B_CHARGER_STATUS_0, &chargerStatus0);
    if (!(chargerStatus0 & BQ25798_VBUS_PRESENT_STAT)) {
        return false;
    }

    return true;
}

void BQ25798::checkSourceAndMPPT() {
    // If the source is found to be bad then it will by default not retry for 7
    // minutes. For our setup in low light solar this will be too long (a cloud
    // could be passing causing a bad source adn we could recover a lot sooner
    // than 7 minutes).

    // TODO track what state we are in and log when there is a change.

    // Check if we have an input source.
    uint8_t chargerStatus0;
    readReg(BQ25798_REG1B_CHARGER_STATUS_0, &chargerStatus0);
    if (!(chargerStatus0 & BQ25798_VBUS_PRESENT_STAT)) {
        return;
    }

    // Check if we need to disable high impedance mode.
    uint8_t chargerControl0;
    readReg(BQ25798_REG0F_CHARGER_CONTROL_0, &chargerControl0);
    if (chargerControl0 & BQ25798_EN_HIZ) {
        // Check source again.
        // From 9.3.4.2 Poor Source Qualification:
        // the host may set EN_HIZ = 0 to force an immediate retry of the poor
        // source qualification.
        setBit(BQ25798_REG0F_CHARGER_CONTROL_0, 2, false);
        logCode(LOG_CHG_DIS_HIZ);
    }

    // Check if MPPT is enabled.
    uint8_t mpptControl;
    readReg(BQ25798_REG15_MPPT_CONTROL, &mpptControl);
    if (!(mpptControl & BQ25798_EN_MPPT)) {
        // MPPT is disabled, enable it.
        logCode(LOG_CHG_EN_MPPT);
        setBit(BQ25798_REG15_MPPT_CONTROL, 0, true);
    }
}

void BQ25798::disableVBUSWakeup() {
    logCode(LOG_CHG_DIS_VBUS_WK);
    writeReg(BQ25798_REG28_CHARGER_MASK_0, 0x11);
};

void BQ25798::enableVBUSWakeup() {
    logCode(LOG_CHG_EN_VBUS_WK);
    writeReg(BQ25798_REG28_CHARGER_MASK_0, 0x00);
};

// vbatPresent returns true if there is a battery pack connected.
// This can sometimes return a false positive when there is not a battery pack
// connected. I don't think the chip should do this but we just want to detect
// if there is a voltage source so we know if devices powered from vbat (like
// the balancer IC) can be expected to be running.
bool BQ25798::vbatPresent() {
    uint8_t regData;
    readReg(BQ25798_REG1D_CHARGER_STATUS_2, &regData);
    uint8_t VBAT_PRESENT_STAT_MASK = 0x01;
    return (regData & VBAT_PRESENT_STAT_MASK) == VBAT_PRESENT_STAT_MASK;
}

void BQ25798::init() {
    logCode(LOG_CHG_INIT);

    // From 9.5.1.7: Reset registers to default values and reset timer by
    // writing 1 to bit 6 on BQ25798_REG09_TERMINATION_CONTROL
    setBit(BQ25798_REG09_TERMINATION_CONTROL, 6, true);

    // 9.5.1.1: REG00_Minimal_System_Voltage -- See BQ25798_CFG_MIN_SYS_VOLTAGE in bq25798.h
    writeReg(BQ25798_REG00_MIN_SYS_VOLTAGE, BQ25798_CFG_MIN_SYS_VOLTAGE);

    // 0x01 REG01_Charge_Voltage_Limit -- See BQ25798_CFG_CHARGE_VOLTAGE_10MV in bq25798.h
    writeWord(BQ25798_REG01_CHARGE_VOLTAGE_LIMIT, uint16_t(BQ25798_CFG_CHARGE_VOLTAGE_10MV), true);

    // 0x03 REG03_Charge_Current_Limit -- See BQ25798_CFG_CHARGE_CURRENT_10MA in bq25798.h
    writeWord(BQ25798_REG03_CHARGE_CURRENT_LIMIT, uint16_t(BQ25798_CFG_CHARGE_CURRENT_10MA), true);

    // 0x05 REG05_Input_Voltage_Limit -- See BQ25798_CFG_INPUT_VOLTAGE_100MV in bq25798.h
    writeReg(BQ25798_REG05_INPUT_VOLTAGE_LIMIT, BQ25798_CFG_INPUT_VOLTAGE_100MV);

    // 0x06 REG06_Input_Current_Limit -- See BQ25798_CFG_INPUT_CURRENT_10MA in bq25798.h
    writeWord(BQ25798_REG06_INPUT_CURRENT_LIMIT, uint16_t(BQ25798_CFG_INPUT_CURRENT_10MA), true);

    // 0x08 REG08_Precharge_Control -- Leave as default

    // 0x09 REG09_Termination_Control -- Need to program
    writeReg(BQ25798_REG09_TERMINATION_CONTROL, B00000101);

    // 0x0A REG0A_Re-charge_Control R -- Need to program
    // Check section 9.5.1.8 for details on setting register.
    // We charge up to 4.1V per cell, and should start re-charging at 3.95V so
    // that gives a voltage of 0.45V for the 3S cells. With the Fixed Offset :
    // 50mV and Bit Step Size : 50mV that means a value of 8 is needed (for the
    // last 4 bits in this register)
    writeReg(BQ25798_REG0A_RECHARGE_CONTROL, B10101000);

    // 0x0B REG0B_VOTG_regulation -- Leave as default

    // 0x0D REG0D_IOTG_regulation -- Leave as default

    // 0x0E REG0E_Timer_Control -- Leave as default

    // 0x0F REG0F_Charger_Control_0 -- Leave as default

    // 0x10 REG10_Charger_Control_1 -- Need to program
    writeReg(BQ25798_REG10_CHARGER_CONTROL_1, 0x85);

    // 0x11 REG11_Charger_Control_2 -- Need to program
    // Disable AUTO_INDET_EN bit
    writeReg(BQ25798_REG11_CHARGER_CONTROL_2, 0x00);

    // 0x12 REG12_Charger_Control_3 -- Need to program PFM?
    writeReg(BQ25798_REG12_CHARGER_CONTROL_3, 0x00);

    // 0x13 REG13_Charger_Control_4 -- Need to program
    writeReg(BQ25798_REG13_CHARGER_CONTROL_4, 0b11100000);

    // 0x14 REG14_Charger_Control_5 -- Need to program
    // 0x14 = 0001 0100: EN_EXTILIM=0 (ignore ILIM pin resistor, use IINDPM register only),
    // EN_IINDPM=1 (internal IINDPM regulation enabled), IBAT_REG=01 (4A OTG discharge limit)
    writeReg(BQ25798_REG14_CHARGER_CONTROL_5, 0x14);

    // 0x15 REG15_MPPT_Control -- Need to program
    writeReg(BQ25798_REG15_MPPT_CONTROL, 0xA8);

    // 0x16 REG16_Temperature_Control -- Need to program
    writeReg(BQ25798_REG16_TEMPERATURE_CONTROL, 0b01110000);

    // 0x17 REG17_NTC_Control_0 -- Need to program
    // Currently we are suspending charge when it is WARM, need to review this.
    writeReg(BQ25798_REG17_NTC_CONTROL_0, 0b00000010);

    // 0x18 REG18_NTC_Control_1 -- Leave as default

    // 0x19 REG19_ICO_Current_Limit -- Leave as default

    // 0x1B REG1B_Charger_Status_0 -- Read only.

    // 0x1C REG1C_Charger_Status_1 -- Read only.

    // 0x1D REG1D_Charger_Status_2 -- Read only.

    // 0x1E REG1E_Charger_Status_3 -- Read only.

    // 0x1F REG1F_Charger_Status_4 -- Read only.

    // 0x20 REG20_FAULT_Status_0 -- Read only.

    // 0x21 REG21_FAULT_Status_1 -- Read only.

    // 0x22 REG22_Charger_Flag_0 -- Read only.

    // 0x23 REG23_Charger_Flag_1 -- Read only.

    // 0x24 REG24_Charger_Flag_2 -- Read only.

    // 0x25 REG25_Charger_Flag_3 -- Read only.

    // 0x26 REG26_FAULT_Flag_0 -- Read only.

    // 0x27 REG27_FAULT_Flag_1 -- Read only.

    // 0x28 REG28_Charger_Mask_0 -- Leave as default.

    // 0x29 REG29_Charger_Mask_1 -- Need to program.
    // TREG_MASK bit is set so we don't get a INT pulse from it.
    writeReg(BQ25798_REG29_CHARGER_MASK_1, 0x04);

    // 0x2A REG2A_Charger_Mask_2 -- Need to program.
    // ADC conversion done does NOT produce INT pulse
    writeReg(BQ25798_REG2A_CHARGER_MASK_2, 0x20);

    // 0x2B REG2B_Charger_Mask_3 -- Leave as default.

    // 0x2C REG2C_FAULT_Mask_0 -- Leave as default.

    // 0x2D REG2D_FAULT_Mask_1 -- Leave as default.

    // 0x2E REG2E_ADC_Control -- Leave as default.

    // 0x2F REG2F_ADC_Function_Disable_0 -- Leave as default.

    // 0x30 REG30_ADC_Function_Disable_1 -- Leave as default.

    // 0x31 REG31_IBUS_ADC -- Read only.

    // 0x33 REG33_IBAT_ADC -- Read only.

    // 0x35 REG35_VBUS_ADC -- Read only.

    // 0x37 REG37_VAC1_ADC -- Read only.

    // 0x39 REG39_VAC2_ADC -- Read only.

    // 0x3B REG3B_VBAT_ADC -- Read only.

    // 0x3D REG3D_VSYS_ADC -- Read only.

    // 0x3F REG3F_TS_ADC -- Read only.

    // 0x41 REG41_TDIE_ADC -- Read only.

    // 0x43 REG43_D+_ADC Register -- Read only.

    // 0x45 REG45_D-_ADC Register -- Read only.

    // 0x47 REG47_DPDM_Driver -- Leave as default.

    // 0x48 REG48_Part_Information -- Read only.
}

BQ25798_TEMP BQ25798::getTemperatureStatus() {
    uint8_t data;
    if (data & BQ25798_TS_HOT_STAT) {
        return BQ25798_TEMP::HOT;
    }
    if (data & BQ25798_TS_COLD_STAT) {
        return BQ25798_TEMP::COLD;
    }
    if (data & BQ25798_TS_WARM_STAT) {
        return BQ25798_TEMP::WARM;
    }
    if (data & BQ25798_TS_COOL_STAT) {
        return BQ25798_TEMP::COOL;
    }
    return BQ25798_TEMP::GOOD;
}

void BQ25798::checkStatus() {
    uint8_t statusRegisterData;
    readReg(BQ25798_REG1C_CHARGER_STATUS_1, &statusRegisterData);

    uint8_t newChargeStatus = (statusRegisterData >> 5) & 0x07;
    if (newChargeStatus != chargeStatus) {
        chargeStatus = newChargeStatus;
        logCodeU8(LOG_CHG_CHARGE_STATUS, chargeStatus);
    }

    uint8_t newVbusStatus = (statusRegisterData >> 1) & 0x0F;
    if (newVbusStatus != vbusStatus) {
        vbusStatus = newVbusStatus;
        logCodeU8(LOG_CHG_VBUS_STATUS, vbusStatus);
    }

    uint8_t faultRegVal;
    readReg(BQ25798_REG20_FAULT_STATUS_0, &faultRegVal);
    if (faultRegVal != 0x00) {
        logCodeU8(LOG_CHG_FAULT0, faultRegVal);
    }
    readReg(BQ25798_REG21_FAULT_STATUS_1, &faultRegVal);
    if (faultRegVal != 0x00) {
        logCodeU8(LOG_CHG_FAULT1, faultRegVal);
    }

    // The flags are cleared once they are read.
    readFlags();

    // Reset WDT
    setBit(BQ25798_REG10_CHARGER_CONTROL_1, 3, true);
}

void BQ25798::clearFlags() {
    poorSourceFlag = false;
    vbusPresentFlag = false;
}

bool BQ25798::vbatOvpStat() {
    uint8_t faultStatus0;
    readReg(BQ25798_REG20_FAULT_STATUS_0, &faultStatus0);
    return faultStatus0 & BQ25798_VBAT_OVP_STAT;
}

void BQ25798::readFlags() {
    uint8_t flags[6] = {0};

    // REG22 .. REG27
    readBlock(BQ25798_REG22_CHARGER_FLAG_0, flags, 6);

    if (flags[0] & VBUS_PRESENT_FLAG)
        vbusPresentFlag = true;
    if (flags[0] & POORSRC_FLAG)
        poorSourceFlag = true;

    logCodeBytes(LOG_CHG_FLAGS, flags, 6);
}

void BQ25798::sourceRetry() {
    // From 9.3.4.2 Poor Source Qualification:
    // the host may set EN_HIZ = 0 to force an immediate retry of the poor
    // source qualification.
    setBit(BQ25798_REG0F_CHARGER_CONTROL_0, 2, false);
}

bool BQ25798::writeWord(bq25798_reg_t reg, uint16_t data, bool check) {
    uint8_t out[] = {static_cast<uint8_t>(data >> 8), static_cast<uint8_t>(data & 0xFF)};
    if (!writeBlock(reg, out, 2)) {
        logCode(LOG_CHG_WR_ERR);
        return false;
    }
    // Return true if we don't need to read back to check the write.
    if (!check) {
        return true;
    }

    uint16_t uint16_read;

    if (!readWord(reg, &uint16_read)) {
        logCode(LOG_CHG_RD_ERR);
        return false;
    }

    if (uint16_read != data) {
        logCodeU8U16U16(LOG_CHG_WR_MISMATCH, uint8_t(reg), data, uint16_read);
        return false;
    }
    return true;
}

bool BQ25798::readWord(bq25798_reg_t reg, uint16_t *data) {
    uint8_t in[2];
    if (!readBlock(reg, in, 2)) {
        return false;
    }
    *data = (static_cast<uint16_t>(in[0]) << 8 | static_cast<uint16_t>(in[1]));
    return true;
}

bool BQ25798::setBit(bq25798_reg_t reg, uint8_t bit, bool value) {
    uint8_t regValue;
    readReg(reg, &regValue);

    if (value) {
        regValue |= (1 << bit);
    } else {
        regValue &= ~(1 << bit);
    }

    return writeReg(reg, regValue);
}

bool BQ25798::setBits(bq25798_reg_t reg, uint8_t val, uint8_t offset, uint8_t len) {
    // Read the current register value
    uint8_t regValue;
    readReg(reg, &regValue);

    // Clear the bits that should be written to
    // (1 << len) -1 makes a bunch of 1s that is len long.
    // << offset shifts the 1s to the correct position.
    // finally &= ~.. will clear the bits.
    regValue &= ~(((1 << len) - 1) << offset);

    // Write the new value
    regValue |= (val << offset);
    return writeReg(reg, regValue);
}

void BQ25798::readADC() {
    writeReg(BQ25798_REG2E_ADC_CONTROL, 0xC0);

    uint8_t regData = 0;
    while (true) {
        delay(100);
        readReg(BQ25798_REG2E_ADC_CONTROL, &regData);
        if ((regData & 0x80) == 0) {
            break;
        }
    }
}

bool BQ25798::writeReg(bq25798_reg_t reg, uint8_t data) { return i2c_.writeReg(BQ25798_DEFAULT_ADDR, reg, data); }

bool BQ25798::readReg(bq25798_reg_t reg, uint8_t *data) {
    return i2c_.readReg(BQ25798_DEFAULT_ADDR, uint8_t(reg), data);
}

bool BQ25798::readBlock(bq25798_reg_t reg, uint8_t data[], size_t len) {
    return i2c_.read(BQ25798_DEFAULT_ADDR, reg, data, len);
}

bool BQ25798::writeBlock(bq25798_reg_t reg, uint8_t data[], size_t len) {
    return i2c_.write(BQ25798_DEFAULT_ADDR, reg, data, len);
}

float BQ25798::readTemp() {
    readADC();
    uint16_t val;
    readWord(BQ25798_REG3F_TS_ADC, &val);

    // Voltage divider diagram
    //
    // REGN──┐
    //       |
    //       R1
    //       |
    //  TS ──┼───┐
    //       R2  Rt
    //       ├───┘
    //       |
    // GND ──┘

    // R2 is in parallel with Rt
    // R1 is in series with Rt and R2
    // This is the equation for finding Rt from the two resistors
    // and the percentage of the voltage divider.

    float percentage = val * 0.000976563f;
    float r1 = BQ25798_NTC_R1_OHMS;
    float r2 = BQ25798_NTC_R2_OHMS;
    float rt = (percentage * r1 * r2) / (r2 - percentage * (r1 + r2));
    return ntcTempFromResistance(uint32_t(rt));
}

bool BQ25798::readStatusRegs(uint8_t out[5]) { return readBlock(BQ25798_REG1B_CHARGER_STATUS_0, out, 5); }

void BQ25798::readADCAll(BQ25798ADC &out) {
    readADC(); // one-shot conversion; all channels enabled by default (REG2F/REG30 = 0x00)

    uint16_t val;
    readWord(BQ25798_REG31_IBUS_ADC, &val);
    out.ibus_ma = val;

    readWord(BQ25798_REG33_IBAT_ADC, &val);
    out.ibat_ma = int16_t(val);

    readWord(BQ25798_REG35_VBUS_ADC, &val);
    out.vbus_mv = val;

    readWord(BQ25798_REG3B_VBAT_ADC, &val);
    out.vbat_mv = val;

    readWord(BQ25798_REG3F_TS_ADC, &val);
    float p = val * 0.000976563f;
    float r1 = BQ25798_NTC_R1_OHMS;
    float r2 = BQ25798_NTC_R2_OHMS;
    float rt = (p * r1 * r2) / (r2 - p * (r1 + r2));
    out.tempC = ntcTempFromResistance(uint32_t(rt));
}

bool BQ25798::inHighInputImpedance() {
    uint8_t chargeStatus;
    readReg(BQ25798_REG0F_CHARGER_CONTROL_0, &chargeStatus);
    if ((chargeStatus & 0x04) == 0x04) {
        return true;
    }
    return false;
}
