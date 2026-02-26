#include "bq25798.h"
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
        println("Bad part#");
        println(regData, HEX);
        return false;
    }

    // Read from the CELL register (REG0x0A[7:6])
    // We should be configured for 3 cells (both for LiFePO4 4S and Li-ion 3S as
    // LiFePO4 4S have a similar voltage range). The actual voltage range is
    // configured at a different register.
    readReg(BQ25798_REG0A_RECHARGE_CONTROL, &regData);
    if ((regData & 0xC0) != 0x80) {
        println("Bad cell cnt");
        return false;
    }

    // Read frequency from PWM_FREQ
    // This is set by a resistor (same one that sets the cell count) to 750MHz,
    // if it is not 750MHz then something is wrong.
    readReg(BQ25798_REG13_CHARGER_CONTROL_4, &regData);
    if ((regData & 0x20) != 0x20) {
        println("Bad PWM freq");
        return false;
    }

    init();

    checkStatus();
    return true;
}

void BQ25798::enable() { digitalWrite(enablePin, LOW); }

void BQ25798::disable() { digitalWrite(enablePin, HIGH); }

void BQ25798::sleepMode() {
    // Enable a low power sleep mode by setting it to ship mode
    writeReg(BQ25798_REG11_CHARGER_CONTROL_2, 0x44);
}

bool BQ25798::haveInputSource() {

    uint8_t chargerStatus0;
    readReg(BQ25798_REG1B_CHARGER_STATUS_0, &chargerStatus0);
    if (!(chargerStatus0 & BQ25798_VBUS_PRESENT_STAT)) {
        // println("Source not present.");
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
        // println("Source not present.");
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
        println("Dis HIZ");
    }

    // Check if MPPT is enabled.
    uint8_t mpptControl;
    readReg(BQ25798_REG15_MPPT_CONTROL, &mpptControl);
    if (!(mpptControl & BQ25798_EN_MPPT)) {
        // MPPT is disabled, enable it.
        println("En MPPT");
        setBit(BQ25798_REG15_MPPT_CONTROL, 0, true);
    }
}

void BQ25798::disableVBUSWakeup() {
    println("Dis VBUS wkup");
    writeReg(BQ25798_REG28_CHARGER_MASK_0, 0x11);
};

void BQ25798::enableVBUSWakeup() {
    println("En VBUS wkup");
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
    println("Init");

    // From 9.5.1.7: Reset registers to default values and reset timer by
    // writing 1 to bit 6 on BQ25798_REG09_TERMINATION_CONTROL
    println("Reset regs");
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
    writeReg(BQ25798_REG14_CHARGER_CONTROL_5, 0x16);

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

    // 0x2E REG2E_ADC_Control -- Need to program.
    /*
    if (!writeReg(0x2E, 0b10100000)) {
        println("Failed to write ADC Control!!!!");
    }
        */

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

    // println("status check");
    //
    // readReg(BQ25798_REG1B_CHARGER_STATUS_0, &statusRegisterData);
    // print("REG1B_CHARGER_STATUS_0: ");
    // println(statusRegisterData, HEX);
    // readReg(BQ25798_REG1C_CHARGER_STATUS_1, &statusRegisterData);
    // print("REG1B_CHARGER_STATUS_1: ");
    // println(statusRegisterData, HEX);
    // readReg(BQ25798_REG1D_CHARGER_STATUS_2, &statusRegisterData);
    // print("REG1B_CHARGER_STATUS_2: ");
    // println(statusRegisterData, HEX);
    // readReg(BQ25798_REG1E_CHARGER_STATUS_3, &statusRegisterData);
    // print("REG1B_CHARGER_STATUS_3: ");
    // println(statusRegisterData, HEX);
    // readReg(BQ25798_REG1F_CHARGER_STATUS_4, &statusRegisterData);
    // print("REG1B_CHARGER_STATUS_4: ");
    // println(statusRegisterData, HEX);
    // readReg(BQ25798_REG20_FAULT_STATUS_0, &statusRegisterData);
    // print("BQ25798_REG20_FAULT_STATUS_0: ");
    // println(statusRegisterData, HEX);
    // readReg(BQ25798_REG21_FAULT_STATUS_1, &statusRegisterData);
    // print("BQ25798_REG21_FAULT_STATUS_1: ");
    // println(statusRegisterData, HEX);

    // return;

    uint8_t statusRegisterData;
    readReg(BQ25798_REG1C_CHARGER_STATUS_1, &statusRegisterData);

    uint8_t newChargeStatus = (statusRegisterData >> 5) & 0x03;
    if (newChargeStatus != chargeStatus) {
        chargeStatus = newChargeStatus;
        switch (chargeStatus) {
        // case 0x00:
        //     println("Not charging");
        //     break;
        // case 0x01:
        //     println("Trickle charge");
        //     break;
        // case 0x02:
        //     println("Pre charge");
        //     break;
        // case 0x03:
        //     println("Fast charge (CC)");
        //     break;
        // case 0x04:
        //     println("Taper charge (CC)");
        //     break;
        // case 0x05:
        //     println("Reserved");
        //     break;
        // case 0x06:
        //     println("Top-off Timer Active Charging");
        //     break;
        // case 0x07:
        //     println("Charge Termination Done");
        //     break;
        default:
            print("Chg:0x");
            println(chargeStatus, HEX);
            break;
        }
    }

    // VBus Status
    uint8_t newVbusStatus = (statusRegisterData >> 1) & 0x0F;
    if (newVbusStatus != vbusStatus) {
        vbusStatus = newVbusStatus;
        switch (newVbusStatus) {
        case 0x00:
            println("No input");
            break;
        // case 0x01:
        //     println("USB SDP (500mA)");
        //     break;
        // case 0x02:
        //     println("USB CDP (1.5A)");
        //     break;
        // case 0x03:
        //     println("USB DCP (3.25A)");
        //     break;
        // case 0x04:
        //     println("Adjustable High Voltage DCP (HVDCP) (1.5A)");
        //     break;
        case 0x05:
            println("Unk adapter");
            break;
        // case 0x06:
        //     println("Non-Standard Adapter (1A/2A/2.1A/2.4A)");
        //     break;
        // case 0x07:
        //     println("In OTG mode");
        //     break;
        case 0x08:
            println("Bad adapter");
            break;
        // case 0x0B:
        //     println("Device directly powered from VBUS");
        // case 0x0C:
        //     println("Backup Mode");
        //     break;
        default:
            print("Vbus:0x");
            println(newVbusStatus, HEX);
            break;
        }
    }

    uint8_t faultRegVal;
    readReg(BQ25798_REG20_FAULT_STATUS_0, &faultRegVal);
    if (faultRegVal != 0x00) {
        print("Flt0:0x");
        println(faultRegVal, HEX);
    }
    readReg(BQ25798_REG21_FAULT_STATUS_1, &faultRegVal);
    if (faultRegVal != 0x00) {
        print("Flt1:0x");
        println(faultRegVal, HEX);
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

    // ---------- REG22 ----------
    if (flags[0] & VBUS_PRESENT_FLAG) {
        println("F:VBUS");
        vbusPresentFlag = true;
    }
    if (flags[0] & POWER_GOOD_FLAG)
        println("F:PwrGd");
    if (flags[0] & POORSRC_FLAG) {
        println("F:PoorSrc");
        poorSourceFlag = true;
    }
    if (flags[0] & ADC_DONE_FLAG)
        println("F:ADCdone");

    // ---------- REG23 ----------
    if (flags[1] & ICO_DONE_FLAG)
        println("F:ICOdone");
    if (flags[1] & ICO_FAIL_FLAG)
        println("F:ICOfail");
    if (flags[1] & IINDPM_FLAG)
        println("F:IINDPM");
    if (flags[1] & VINDPM_FLAG)
        println("F:VINDPM");
    if (flags[1] & TREG_FLAG)
        println("F:TREG");

    // ---------- REG24 ----------
    if (flags[2] & CHARGE_DONE_FLAG)
        println("F:ChgDone");
    if (flags[2] & RECHARGE_FLAG)
        println("F:Rechg");
    if (flags[2] & PRECHARGE_FLAG)
        println("F:Prechg");
    if (flags[2] & FASTCHARGE_FLAG)
        println("F:FastChg");
    if (flags[2] & TOPOFF_FLAG)
        println("F:TopOff");
    if (flags[2] & TERMINATION_FLAG)
        println("F:ChgTerm");

    // ---------- REG25 ----------
    if (flags[3] & TS_COLD_FLAG)
        println("F:TScold");
    if (flags[3] & TS_COOL_FLAG)
        println("F:TScool");
    if (flags[3] & TS_WARM_FLAG)
        println("F:TSwarm");
    if (flags[3] & TS_HOT_FLAG)
        println("F:TShot");

    // ---------- REG26 ----------
    if (flags[4] & VBUS_OVP_FLAG)
        println("E:VBUS OVP");
    if (flags[4] & IBUS_OCP_FLAG)
        println("E:IBUS OCP");
    if (flags[4] & IBAT_OCP_FLAG)
        println("E:IBAT OCP");
    if (flags[4] & VSYS_OVP_FLAG)
        println("E:VSYS OVP");
    if (flags[4] & VBAT_OVP_FLAG)
        println("E:VBAT OVP");
    if (flags[4] & VBAT_UVP_FLAG)
        println("E:VBAT UVP");

    // ---------- REG27 ----------
    if (flags[5] & TS_FAULT_FLAG)
        println("E:TS");
    if (flags[5] & TSHUT_FLAG)
        println("E:TShut");
    if (flags[5] & WATCHDOG_FLAG)
        println("E:WDT");
    if (flags[5] & SAFETY_TIMER_FLAG)
        println("E:SafeTimer");
}

void BQ25798::resetWatchdog() {
    // the host has to reset the watchdog timer by writing 1 to WD_RST bit
    // before the watchdog timer expire
    setBit(BQ25798_REG1B_CHARGER_STATUS_0, 5, true);
    uint8_t data;
    readReg(BQ25798_REG10_CHARGER_CONTROL_1, &data);
    // print("CHARGER_CONTROL_1: ");
    // println(data);
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
        println("Write err");
        return false;
    }
    // Return true if we don't need to read back to check the write.
    if (!check) {
        return true;
    }

    uint16_t uint16_read;

    if (!readWord(reg, &uint16_read)) {
        println("Read err");
        return false;
    }

    if (uint16_read != data) {
        print("Wr:0x");
        print(data, HEX);
        print("@0x");
        print(reg, HEX);
        print(" rd:0x");
        println(uint16_read, HEX);
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

uint16_t BQ25798::readTSADC() {

    // Disable all ADC apart from TS
    // writeWord(BQ25798_REG2F_ADC_FUNCTION_DISABLE_0, 0xFBFF);
    // Make a reading
    readADC();

    // Read TS as fraction of REGN
    uint16_t val;
    readWord(BQ25798_REG3F_TS_ADC, &val);
    float p = (val * 0.000976563f);
    print("P: ");
    println(p);

    float r1 = BQ25798_NTC_R1_OHMS;
    float r2 = BQ25798_NTC_R2_OHMS;

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
    float rt = (p * r1 * r2) / (r2 - p * (r1 + r2));

    float temp = ntcTempFromResistance(uint32_t(rt));
    print("Temp: ");
    println(temp);
    return val;
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

bool BQ25798::inHighInputImpedance() {
    uint8_t chargeStatus;
    readReg(BQ25798_REG0F_CHARGER_CONTROL_0, &chargeStatus);
    if ((chargeStatus & 0x04) == 0x04) {
        return true;
    }
    return false;
}
