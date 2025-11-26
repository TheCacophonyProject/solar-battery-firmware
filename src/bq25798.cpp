#include "bq25798.h"

// Datasheet: https://www.ti.com/lit/ds/symlink/bq25798.pdf

// Check that we can find the BQ76920 on the I2C bus.
// This is done by reading the Part Information Register (0x48).
// Returning false if it can not be found.
bool BQ25798::begin() {
    uint8_t regData;
    readReg(0x48, &regData);
    // From section 9.5.1.57
    // Check that Device Part Number is 0x03
    if ((regData & 0x38) != 0x18) {
        return false;
    }

    init();

    //setInputAndChargeLimits();
    checkStatus();
    return true;
}

void BQ25798::init() {
    Serial.println("Running initialization");
    
    Serial.println("Resetting registers");
    setBit(0x09, 6, true);
    
    Serial.println("Writing to registers");
    
    // 0x00 REG00_Minimal_System_Voltage -- Need to program
    if (!writeReg(0x00, 10)) {
        Serial.println("Failed to write minimum system voltage!!!!");
    }

    // 0x01 REG01_Charge_Voltage_Limit -- Need to program
    if (!writeWord(0x01, uint16_t(1230), true)) {
        Serial.println("Failed to write charge voltage!!!!");
    }

    // 0x03 REG03_Charge_Current_Limit -- Need to program
    if (!writeWord(0x03, uint16_t(100), true)) {
        Serial.println("Failed to write charge current limit!!!!");
    }

    // 0x05 REG05_Input_Voltage_Limit -- Need to program
    if (!writeReg(0x05, 220)) {
        Serial.println("Failed to write input voltage limit!!!!");
    }

    // 0x06 REG06_Input_Current_Limit -- Need to program
    if (!writeWord(0x06, uint16_t(200), true)) {
        Serial.println("Failed to write input current limit!!!!");
    }

    // 0x08 REG08_Precharge_Control -- Leave as default

    // 0x09 REG09_Termination_Control -- Need to program
    if (!writeReg(0x09, B00000101)) {
        Serial.println("Failed to write termination control!!!!");
    }

    // 0x0A REG0A_Re-charge_Control R -- Need to program
    // Check section 9.5.1.8 for details on setting register.
    // We charge up to 4.1V per cell, and should start re-charging at 3.95V so that
    // gives a voltage of 0.45V for the 3S cells. 
    // With the Fixed Offset : 50mV and Bit Step Size : 50mV 
    // that means a value of 8 is needed (for the last 4 bits in this register)
    if (!writeReg(0x0A, B10101000)) {
        Serial.println("Failed to write re-charge control register!!!!");
    }

    // 0x0B REG0B_VOTG_regulation -- Leave as default

    // 0x0D REG0D_IOTG_regulation -- Leave as default

    // 0x0E REG0E_Timer_Control -- Leave as default

    // 0x0F REG0F_Charger_Control_0 -- Leave as default

    // 0x10 REG10_Charger_Control_1 -- Need to program
    
    uint8_t regData;
    readReg(0x10, &regData);
    Serial.print("REG 0x10: ");
    Serial.println(regData, HEX);
    if (!writeReg(0x10, 0x85)) {
        Serial.println("Failed to write over voltage protection and WDT timeout control!!!!");
    }


    // 0x11 REG11_Charger_Control_2 -- Leave as default



    // 0x12 REG12_Charger_Control_3 -- Need to program PFM?
    if (!writeReg(0x12, 0x00)) {
        Serial.println("Failed to write Charger Control 3!!!!");
    }

    // 0x13 REG13_Charger_Control_4 -- Need to program
    if (!writeReg(0x13, 0b11100000)) {
        Serial.println("Failed to write Charger Control 4!!!!");
    }
    
    // 0x14 REG14_Charger_Control_5 -- Need to program
    if (!writeReg(0x14, 0x16)) {
        Serial.println("Failed to write Charger Control 5!!!!");
    }

    // 0x15 REG15_MPPT_Control -- Need to program
    if (!writeReg(0x15, 0xA8)) {
        Serial.println("Failed to write MPPT Control!!!!");
    }

    // 0x16 REG16_Temperature_Control -- Need to program
    if (!writeReg(0x16, 0b01110000)) {
        Serial.println("Failed to write Temperature Control!!!!");
    }

    // 0x17 REG17_NTC_Control_0 -- Need to program
    if (!writeReg(0x17, 0b00000010)) {
        Serial.println("Failed to write NTC Control 0!!!!");
    }

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

    // 0x29 REG29_Charger_Mask_1 -- Leave as default.

    // 0x2A REG2A_Charger_Mask_2 -- Leave as default.

    // 0x2B REG2B_Charger_Mask_3 -- Leave as default.

    // 0x2C REG2C_FAULT_Mask_0 -- Leave as default.

    // 0x2D REG2D_FAULT_Mask_1 -- Leave as default.



    // 0x2E REG2E_ADC_Control -- Need to program.
    /*
    if (!writeReg(0x2E, 0b10100000)) {
        Serial.println("Failed to write ADC Control!!!!");
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

//void BQ25798::ReadChargeRate()

void BQ25798::checkStatus() {
    uint8_t statusRegisterData;
    readReg(0x1C, &statusRegisterData);
    
    uint8_t newChargeStatus = (statusRegisterData >> 5) & 0x03;
    if (newChargeStatus != chargeStatus) {
        chargeStatus = newChargeStatus;
        switch (chargeStatus) {
        case 0x00:
            Serial.println("Not charging");
            break;
        case 0x01:
            Serial.println("Trickle charge");
            break;
        case 0x02:
            Serial.println("Pre charge");
            break;
        case 0x03:
            Serial.println("Fast charge (CC)");
            break;
        case 0x04:
            Serial.println("Taper charge (CC)");
            break;
        case 0x05:
            Serial.println("Reserved");
            break;
        case 0x06:
            Serial.println("Top-off Timer Active Charging");
            break;
        case 0x07:
            Serial.println("Charge Termination Done");
            break;
        default:
            Serial.println("Unknown charge status");
            break;
        }

    }
    
    /*
    0h: No Input or BHOT or BCOLD in OTG mode
    1h: USB SDP (500mA)
    2h: USB CDP (1.5A)
    3h: USB DCP (3.25A)
    4h: Adjustable High Voltage DCP (HVDCP) (1.5A)
    5h: Unknown adaptor (3A)
    6h: Non-Standard Adapter (1A/2A/2.1A/2.4A)
    7h: In OTG mode
    8h: Not qualified adaptor
    9h: Reserved
    Ah: Reserved
    Bh: Device directly powered from VBUS
    Ch: Backup Mode
    Dh: Reserved
    Eh: Reserved
    Fh: Reserved
    */
    // VBus Status
    uint8_t newVbusStatus = (statusRegisterData >> 1) & 0x0F;
    if (newVbusStatus != vbusStatus) {
        vbusStatus = newVbusStatus;
        switch (newVbusStatus) {
        case 0x00:
            Serial.println("No Input or BHOT or BCOLD in OTG mode");
            break;
        case 0x01:
            Serial.println("USB SDP (500mA)");
            break;
        case 0x02:
            Serial.println("USB CDP (1.5A)");
            break;
        case 0x03:
            Serial.println("USB DCP (3.25A)");
            break;
        case 0x04:
            Serial.println("Adjustable High Voltage DCP (HVDCP) (1.5A)");
            break;
        case 0x05:
            Serial.println("Unknown adaptor (3A)");
            break;
        case 0x06:
            Serial.println("Non-Standard Adapter (1A/2A/2.1A/2.4A)");
            break;
        case 0x07:
            Serial.println("In OTG mode");
            break;
        case 0x08:
            Serial.println("Not qualified adaptor");
            break;
        case 0x0B:
            Serial.println("Device directly powered from VBUS");
        case 0x0C:
            Serial.println("Backup Mode");
            break;
        default:
            Serial.println("Unknown VBus status");
            break;
        }
    }
    

    uint8_t faultRegVal;
    readReg(0x20, &faultRegVal);
    if (faultRegVal != 0x00) {
        Serial.print("Fault register status_0 value: 0x");
        Serial.println(faultRegVal, HEX);
    }
    readReg(0x21, &faultRegVal);
    if (faultRegVal != 0x00) {
        Serial.print("Fault register status_1 value: 0x");
        Serial.println(faultRegVal, HEX);
    }
    
    // TODO: just enable the ADC when needed so to not use as much power

    //readReg(0x2E, &faultRegVal);
    //Serial.print("ADC reg value: 0x");
    //Serial.println(faultRegVal, HEX);

    // Enable ADC
    /*
    setBit(0x2E, 7, true);
    
    uint8_t adcData[2];
    readBlock(0x33, adcData, 2);
    uint16_t adcRaw = (uint16_t(adcData[0]) << 8) + adcData[1];
    int16_t newChargingCurrent = (int16_t)adcRaw;
    Serial.print("Charging current: ");
    Serial.println(newChargingCurrent);
    //int16_t signedVal = 
    if (abs(chargingCurrent - newChargingCurrent) > 10) {
        chargingCurrent = newChargingCurrent;
        Serial.print("Charging current: ");
        Serial.print(newChargingCurrent);
        Serial.println(" mA");
    }
        */
    
    
    // Check WDT



    //uint8_t wdtReg;
    //readReg(0x10, &wdtReg);
    //Serial.println(wdtReg, HEX);

    //setBit(0x10, 4, true);
    //readReg(0x10, &wdtReg);
    //Serial.println(wdtReg, HEX);

    // The flags are cleared once they are read.
    uint8_t flags;
    readReg(0x22, &flags);
    if (flags != 0x00) {
        Serial.print("Charger flags 0: 0x");
        Serial.println(flags, HEX);
    }
    readReg(0x23, &flags);
    if (flags != 0x00) {
        Serial.print("Charger flags 1: 0x");
        Serial.println(flags, HEX);
    }
    readReg(0x24, &flags);
    if (flags != 0x00) {
        Serial.print("Charger flags 2: 0x");
        Serial.println(flags, HEX);
    }
    readReg(0x25, &flags);
    if (flags != 0x00) {
        Serial.print("Charger flags 3: 0x");
        Serial.println(flags, HEX);
    }

    readReg(0x26, &flags);
    if (flags != 0x00) {
        Serial.print("Fault flags 0: 0x");
        Serial.println(flags, HEX);
    }

    readReg(0x27, &flags);
    if (flags != 0x00) {
        Serial.print("Fault flags 1: 0x");
        Serial.println(flags, HEX);
    }


    if ((flags & 0x2000) == 0x2000) {
        Serial.println("WDT flag set, triggering charger init sequence");
        init();
    }

    // Checking the status flag
    readReg(0x1F, &flags);
    if (flags != 0x00) {
        Serial.print("Charger status 4: 0x");
        Serial.println(flags, HEX);
    }

    // Reset WDT
    setBit(0x10, 3, true);

    /*
    Serial.println("reg 0x00");
    begin();
    readReg(0x00, &wdtReg);
    Serial.println(wdtReg, HEX);
    */
}

void BQ25798::setChargeVoltageLimit() {
    Serial.println("Checking that charge voltage is set to 12.3V");

    uint8_t readRegData[2];
    if (!readBlock(0x01, readRegData, 2)) {
        Serial.println("Failed to read charge voltage");
        return;
    }
    uint16_t chargeVoltage = (uint16_t(readRegData[0]) << 8) + readRegData[1];
    Serial.print("Current charge voltage: ");
    Serial.println(chargeVoltage);
    
    // Target charge voltage is 4.1V per cell, so 12.3V for the pack.
    Serial.println("Setting charge voltage to 12.3V");
    // register has a bit step size of 10mV
    uint16_t target = 1230;
    uint8_t writeRegData[] = {(target >> 8) & 0x07, target & 0xFF};
    if (!writeBlock(0x01, writeRegData, 2)) {
        Serial.println("Failed to write charge voltage");
        return;
    }

    readRegData[2];
    if (!readBlock(0x01, readRegData, 2)) {
        Serial.println("Failed to read charge voltage");
        return;
    }
    chargeVoltage = (uint16_t(readRegData[0]) << 8) + readRegData[1];
    if (chargeVoltage != target) {
        Serial.println("Failed to set charge voltage");
        Serial.println("Actual charge voltage: ");
        Serial.println(chargeVoltage);
        return;
    }
}

void BQ25798::sourceRetry() {
    // From 9.3.4.2 Poor Source Qualification:
	// the host may set EN_HIZ = 0 to force an immediate retry of the poor source qualification.
    setBit(0x0F, 2, false);
}

void BQ25798::setVOCRate() {
    // From 9.3.4.2 Poor Source Qualification:
    // the host may set EN_HIZ = 0 to force an immediate retry of the poor source qualification.
    setBits(0x15, 0, 1, 2);
}

void BQ25798::enableMPPT() {
    uint8_t mpptRegVal;
    readReg(0x1B, &mpptRegVal);
    //Serial.println(mpptRegVal, HEX);
    if ((mpptRegVal & 0x08) != 0x08) {
        //Serial.println("Not power good, not enabling MPPT");
        return;
    }

    readReg(0x15, &mpptRegVal);
    bool mpptActive = mpptRegVal & 0x01;
    setBit(0x15, 0, true);
    if (!mpptActive) {
        Serial.println("Enabling MPPT");    
    }
}

// TODO
//void BQ25798::SetBatteryChargeCurrentLimit() {}

// TODO
//void BQ25798::SetBatteryVoltageLimit() {}

void BQ25798::setInputAndChargeLimits() {
    Serial.println("Setting minimum system voltage to 5V");
    // Fixed Offset : 2500mV
    // Bit Step Size : 250mV
    if (!writeReg(0x00, 10)) {
        Serial.println("Failed to write minimum system voltage!!!!");
    }

    Serial.println("Setting charge voltage to 12.3V");
    // register has a bit step size of 10mV
    if (!writeWord(0x01, uint16_t(1230), true)) {
        Serial.println("Failed to write charge voltage!!!!");
    }

    Serial.println("Setting charge current limit to 1000mA");
    // register has a bit step size of 10mA
    if (!writeWord(0x03, uint16_t(100), true)) {
        Serial.println("Failed to write charge current limit!!!!");
    }
    
    Serial.println("Setting input voltage limit to 22V");
    // register has a bit step size of 100mV
    if (!writeReg(0x05, 220)) {
        Serial.println("Failed to write input voltage limit!!!!");
    }

    Serial.println("Setting input current limit to 2000mA");
    // register has a bit step size of 10mA
    if (!writeWord(0x06, uint16_t(200), true)) {
        Serial.println("Failed to write input current limit!!!!");
    }

    Serial.println("Setting precharge limits");
    // Check section 9.5.1.6 for details on setting register.
    if (!writeReg(0x08, B11000011)) {
        Serial.println("Failed to write precharge limits!!!!");
    }

    Serial.println("Setting termination control");
    // Check section 9.5.1.7 for details on setting register.
    if (!writeReg(0x09, B00000101)) {
        Serial.println("Failed to write termination control!!!!");
    }

    Serial.println("Setting re-charge control register");
    // Check section 9.5.1.8 for details on setting register.
    // We charge up to 4.1V per cell, and should start re-charging at 3.95V so that
    // gives a voltage of 0.45V for the 3S cells. 
    // With the Fixed Offset : 50mV and Bit Step Size : 50mV 
    // that means a value of 8 is needed (for the last 4 bits in this register)
    if (!writeReg(0x0A, B10101000)) {
        Serial.println("Failed to write re-charge control register!!!!");
    }

    // Reg 0x0B, just for the OTG mode, which we don't use.

    Serial.println("Setting precharge timer");
    // This register will set the precharge timer to 2 hours. It also sets the OTG current limit but we just leave that as default as we are not using that.
    if (!writeReg(0x0C, 0x4B)) {
        Serial.println("Failed to write precharge timer!!!!");
    }

    Serial.println("Set timer control register");
    // We are just leaving these as the default values
    if (!writeReg(0x0D, 0x7D)) {
        Serial.println("Failed to write timer control register!!!!");
    }

    Serial.println("Set timer control register");
    // Leaving this as the default value
    if (!writeReg(0x0E, 0x3D)) {
        Serial.println("Failed to write timer control register!!!!");
    }

    Serial.println("Set Charge control register 0");
    // Leaving this as the default value
    if (!writeReg(0x0F, 0xA2)) {
        Serial.println("Failed to write charge control register 0!!!!");
    }

    Serial.println("Setting over voltage protection and WDT timeout control");
    // TODO look into what VBUS_BACKUP_1 is
    if (!writeReg(0x10, 0x85)) {
        Serial.println("Failed to write over voltage protection and WDT timeout control!!!!");
    }
}

// TODO check flags 9.1.5.30

// TODO Watch dog timer, just need to trigger that ever ? seconds.

// Trigger checks from the INT pin

bool BQ25798::writeWord(uint8_t reg, uint16_t data, bool check) {
    uint8_t out[] = {
        static_cast<uint8_t>(data >> 8), 
        static_cast<uint8_t>(data & 0xFF)
    };
    if (!writeBlock(reg, out, 2)) {
        Serial.println("Error writing word");
        return false;
    }
    // Return true if we don't need to read back to check the write.
    if (!check) {
        return true;
    }

    uint16_t uint16_read;

    if (!readWord(reg, &uint16_read)) {
        Serial.println("Error reading back word");
        return false;
    }

    if (uint16_read != data) {
        Serial.print("Error: When writing 0x"); 
        Serial.print(data, HEX); 
        Serial.print(" to 0x"); 
        Serial.print(reg, HEX); 
        Serial.print(", read back 0x"); 
        Serial.println(uint16_read, HEX);
        return false;
    }
    return true;
}

bool BQ25798::readWord(uint8_t reg, uint16_t *data) {
    uint8_t in[2];
    if (!readBlock(reg, in, 2)) {
        return false;
    }
    *data = (
        static_cast<uint16_t>(in[0]) << 8 |
        static_cast<uint16_t>(in[1])
    );
    return true;
}

// TODO
//void BQ25798::SetInputVoltageLimit() {}

// TODO
//void BQ25798::SetInputCurrentLimit() {}

// TODO
//void BQ25798::ReadChargingPower() {}

bool BQ25798::writeReg(uint8_t reg, uint8_t data) {
    return i2c_.writeReg(BQ25798_DEFAULT_ADDR, reg, data);
}

bool BQ25798::readReg(uint8_t reg, uint8_t *data) {
    return i2c_.readReg(BQ25798_DEFAULT_ADDR, reg, data);
}

bool BQ25798::setBit(uint8_t reg, uint8_t bit, bool value) {
    uint8_t regValue;
    readReg(reg, &regValue);

    if (value) {
        regValue |= (1 << bit);
    } else {
        regValue &= ~(1 << bit);
    }

    return writeReg(reg, regValue);
}

bool BQ25798::setBits(uint8_t reg, uint8_t val, uint8_t offset, uint8_t len) {
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

bool BQ25798::readBlock(uint8_t reg, uint8_t data[], size_t len) {
    return i2c_.read(BQ25798_DEFAULT_ADDR, reg, data, len);
}

bool BQ25798::writeBlock(uint8_t reg, uint8_t data[], size_t len) {
    return i2c_.write(BQ25798_DEFAULT_ADDR, reg, data, len);
}