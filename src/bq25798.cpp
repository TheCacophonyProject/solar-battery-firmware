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
    disable();

    // Make sure we are talking to the right chip.
    // From section 9.5.1.57
    // Check that Device Part Number is 0x03
    uint8_t regData;
    readReg(BQ25798_REG48_PART_INFORMATION, &regData);
    if ((regData & 0x38) != 0x18) {
        println("Failed to find BQ25798, wrong part number");
        return false;
    }

    // Read from the CELL register (REG0x0A[7:6])
    
    readReg(BQ25798_REG0A_RECHARGE_CONTROL, &regData);
    if ((regData & 0xC0) != 0x80) {
        println("Wrong cell count for BQ25798");
        return false;
    }

    // Read frequency from PWM_FREQ
    readReg(BQ25798_REG13_CHARGER_CONTROL_4, &regData);
    if ((regData & 0x20) != 0x20) {
        println("Wrong PWM frequency for BQ25798");
        return false;
    }

    init();

    checkStatus();
    return true;
}

void BQ25798::dumpTempRelated() {

    uint8_t b[8] = {0};
    bool ok = i2c_.read(BQ25798_DEFAULT_ADDR, 0x1E, b, 8); // reads 0x1E,0x1F,0x20
    print("burst ok="); println(ok);
    print("REG1E="); print(b[0], HEX);
    print(" REG1F="); print(b[1], HEX);
    print(" REG20="); print(b[2], HEX);
    print(" REG21="); print(b[3], HEX);
    print(" REG22="); print(b[4], HEX);
    print(" REG23="); print(b[5], HEX);
    print(" REG24="); print(b[6], HEX);
    print(" REG25="); println(b[7], HEX);

    /*
    //uint8_t b[8] = {0};
    ok = i2c_.read(BQ25798_DEFAULT_ADDR, 0x1E, b, 8); // reads 0x1E,0x1F,0x20
    print("burst ok="); println(ok);
    print("REG1E="); print(b[0], HEX);
    print(" REG1F="); print(b[1], HEX);
    print(" REG20="); print(b[2], HEX);
    print(" REG21="); print(b[3], HEX);
    print(" REG22="); print(b[4], HEX);
    print(" REG23="); print(b[5], HEX);
    print(" REG24="); print(b[6], HEX);
    print(" REG25="); println(b[7], HEX);
    */

    /*
    uint8_t reg1f=0, reg18=0, reg25=0, reg20=0, reg1e=0;

    readReg(BQ25798_REG18_NTC_CONTROL_1, &reg18);
    readReg(BQ25798_REG1F_CHARGER_STATUS_4, &reg1f);
    readReg(BQ25798_REG25_CHARGER_FLAG_3, &reg25);
    readReg(BQ25798_REG20_FAULT_STATUS_0, &reg20);
    readReg(BQ25798_REG1E_CHARGER_STATUS_3, &reg1e);

    print("REG18=0x");
    print(reg18, HEX);
    print(", REG1F=0x");
    print(reg1f, HEX);
    print(", REG25=0x");
    print(reg25, HEX);
    print(", REG20=0x");
    print(reg20, HEX);
    print(", REG1E=0x");
    print(reg1e, HEX);
    println("");
    
    print("TS: ");
    if (reg1f & BQ25798_TS_HOT_STAT) {
        print("HOT ");
    }
    if (reg1f & BQ25798_TS_COLD_STAT) {
        print("COLD ");
    }
    if (reg1f & BQ25798_TS_WARM_STAT) {
        print("WARM ");
    }
    if (reg1f & BQ25798_TS_COOL_STAT) {
        print("COOL ");
    }
    println("");
    */

}

void BQ25798::enable() {
    digitalWrite(enablePin, LOW);
}

void BQ25798::disable() {
    digitalWrite(enablePin, HIGH);
}

void BQ25798::checkSourceAndMPPT() {
    // If the source is found to be bad then it will by default not retry for 7 minutes.
    // For our setup in low light solar this will be too long (a cloud could be passing causing a bad source adn we could recover a lot sooner than 7 minutes).
    
    // TODO track what state we are in and log when there is a change.

    // Check if we have an input source.
    uint8_t chargerStatus0;
    readReg(BQ25798_REG1B_CHARGER_STATUS_0, &chargerStatus0);
    if (!(chargerStatus0 & BQ25798_VBUS_PRESENT_STAT)) {
        //println("Source not present.");
        return;
    }

    // Check if we need to disable high impedance mode.
    uint8_t chargerControl0;
    readReg(BQ25798_REG0F_CHARGER_CONTROL_0, &chargerControl0);
    if (chargerControl0 & BQ25798_EN_HIZ) {
        // Check source again.
        // From 9.3.4.2 Poor Source Qualification:
        // the host may set EN_HIZ = 0 to force an immediate retry of the poor source qualification.
        setBit(BQ25798_REG0F_CHARGER_CONTROL_0, 2, false);
        println("Disabling high impedance mode");
    }

    // Check if MPPT is enabled.
    uint8_t mpptControl;
    readReg(BQ25798_REG15_MPPT_CONTROL, &mpptControl);
    if (!(mpptControl & BQ25798_EN_MPPT)) {
        // MPPT is disabled, enable it.
        println("Enabling MPPT");
        setBit(BQ25798_REG15_MPPT_CONTROL, 0, true);    
    }
}

void BQ25798::init() {
    println("Running initialization");
    
    // From 9.5.1.7: Reset registers to default values and reset timer by writing 1 to bit 6 on BQ25798_REG09_TERMINATION_CONTROL
    println("Resetting registers to initial values.");
    setBit(BQ25798_REG09_TERMINATION_CONTROL, 6, true);
    
    println("Writing to registers");
    
    // 9.5.1.1: Write to register BQ25798_REG00_MIN_SYS_VOLTAGE to set the minimum system voltage.
    // 0x00 REG00_Minimal_System_Voltage -- Need to program
    writeReg(BQ25798_REG00_MIN_SYS_VOLTAGE, 10);
    
    // 0x01 REG01_Charge_Voltage_Limit -- Need to program
    writeWord(BQ25798_REG01_CHARGE_VOLTAGE_LIMIT, uint16_t(1230), true);

    // 0x03 REG03_Charge_Current_Limit -- Need to program
    writeWord(BQ25798_REG03_CHARGE_CURRENT_LIMIT, uint16_t(100), true);

    // 0x05 REG05_Input_Voltage_Limit -- Need to program
    writeReg(BQ25798_REG05_INPUT_VOLTAGE_LIMIT, 220);

    // 0x06 REG06_Input_Current_Limit -- Need to program
    writeWord(BQ25798_REG06_INPUT_CURRENT_LIMIT, uint16_t(200), true);

    // 0x08 REG08_Precharge_Control -- Leave as default

    // 0x09 REG09_Termination_Control -- Need to program
    writeReg(BQ25798_REG09_TERMINATION_CONTROL, B00000101);

    // 0x0A REG0A_Re-charge_Control R -- Need to program
    // Check section 9.5.1.8 for details on setting register.
    // We charge up to 4.1V per cell, and should start re-charging at 3.95V so that
    // gives a voltage of 0.45V for the 3S cells. 
    // With the Fixed Offset : 50mV and Bit Step Size : 50mV 
    // that means a value of 8 is needed (for the last 4 bits in this register)
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

    // 0x29 REG29_Charger_Mask_1 -- Leave as default.

    // 0x2A REG2A_Charger_Mask_2 -- Leave as default.
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

//void BQ25798::ReadChargeRate()

uint8_t BQ25798::getReg(bq25798_reg_t reg) {
    uint8_t data;
    readReg(reg, &data);
    return data;
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
    
    uint8_t newChargeStatus = (statusRegisterData >> 5) & 0x03;
    if (newChargeStatus != chargeStatus) {
        chargeStatus = newChargeStatus;
        /*
        switch (chargeStatus) {
        case 0x00:
            println("Not charging");
            break;
        case 0x01:
            println("Trickle charge");
            break;
        case 0x02:
            println("Pre charge");
            break;
        case 0x03:
            println("Fast charge (CC)");
            break;
        case 0x04:
            println("Taper charge (CC)");
            break;
        case 0x05:
            println("Reserved");
            break;
        case 0x06:
            println("Top-off Timer Active Charging");
            break;
        case 0x07:
            println("Charge Termination Done");
            break;
        default:
            println("Unknown charge status");
            break;
        }
            */

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
            println("No Input or BHOT or BCOLD in OTG mode");
            break;
        case 0x01:
            println("USB SDP (500mA)");
            break;
        case 0x02:
            println("USB CDP (1.5A)");
            break;
        case 0x03:
            println("USB DCP (3.25A)");
            break;
        case 0x04:
            println("Adjustable High Voltage DCP (HVDCP) (1.5A)");
            break;
        case 0x05:
            println("Unknown adaptor (3A)");
            break;
        case 0x06:
            println("Non-Standard Adapter (1A/2A/2.1A/2.4A)");
            break;
        case 0x07:
            println("In OTG mode");
            break;
        case 0x08:
            println("Not qualified adaptor");
            break;
        case 0x0B:
            println("Device directly powered from VBUS");
        case 0x0C:
            println("Backup Mode");
            break;
        default:
            println("Unknown VBus status");
            break;
        }
    }
    

    uint8_t faultRegVal;
    readReg(BQ25798_REG20_FAULT_STATUS_0, &faultRegVal);
    if (faultRegVal != 0x00) {
        print("Fault register status_0 value: 0x");
        println(faultRegVal, HEX);
    }
    readReg(BQ25798_REG21_FAULT_STATUS_1, &faultRegVal);
    if (faultRegVal != 0x00) {
        print("Fault register status_1 value: 0x");
        println(faultRegVal, HEX);
    }
    
    // TODO: just enable the ADC when needed so to not use as much power

    //readReg(0x2E, &faultRegVal);
    //print("ADC reg value: 0x");
    //println(faultRegVal, HEX);

    // Enable ADC
    /*
    setBit(0x2E, 7, true);
    
    uint8_t adcData[2];
    readBlock(0x33, adcData, 2);
    uint16_t adcRaw = (uint16_t(adcData[0]) << 8) + adcData[1];
    int16_t newChargingCurrent = (int16_t)adcRaw;
    print("Charging current: ");
    println(newChargingCurrent);
    //int16_t signedVal = 
    if (abs(chargingCurrent - newChargingCurrent) > 10) {
        chargingCurrent = newChargingCurrent;
        print("Charging current: ");
        print(newChargingCurrent);
        println(" mA");
    }
        */
    
    
    // Check WDT



    //uint8_t wdtReg;
    //readReg(0x10, &wdtReg);
    //println(wdtReg, HEX);

    //setBit(0x10, 4, true);
    //readReg(0x10, &wdtReg);
    //println(wdtReg, HEX);

    // The flags are cleared once they are read.
    dumpFlags();
    /*

    uint8_t flags[6];
    readBlock(BQ25798_REG22_CHARGER_FLAG_0, flags, 6);
    
    if (flags[0] & VBUS_PRESENT_FLAG) {
        println("VBUS present");
    }
    if (flags[0] & AC1_PRESENT_FLAG) {
        println("Charger present");
    } 

    
    
    readReg(BQ25798_REG22_CHARGER_FLAG_0, &flags);
    if (flags != 0x00) {
        print("Charger flags 0: 0x");
        println(flags, HEX);
    }
    readReg(BQ25798_REG23_CHARGER_FLAG_1, &flags);
    if (flags != 0x00) {
        print("Charger flags 1: 0x");
        println(flags, HEX);
    }
    readReg(BQ25798_REG24_CHARGER_FLAG_2, &flags);
    if (flags != 0x00) {
        print("Charger flags 2: 0x");
        println(flags, HEX);
    }
    readReg(BQ25798_REG25_CHARGER_FLAG_3, &flags);
    if (flags != 0x00) {
        print("Charger flags 3: 0x");
        println(flags, HEX);
    }

    readReg(BQ25798_REG26_FAULT_FLAG_0, &flags);
    if (flags != 0x00) {
        print("Fault flags 0: 0x");
        println(flags, HEX);
    }

    readReg(BQ25798_REG27_FAULT_FLAG_1, &flags);
    if (flags != 0x00) {
        print("Fault flags 1: 0x");
        println(flags, HEX);
    }


    if ((flags & 0x2000) == 0x2000) {
        println("WDT flag set, triggering charger init sequence");
        init();
    }
        */

    uint8_t flags;
    // Checking the status
    readReg(BQ25798_REG1F_CHARGER_STATUS_4, &flags);
    if (flags != 0x00) {
        print("Charger status 4: 0x");
        println(flags, HEX);
    }

    // Reset WDT
    setBit(BQ25798_REG10_CHARGER_CONTROL_1, 3, true);

    /*
    println("reg 0x00");
    begin();
    readReg(0x00, &wdtReg);
    println(wdtReg, HEX);
    */
}

void BQ25798::dumpFlags() {
    uint8_t flags[6] = {0};

    // REG22 .. REG27
    readBlock(BQ25798_REG22_CHARGER_FLAG_0, flags, 6);

    // ---------- REG22 ----------
    if (flags[0] & VBUS_PRESENT_FLAG)   println("FLAG: VBUS present");
    if (flags[0] & VAC1_PRESENT_FLAG)   println("FLAG: VAC1 present");
    if (flags[0] & VAC2_PRESENT_FLAG)   println("FLAG: VAC2 present");
    if (flags[0] & POWER_GOOD_FLAG)     println("FLAG: Power good");
    if (flags[0] & POORSRC_FLAG)        println("FLAG: Poor source detected");
    if (flags[0] & ADC_DONE_FLAG)       println("FLAG: ADC conversion done");

    // ---------- REG23 ----------
    if (flags[1] & ICO_DONE_FLAG)       println("FLAG: ICO done");
    if (flags[1] & ICO_FAIL_FLAG)       println("FLAG: ICO failed");
    if (flags[1] & IINDPM_FLAG)         println("FLAG: Input current limited");
    if (flags[1] & VINDPM_FLAG)         println("FLAG: Input voltage limited");
    if (flags[1] & TREG_FLAG)           println("FLAG: Thermal regulation active");

    // ---------- REG24 ----------
    if (flags[2] & CHARGE_DONE_FLAG)    println("FLAG: Charge done");
    if (flags[2] & RECHARGE_FLAG)       println("FLAG: Re-charge");
    if (flags[2] & PRECHARGE_FLAG)      println("FLAG: Pre-charge");
    if (flags[2] & FASTCHARGE_FLAG)     println("FLAG: Fast charge");
    if (flags[2] & TOPOFF_FLAG)         println("FLAG: Top-off");
    if (flags[2] & TERMINATION_FLAG)    println("FLAG: Charge termination");

    // ---------- REG25 ----------
    if (flags[3] & TS_COLD_FLAG)        println("FLAG: TS cold");
    if (flags[3] & TS_COOL_FLAG)        println("FLAG: TS cool");
    if (flags[3] & TS_WARM_FLAG)        println("FLAG: TS warm");
    if (flags[3] & TS_HOT_FLAG)         println("FLAG: TS hot");

    // ---------- REG26 ----------
    if (flags[4] & VBUS_OVP_FLAG)       println("FAULT: VBUS over-voltage");
    if (flags[4] & IBUS_OCP_FLAG)       println("FAULT: IBUS over-current");
    if (flags[4] & IBAT_OCP_FLAG)       println("FAULT: IBAT over-current");
    if (flags[4] & VSYS_OVP_FLAG)       println("FAULT: VSYS over-voltage");
    if (flags[4] & VBAT_OVP_FLAG)       println("FAULT: VBAT over-voltage");
    if (flags[4] & VBAT_UVP_FLAG)       println("FAULT: VBAT under-voltage");

    // ---------- REG27 ----------
    if (flags[5] & TS_FAULT_FLAG)       println("FAULT: TS fault");
    if (flags[5] & TSHUT_FLAG)          println("FAULT: Thermal shutdown");
    if (flags[5] & WATCHDOG_FLAG)       println("FAULT: Watchdog");
    if (flags[5] & SAFETY_TIMER_FLAG)   println("FAULT: Safety timer expired");
}

void BQ25798::setChargeVoltageLimit() {
    println("Checking that charge voltage is set to 12.3V");

    uint8_t readRegData[2];
    if (!readBlock(BQ25798_REG01_CHARGE_VOLTAGE_LIMIT, readRegData, 2)) {
        println("Failed to read charge voltage");
        return;
    }
    uint16_t chargeVoltage = (uint16_t(readRegData[0]) << 8) + readRegData[1];
    print("Current charge voltage: ");
    println(chargeVoltage);
    
    // Target charge voltage is 4.1V per cell, so 12.3V for the pack.
    println("Setting charge voltage to 12.3V");
    // register has a bit step size of 10mV
    uint16_t target = 1230;
    uint8_t writeRegData[] = {(target >> 8) & 0x07, target & 0xFF};
    if (!writeBlock(BQ25798_REG01_CHARGE_VOLTAGE_LIMIT, writeRegData, 2)) {
        println("Failed to write charge voltage");
        return;
    }

    readRegData[2];
    if (!readBlock(BQ25798_REG01_CHARGE_VOLTAGE_LIMIT, readRegData, 2)) {
        println("Failed to read charge voltage");
        return;
    }
    chargeVoltage = (uint16_t(readRegData[0]) << 8) + readRegData[1];
    if (chargeVoltage != target) {
        println("Failed to set charge voltage");
        println("Actual charge voltage: ");
        println(chargeVoltage);
        return;
    }
}

void BQ25798::sourceRetry() {
    // From 9.3.4.2 Poor Source Qualification:
	// the host may set EN_HIZ = 0 to force an immediate retry of the poor source qualification.
    setBit(BQ25798_REG0F_CHARGER_CONTROL_0, 2, false);
}

void BQ25798::setVOCRate() {
    // From 9.3.4.2 Poor Source Qualification:
    // the host may set EN_HIZ = 0 to force an immediate retry of the poor source qualification.
    setBits(BQ25798_REG15_MPPT_CONTROL, 0, 1, 2);
}

void BQ25798::enableMPPT() {
    uint8_t mpptRegVal;
    readReg(BQ25798_REG1B_CHARGER_STATUS_0, &mpptRegVal);
    //println(mpptRegVal, HEX);
    if ((mpptRegVal & 0x08) != 0x08) {
        //println("Not power good, not enabling MPPT");
        return;
    }

    readReg(BQ25798_REG15_MPPT_CONTROL, &mpptRegVal);
    bool mpptActive = mpptRegVal & 0x01;
    setBit(BQ25798_REG15_MPPT_CONTROL, 0, true);
    if (!mpptActive) {
        println("Enabling MPPT");    
    }
}

// TODO
//void BQ25798::SetBatteryChargeCurrentLimit() {}

// TODO
//void BQ25798::SetBatteryVoltageLimit() {}

void BQ25798::setInputAndChargeLimits() {
    /*
    println("Setting minimum system voltage to 5V");
    // Fixed Offset : 2500mV
    // Bit Step Size : 250mV
    if (!writeReg(BQ25798_REG00_MIN_SYS_VOLTAGE, 10)) {
        println("Failed to write minimum system voltage!!!!");
    }

    println("Setting charge voltage to 12.3V");
    // register has a bit step size of 10mV
    if (!writeWord(BQ25798_REG01_CHARGE_VOLTAGE_LIMIT, uint16_t(1230), true)) {
        println("Failed to write charge voltage!!!!");
    }

    println("Setting charge current limit to 1000mA");
    // register has a bit step size of 10mA
    if (!writeWord(BQ25798_REG03_CHARGE_CURRENT_LIMIT, uint16_t(100), true)) {
        println("Failed to write charge current limit!!!!");
    }
    
    println("Setting input voltage limit to 22V");
    // register has a bit step size of 100mV
    if (!writeReg(BQ25798_REG05_INPUT_VOLTAGE_LIMIT, 220)) {
        println("Failed to write input voltage limit!!!!");
    }

    println("Setting input current limit to 2000mA");
    // register has a bit step size of 10mA
    if (!writeWord(BQ25798_REG06_INPUT_CURRENT_LIMIT, uint16_t(200), true)) {
        println("Failed to write input current limit!!!!");
    }

    println("Setting precharge limits");
    // Check section 9.5.1.6 for details on setting register.
    if (!writeReg(BQ25798_REG08_PRECHARGE_CONTROL, B11000011)) {
        println("Failed to write precharge limits!!!!");
    }

    println("Setting termination control");
    // Check section 9.5.1.7 for details on setting register.
    if (!writeReg(BQ25798_REG09_TERMINATION_CONTROL, B00000101)) {
        println("Failed to write termination control!!!!");
    }

    println("Setting re-charge control register");
    // Check section 9.5.1.8 for details on setting register.
    // We charge up to 4.1V per cell, and should start re-charging at 3.95V so that
    // gives a voltage of 0.45V for the 3S cells. 
    // With the Fixed Offset : 50mV and Bit Step Size : 50mV 
    // that means a value of 8 is needed (for the last 4 bits in this register)
    if (!writeReg(BQ25798_REG0A_RECHARGE_CONTROL, B10101000)) {
        println("Failed to write re-charge control register!!!!");
    }

    // Reg 0x0B, just for the OTG mode, which we don't use.

    println("Set timer control register");
    // We are just leaving these as the default values
    if (!writeReg(0x0D, 0x7D)) {
        println("Failed to write timer control register!!!!");
    }

    println("Set timer control register");
    // Leaving this as the default value
    if (!writeReg(0x0E, 0x3D)) {
        println("Failed to write timer control register!!!!");
    }

    println("Set Charge control register 0");
    // Leaving this as the default value
    if (!writeReg(0x0F, 0xA2)) {
        println("Failed to write charge control register 0!!!!");
    }

    println("Setting over voltage protection and WDT timeout control");
    // TODO look into what VBUS_BACKUP_1 is
    if (!writeReg(0x10, 0x85)) {
        println("Failed to write over voltage protection and WDT timeout control!!!!");
    }
        */
}

// TODO check flags 9.1.5.30

// TODO Watch dog timer, just need to trigger that ever ? seconds.

// Trigger checks from the INT pin

bool BQ25798::writeWord(bq25798_reg_t reg, uint16_t data, bool check) {
    uint8_t out[] = {
        static_cast<uint8_t>(data >> 8), 
        static_cast<uint8_t>(data & 0xFF)
    };
    if (!writeBlock(reg, out, 2)) {
        println("Error writing word");
        return false;
    }
    // Return true if we don't need to read back to check the write.
    if (!check) {
        return true;
    }

    uint16_t uint16_read;

    if (!readWord(reg, &uint16_read)) {
        println("Error reading back word");
        return false;
    }

    if (uint16_read != data) {
        print("Error: When writing 0x"); 
        print(data, HEX); 
        print(" to 0x"); 
        print(reg, HEX); 
        print(", read back 0x"); 
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
    //writeWord(BQ25798_REG2F_ADC_FUNCTION_DISABLE_0, 0xFBFF);
    // Make a reading
    readADC();

    // Read TS as fraction of REGN
    uint16_t val;
    readWord(BQ25798_REG3F_TS_ADC, &val);
    float p = (val * 0.000976563f);
    print("P: ");
    println(p);

    float r1 = 5000;
    float r2 = 30000;

    // R2 is in parallel with Rt
    // R1 is in series with Rt and R2
    // This is the equation for finding Rt from the two resistors
    // and the percentage of the voltage divider.
    float rt = (p*r1*r2)/(r2-p*(r1+r2));
    //print("RT: ");
    //println(rt);

    float temp = ntc_temp_from_resistance(uint32_t(rt));
    print("Temp: ");
    println(temp);
    return val;
}

bool BQ25798::writeReg(bq25798_reg_t reg, uint8_t data) {
    return i2c_.writeReg(BQ25798_DEFAULT_ADDR, reg, data);
}

bool BQ25798::readReg(bq25798_reg_t reg, uint8_t *data) {
    return i2c_.readReg(BQ25798_DEFAULT_ADDR, uint8_t(reg), data);
}

bool BQ25798::readBlock(bq25798_reg_t reg, uint8_t data[], size_t len) {
    return i2c_.read(BQ25798_DEFAULT_ADDR, reg, data, len);
}



bool BQ25798::writeBlock(bq25798_reg_t reg, uint8_t data[], size_t len) {
    return i2c_.write(BQ25798_DEFAULT_ADDR, reg, data, len);
}
