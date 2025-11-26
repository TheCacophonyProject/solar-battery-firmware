#include "bq76920.h"


#define CELL_COUNT 3
// Check that we can find the BQ76920 on the I2C bus
// Returning false if it can not be found.
bool BQ76920::begin() {
    writeReg(CC_CFG_REG, 0x19);
    uint8_t readBackVal = 0;
    readReg(CC_CFG_REG, &readBackVal);
    if (readBackVal != 0x19) {
        return false;
    }
    getADCGainAndOffset();
    writeOVandUVTripVoltages();
    return true;
}

// Returns the temperature in °C
// There is no automatic logic on the BQ76920 to react to temperatures that are too high
// or low so we need to program stopping/reducing the charging/discharging current.
// 
float BQ76920::ReadTemp() {
    uint8_t data[2] = {};
    readBlock(0x2C, data, 2);
    uint32_t rawADC = (data[0] & 0b00111111)<< 8 | data[1];
    uint32_t RTADC = (rawADC * 382)/1000;
    uint32_t Resistance = (10000 * RTADC)/(3300-RTADC);
    float temp = ntc_temp_from_resistance(Resistance);
    if (temp > 60.0 || temp < 5.0) {
        Serial.print("Temperature out of range: ");
        Serial.println(temp);
    }
    return temp;
}

bool BQ76920::writeReg(uint8_t reg, uint8_t data) {
    // Need to handle the CRC.
    // {(write address) 0x01, (register) reg, (data) data}
    // The CRC is calculate from the I2C write address (normal address << 1), the register and the data itself.
    uint8_t bytes[] = {0x10, reg, data};
    uint8_t crc = crc8_atm(bytes, 3);
    uint8_t writeBytes[] = {data, crc};
    return i2c_.write(BQ76920_ADDRESS, reg, writeBytes, 2);
}

bool BQ76920::writeBlock(uint8_t reg, uint8_t data[], uint8_t len) {
    uint8_t writeData[len*2] = {};
    
    // For the first byte calculate the CRC from the I2C address write address and the data.
    uint8_t bytes[] = {0x10, reg, data[0]};
    writeData[0] = data[0];
    writeData[1] = crc8_atm(bytes, 3);

    // Calculate hash just from the data byte only
    for (int i = 1; i < len; i++) {
        writeData[i*2] = data[i];
        writeData[i*2+1] = crc8_atm(&data[i], 1);
    }
    return i2c_.write(BQ76920_ADDRESS, reg, writeData, len*2);

}

bool BQ76920::readReg(uint8_t reg, uint8_t *data) {
    uint8_t readData[2];
    i2c_.read(BQ76920_ADDRESS, reg, readData, 2);
    uint8_t readCRC = readData[1];
    // the data used to calculate the CRC is the write address and the data bit.
    uint8_t crcData[] = {0x11, readData[0]};
    uint8_t calculatedCRC =  crc8_atm(crcData, 2);
    if (calculatedCRC != readCRC) {
        Serial.println("Error with CRC in reading the register");
        return false;
    }
    *data = readData[0];
    return true;
}

bool BQ76920::readBlock(uint8_t reg, uint8_t data[], size_t len) {
    // Need a custom read function to handle the CRC.
    // We need to read twice the bytes to get the CRC bytes also.
    uint8_t dataAndCRC[2*len];
    i2c_.read(BQ76920_ADDRESS, reg, dataAndCRC, len*2);
    // The first CRC byte is the read address (0x11) and the first data byte.
    uint8_t crcData[] = {0x11, dataAndCRC[0]};
    uint8_t calculatedCRC =  crc8_atm(crcData, 2);
    if (calculatedCRC != dataAndCRC[1]) {
        Serial.println("Error with CRC for the first bit block");
        Serial.println(calculatedCRC);
        Serial.println(dataAndCRC[1]);
        Serial.println(crcData[0]);
        Serial.println(crcData[1]);
        return false;
    }
    data[0] = dataAndCRC[0];

    // For the other bytes the CRC is calculated just from the data byte.
    for (size_t i = 1; i < len; i++) {
        crcData[0] = dataAndCRC[i*2];
        calculatedCRC =  crc8_atm(crcData, 1);
        if (calculatedCRC != dataAndCRC[i*2+1]) {
            Serial.println("Error with CRC in reading the block");
            return false;
        }
        data[i] = dataAndCRC[i*2];
    }
    return true;
}

uint8_t BQ76920::crc8_atm(uint8_t *data, size_t len) {
    uint8_t crc = 0x00;
    for (size_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            } else {
                crc = (uint8_t)(crc << 1);
            }
        }
    }
    return crc;
}

void BQ76920::getADCGainAndOffset() {
    uint8_t adcOffsetRegVal;
    adcOffset = (signed int) readReg(0x51, &adcOffsetRegVal);  // convert from 2's complement

    uint8_t adcGainRegVal1;
    uint8_t adcGainRegVal2;
    readReg(0x50, &adcGainRegVal1);
    readReg(0x52, &adcGainRegVal2);
    adcGain = 365 + (((adcGainRegVal1 & B00001100) << 1) | ((adcGainRegVal2 & B11100000) >> 5)); // uV/LSB
}

bool BQ76920::getCellVoltages() {
    uint8_t data[10];
    if (!readBlock(0x0C, data, 10)) {
        Serial.println("Error reading cell voltages");
        return false;
    }

    for (int i = 0; i < 5; i++) {
        cellMilliVoltages[i] = CalculateADC(data[i*2], data[i*2+1]);
    }
    return true;
}

uint16_t BQ76920::CalculateADC(uint8_t msb, uint8_t lsb) {
    uint32_t adcRaw = (msb & 0b00111111) << 8 | lsb;
    // adcRaw (no unit) * adcGain (uV/LSB) + adcOffset (mV)
    uint32_t microVolts = adcRaw * adcGain + adcOffset*1000;
    return microVolts / 1000;
}

void BQ76920::updateBalanceRoutine() {
    if (!getCellVoltages()) {
        Serial.println("Error getting cell voltages!!!!");
        return;
    }

    // Find the min and max voltage.
    uint16_t minVoltage = 50000;
    uint16_t maxVoltage = 0;

    uint8_t newMaxVoltageCell = 0;
    for (int i = 0; i < 5; i++) {
        if (cellShouldBePopulated(i)) {
            //Serial.println("===============");
            if (cellMilliVoltages[i] < 20) {
                Serial.println("Cell should be populated!!!!");
                // TODO
            }
            
            if (cellMilliVoltages[i] < minVoltage) {
                minVoltage = cellMilliVoltages[i];
            }
            if (cellMilliVoltages[i] > maxVoltage) {
                maxVoltage = cellMilliVoltages[i];
                newMaxVoltageCell = i;
            }
        } else {
            if (cellMilliVoltages[i] > 20) {
                Serial.println("Cell should not be populated!!!!");
                // TODO
            }
        }

    }
    bool change = maxVoltageCell != newMaxVoltageCell;
    maxVoltageCell = newMaxVoltageCell;

    // Hysteresis
    bool newBalancing;
    if (balancing) {
        newBalancing = maxVoltage - minVoltage > CELL_BALANCE_THRESHOLD_END;
    } else {
        newBalancing = maxVoltage - minVoltage > CELL_BALANCE_THRESHOLD_START;
    }

    if (newBalancing != balancing) {
        balancing = newBalancing;
        if (balancing) {
            Serial.println("Cell balancing enabled");
        } else {
            Serial.println("Cell balancing disabled");
        }
    }

    if (balancing) {
        if (change) {
            Serial.print("Cell difference: "); Serial.println(maxVoltage - minVoltage);
            Serial.print("Cell balance needed. Balancing cell ");
            Serial.print(maxVoltageCell);
            Serial.print(" max cell voltage: ");
            Serial.print(maxVoltage);
            Serial.print(" min cell voltage: ");
            Serial.println(minVoltage);
        }
        // Drain the max cell.
        writeReg(CELL_BALANCE_REG, 0x01 << maxVoltageCell);
    } else {
        // Stop any cells from balancing.
        writeReg(CELL_BALANCE_REG, 0x00);
    }
}

bool BQ76920::cellShouldBePopulated(int cell) {
    // For 3 cells
    bool values[] = {true, true, false, false, true};
    return values[cell];
}

void BQ76920::readChargeDischargeBits() {
    uint8_t regData;
    readReg(0x05, &regData);

    bool DSG_ON = regData & 1<<1;
    Serial.print("DSG_ON: ");
    Serial.println(DSG_ON);
    bool CHG_ON = regData & 1<<0;
    Serial.print("CHG_ON: ");
    Serial.println(CHG_ON);
}

void BQ76920::readStatus() {
    uint8_t statusRegData;
    readReg(0x00, &statusRegData);
    bool ccReady = statusRegData & 1<<7;
    Serial.print("CC ready: ");
    Serial.println(ccReady);


    bool DEVICE_XREADY = statusRegData & 1<<5;
    Serial.print("DEVICE_XREADY: ");
    Serial.println(DEVICE_XREADY);


    bool OVRD_ALERT = statusRegData & 1<<4;
    Serial.print("OVRD_ALERT: ");
    Serial.println(OVRD_ALERT);


    bool UV = statusRegData & 1<<3;
    Serial.print("UV: ");
    Serial.println(UV);

    bool OV = statusRegData & 1<<2;
    Serial.print("OV: ");
    Serial.println(OV);

    bool SCD = statusRegData & 1<<1;
    Serial.print("SCD: ");
    Serial.println(SCD);

    bool OCD = statusRegData & 1<<0;
    Serial.print("OCD: ");
    Serial.println(OCD);

    getCellVoltages();
    for (int i = 0; i < 5; i++) {
        Serial.print("Cell ");
        Serial.print(i);
        Serial.print(": ");
        Serial.println(cellMilliVoltages[i]);
    }
}

void BQ76920::enableChargeAndDischarge() {
    setBit(0x05, 1, true);
    setBit(0x05, 0, true);
}

void BQ76920::disableChargeAndDischarge() {
    setBit(0x05, 1, false);
    setBit(0x05, 0, false);
}

bool BQ76920::setBit(uint8_t reg, uint8_t bit, bool value) {
    uint8_t regValue;
    readReg(reg, &regValue);

    if (value) {
        regValue |= (1 << bit);
    } else {
        regValue &= ~(1 << bit);
    }

    return writeReg(reg, regValue);
}

void BQ76920::writeOVandUVTripVoltages() {
    // Holds the reg data for the UV and OV voltages.
    uint8_t writeData[2];

    // Calculate the target raw value for over voltage.
    uint32_t targetRaw = (uint32_t(CELL_OV_TARGET)*1000 - adcOffset*1000) / adcGain;
    // Check that the raw value is in range.
    if ((targetRaw & 0x3000) != 0x2000) {
        Serial.println("Target cell over voltage out of range!!!!");
        Serial.print("Target raw value: ");
        Serial.println(targetRaw);
        Serial.print("Max value: ");
        Serial.println((0x2FF8 * adcGain + adcOffset*1000)/1000);
        Serial.print("Min value: ");
        Serial.println((0x2008 * adcGain + adcOffset*1000)/1000);
    }
    // We only write the middle 8 bits of the 16 bits.
    // The first and last 4 are staticly set.
    // The first byte is the OV value.
    writeData[0] = (targetRaw >> 4) & 0xFF;
    
    // Calculate the target raw value for under voltage.
    targetRaw = (uint32_t(CELL_UV_TARGET)*1000 - adcOffset*1000) / adcGain;
    // Check that the raw value is in range.
    if ((targetRaw & 0x3000) != 0x1000) {
        Serial.println("Target cell under voltage out of range!!!!");
        Serial.print("Target raw value: ");
        Serial.println(targetRaw);
        Serial.print("Max value: ");
        Serial.println((0x1FF8 * adcGain + adcOffset*1000)/1000);
        Serial.print("Min value: ");
        Serial.println((0x1008 * adcGain + adcOffset*1000)/1000);
    }
    // We only write the middle 8 bits of the 16 bits.
    // The first and last 4 are staticly set.
    // The second byte is the UV value.
    writeData[1] = (targetRaw >> 4) & 0xFF;

    // Write to the OV and UV registers
    if (!writeBlock(0x09, writeData, 2)) {
        Serial.println("Failed to write OV and UV trip voltages!!!!");
    }
    
    // Read back the OV and UV trip voltages as a way to check the write/math.
    uint8_t blockData[2];
    if (!readBlock(0x09, blockData, 2)) {
        Serial.println("Failed to read OV and UV trip voltages!!!!");
        return;
    }
    
    // 0b10_blockData[0]_1000
    uint32_t ovRaw = uint32_t(1<<(4+8+1)) + (uint32_t(blockData[0]) << 4) + (1<<3);
    cellOVMilliVoltage = (ovRaw * adcGain)/1000 + adcOffset;
    Serial.print("Actual OV trip voltage: ");
    Serial.println(cellOVMilliVoltage);

    // uvRaw: 0b01_blockData[1]_0000
    uint32_t uvRaw = uint32_t(1<<(4+8)) + (uint32_t(blockData[1]) << 4);
    cellUVMilliVoltage = (uvRaw * adcGain)/1000 + adcOffset;
    Serial.print("Actual UV trip voltage: ");
    Serial.println(cellUVMilliVoltage);
}

void BQ76920::clearOVandUVTrip() {
    // Update cell protection.

    // Check SYS status 1.
    // To clear the bits, write 1 to them.

    



    // 1) Check the UV and OV flags.
    // 2) If either are tripped, read the cell voltages and 

    // 1) Read the cell voltages.
    // 3)  


    // Check that all of the cells are in range, then clear the OV and UV trip.
    if (!getCellVoltages()) {
        Serial.println("Failed to read cell voltages");
        Serial.println("Cannot clear OV and UV trip");
        return;
    }


    if (!setBit(0x00, 3, true)) {
        Serial.println("Failed to clear OV trip");
    }
    if (!setBit(0x00, 2, false)) {
        Serial.println("Failed to clear UV trip");
    }
}

// 11702 With using String()
// 10292
// 7031 Without Serial.println()

#include <math.h>

typedef struct {
    float tempC;      // degrees C
    float res_kohm;   // resistance in kΩ
} NtcPoint;

// NCP□XH103 10k, B = 3380K
// Temp (°C) , Resistance (kΩ)
static const NtcPoint ntcTable[] = {
    { -40.0f, 195.652f },
    { -35.0f, 148.171f },
    { -30.0f, 113.347f },
    { -25.0f,  87.559f },
    { -20.0f,  68.237f },
    { -15.0f,  53.650f },
    { -10.0f,  42.506f },
    {  -5.0f,  33.892f },
    {   0.0f,  27.219f },
    {   5.0f,  22.021f },
    {  10.0f,  17.926f },
    {  15.0f,  14.674f },
    {  20.0f,  12.081f },
    {  25.0f,  10.000f },
    {  30.0f,   8.315f },
    {  35.0f,   6.948f },
    {  40.0f,   5.834f },
    {  45.0f,   4.917f },
    {  50.0f,   4.161f },
    {  55.0f,   3.535f },
    {  60.0f,   3.014f },
    {  65.0f,   2.586f },
    {  70.0f,   2.228f },
    {  75.0f,   1.925f },
    {  80.0f,   1.669f },
    {  85.0f,   1.452f },
    {  90.0f,   1.268f },
    {  95.0f,   1.110f },
    { 100.0f,   0.974f },
    { 105.0f,   0.858f },
    { 110.0f,   0.758f },
    { 115.0f,   0.672f },
    { 120.0f,   0.596f },
    { 125.0f,   0.531f },
};

#define NTC_TABLE_SIZE (sizeof(ntcTable) / sizeof(ntcTable[0]))

// Convert resistance (ohms) to temperature (°C)
float ntc_temp_from_resistance(float r_ohms)
{
    if (r_ohms <= 0.0f) {
        return NAN;
    }

    float r_kohm = r_ohms / 1000.0f;

    // Clamp outside table range
    if (r_kohm >= ntcTable[0].res_kohm) {
        return ntcTable[0].tempC;                             // <= -40°C
    }
    if (r_kohm <= ntcTable[NTC_TABLE_SIZE - 1].res_kohm) {
        return ntcTable[NTC_TABLE_SIZE - 1].tempC;            // >= 125°C
    }

    // Find the two surrounding points (R decreases as T increases)
    int i;
    for (i = 0; i < (int)NTC_TABLE_SIZE - 1; i++) {
        float r1 = ntcTable[i].res_kohm;
        float r2 = ntcTable[i + 1].res_kohm;

        if (r_kohm <= r1 && r_kohm >= r2) {
            // Log-linear interpolation in resistance
            float logR  = logf(r_kohm);
            float logR1 = logf(r1);
            float logR2 = logf(r2);

            float t = (logR - logR1) / (logR2 - logR1);  // 0..1
            float temp = ntcTable[i].tempC +
                         t * (ntcTable[i + 1].tempC - ntcTable[i].tempC);
            return temp;
        }
    }

    // Should never get here if table is monotonic
    return NAN;
}
