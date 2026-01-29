#include "bq76920.h"
#include "ntcTemp.h"
#include "util.h"

#define CELL_COUNT 3
// Check that we can find the BQ76920 on the I2C bus
// Returning false if it can not be found.
bool BQ76920::begin() {
    writeReg(BQ76920_REG0B_CC_CFG, 0x19);
    uint8_t readBackVal = 0;
    readReg(BQ76920_REG0B_CC_CFG, &readBackVal);
    if (readBackVal != 0x19) {
        return false;
    }
    getADCGainAndOffset();
    writeOVandUVTripVoltages();
    return true;
}

void BQ76920::enableCharging() { setBit(BQ76920_REG05_SYS_CTRL2, 0, true); }

void BQ76920::disableCharging() { setBit(BQ76920_REG05_SYS_CTRL2, 0, false); }

void BQ76920::enableDischarging() { setBit(BQ76920_REG05_SYS_CTRL2, 1, true); }

void BQ76920::disableDischarging() { setBit(BQ76920_REG05_SYS_CTRL2, 1, false); }

// uint8_t BQ76920::getReg(bq76920_reg_t reg) {
//     uint8_t data;
//     readReg(reg, &data);
//     return data;
// }

// Returns the temperature in °C
// There is no automatic logic on the BQ76920 to react to temperatures that are too high
// or low so we need to program stopping/reducing the charging/discharging current.
//
float BQ76920::readTemp() {
    // Set the TEMP_SEL bit to 1 to read the external temperature sensor
    setBit(BQ76920_REG04_SYS_CTRL1, 3, true);

    uint8_t data[2] = {};
    readBlock(BQ76920_REG2C_TS1_HI, data, 2);
    uint32_t rawADC = (data[0] & 0b00111111) << 8 | data[1];
    uint32_t RTADC = (rawADC * 382) / 1000;
    uint32_t Resistance = (10000 * RTADC) / (3300 - RTADC);
    float temp = ntcTempFromResistance(Resistance);
    if (temp > 60.0 || temp < 5.0) {
        print("Temperature out of range: ");
        println(temp);
    }
    return temp;
}

bool BQ76920::writeReg(bq76920_reg_t reg, uint8_t data) {
    // Need to handle the CRC.
    // {(write address) 0x01, (register) reg, (data) data}
    // The CRC is calculate from the I2C write address (normal address << 1), the register and the data itself.
    uint8_t bytes[] = {0x10, reg, data};
    uint8_t crc = crc8_atm(bytes, 3);
    uint8_t writeBytes[] = {data, crc};
    return i2c_.write(BQ76920_ADDRESS, reg, writeBytes, 2);
}

bool BQ76920::writeBlock(bq76920_reg_t reg, uint8_t data[], uint8_t len) {
    uint8_t writeData[len * 2] = {};

    // For the first byte calculate the CRC from the I2C address write address and the data.
    uint8_t bytes[] = {0x10, reg, data[0]};
    writeData[0] = data[0];
    writeData[1] = crc8_atm(bytes, 3);

    // Calculate hash just from the data byte only
    for (int i = 1; i < len; i++) {
        writeData[i * 2] = data[i];
        writeData[i * 2 + 1] = crc8_atm(&data[i], 1);
    }
    return i2c_.write(BQ76920_ADDRESS, reg, writeData, len * 2);
}

bool BQ76920::readReg(bq76920_reg_t reg, uint8_t *data) {
    uint8_t readData[2];
    i2c_.read(BQ76920_ADDRESS, reg, readData, 2);
    uint8_t readCRC = readData[1];
    // the data used to calculate the CRC is the write address and the data bit.
    uint8_t crcData[] = {0x11, readData[0]};
    uint8_t calculatedCRC = crc8_atm(crcData, 2);
    if (calculatedCRC != readCRC) {
        println("Error with CRC in reading the register");
        return false;
    }
    *data = readData[0];
    return true;
}

bool BQ76920::readBlock(bq76920_reg_t reg, uint8_t data[], size_t len) {
    // Need a custom read function to handle the CRC.
    // We need to read twice the bytes to get the CRC bytes also.
    uint8_t dataAndCRC[2 * len];
    i2c_.read(BQ76920_ADDRESS, reg, dataAndCRC, len * 2);
    // The first CRC byte is the read address (0x11) and the first data byte.
    uint8_t crcData[] = {0x11, dataAndCRC[0]};
    uint8_t calculatedCRC = crc8_atm(crcData, 2);
    if (calculatedCRC != dataAndCRC[1]) {
        println("Error with CRC for the first bit block");
        println(calculatedCRC);
        println(dataAndCRC[1]);
        println(crcData[0]);
        println(crcData[1]);
        return false;
    }
    data[0] = dataAndCRC[0];

    // For the other bytes the CRC is calculated just from the data byte.
    for (size_t i = 1; i < len; i++) {
        crcData[0] = dataAndCRC[i * 2];
        calculatedCRC = crc8_atm(crcData, 1);
        if (calculatedCRC != dataAndCRC[i * 2 + 1]) {
            println("Error with CRC in reading the block");
            return false;
        }
        data[i] = dataAndCRC[i * 2];
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
    adcOffset = (int8_t)readReg(BQ76920_REG51_ADC_OFFSET, &adcOffsetRegVal); // convert from 2's complement

    uint8_t adcGainRegVal1;
    uint8_t adcGainRegVal2;
    readReg(BQ76920_REG50_ADC_GAIN1, &adcGainRegVal1);
    readReg(BQ76920_REG59_ADC_GAIN2, &adcGainRegVal2);
    adcGain = 365 + (((adcGainRegVal1 & B00001100) << 1) | ((adcGainRegVal2 & B11100000) >> 5)); // uV/LSB
}

BQ76920_OV_UV_STATE BQ76920::getOVUVState() {
    // TOOD: Add hysteresis for cell recovery so as to not trip right away again.

    // Get state if there is a UV or OV trip.
    uint8_t sysStat;
    readReg(BQ76920_REG00_SYS_STAT, &sysStat);
    if (!(sysStat & BQ76920_SYS_STAT_OV) && !(sysStat & BQ76920_SYS_STAT_UV)) {
        // No OV or UV trip, we are healthy.
        return BQ76920_OV_UV_STATE::HEALTHY;
    }

    // Get new cell voltage readings
    getCellVoltages();

    // Check if we have an OV or UV
    bool ov = false;
    bool uv = false;
    for (int i = 0; i < 5; i++) {
        if (cellShouldBePopulated(i)) {
            if (cellMilliVoltages[i] < 20) {
                println("Cell should be populated!!!!");
                // TODO: What do we do here?
            }

            if (cellMilliVoltages[i] >= cellOVMilliVoltage) {
                ov = true;
            }
            if (cellMilliVoltages[i] <= cellUVMilliVoltage) {
                uv = true;
            }
        } else {
            if (cellMilliVoltages[i] > 20) {
                println("Cell should not be populated!!!!");
                // TODO: What do we do here?
            }
        }
    }

    // If a unhealthy state is found, return it.
    if (ov && uv) {
        return BQ76920_OV_UV_STATE::UNDER_VOLTAGE_AND_OVER_VOLTAGE;
    }
    if (ov) {
        return BQ76920_OV_UV_STATE::OVER_VOLTAGE;
    }
    if (uv) {
        return BQ76920_OV_UV_STATE::UNDER_VOLTAGE;
    }

    // No unhealthy state is found, clear the OV and UV trip bits.
    writeReg(BQ76920_REG00_SYS_STAT, BQ76920_SYS_STAT_OV | BQ76920_SYS_STAT_UV);
    return BQ76920_OV_UV_STATE::HEALTHY;
}

bool BQ76920::getCellVoltages() {
    uint8_t data[10];
    if (!readBlock(BQ76920_REG0C_VC1_HI, data, 10)) {
        println("Error reading cell voltages");
        return false;
    }

    for (int i = 0; i < 5; i++) {
        cellMilliVoltages[i] = calculateADC(data[i * 2], data[i * 2 + 1]);
    }
    return true;
}

uint16_t BQ76920::calculateADC(uint8_t msb, uint8_t lsb) {
    uint32_t adcRaw = (msb & 0b00111111) << 8 | lsb;
    // adcRaw (no unit) * adcGain (uV/LSB) + adcOffset (mV)
    uint32_t microVolts = adcRaw * adcGain + adcOffset * 1000;
    return microVolts / 1000;
}

void BQ76920::stopCellBalancing() {
    // Stop any cells from balancing.
    writeReg(BQ76920_REG01_CELL_BAL1, 0x00);
}

void BQ76920::updateBalanceRoutine() {
    if (!getCellVoltages()) {
        println("Error getting cell voltages!!!!");
        return;
    }

    // Find the min and max voltage.
    uint16_t minVoltage = 50000;
    uint16_t maxVoltage = 0;

    uint8_t newMaxVoltageCell = 0;
    for (int i = 0; i < 5; i++) {
        if (cellShouldBePopulated(i)) {
            // println("===============");
            if (cellMilliVoltages[i] < 20) {
                println("Cell should be populated!!!!");
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
                println("Cell should not be populated!!!!");
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
            println("Cell balancing enabled");
        } else {
            println("Cell balancing disabled");
        }
    }

    if (balancing) {
        if (change) {
            print("Cell difference: ");
            println(maxVoltage - minVoltage);
            print("Cell balance needed. Balancing cell ");
            print(maxVoltageCell);
            print(" max cell voltage: ");
            print(maxVoltage);
            print(" min cell voltage: ");
            println(minVoltage);
        }
        // Drain the max cell.
        writeReg(BQ76920_REG01_CELL_BAL1, 0x01 << maxVoltageCell);
    } else {
        // Stop any cells from balancing.
        writeReg(BQ76920_REG01_CELL_BAL1, 0x00);
    }
}

bool BQ76920::cellShouldBePopulated(int cell) {
    // For 3 cells
    bool values[] = {true, true, false, false, true};
    return values[cell];
}

bool BQ76920::setBit(bq76920_reg_t reg, uint8_t bit, bool value) {
    uint8_t regValue;
    readReg(reg, &regValue);

    if (value) {
        regValue |= (1 << bit);
    } else {
        regValue &= ~(1 << bit);
    }

    return writeReg(reg, regValue);
}

bool BQ76920::uvCellRecovered() {
    getCellVoltages();
    for (int i = 0; i < 5; i++) {
        if (cellShouldBePopulated(i)) {
            if (cellMilliVoltages[i] <= CELL_UV_RECOVERY) {
                return false;
            }
        }
    }
    return true;
}

void BQ76920::debugLogging() {
    if (!getCellVoltages()) {
        println("Error getting cell voltages!!!!");
        return;
    }

    // Find the min and max voltage.
    uint16_t minVoltage = 50000;
    uint16_t maxVoltage = 0;

    uint8_t newMaxVoltageCell = 0;
    for (int i = 0; i < 5; i++) {
        if (cellShouldBePopulated(i)) {
            // println("===============");
            if (cellMilliVoltages[i] < 20) {
                println("Cell should be populated!!!!");
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
                println("Cell should not be populated!!!!");
                // TODO
            }
        }
    }
    // print("Max voltage: ");
    // println(maxVoltage);
    print("Min voltage: ");
    println(minVoltage);
}

void BQ76920::writeOVandUVTripVoltages() {
    // Holds the reg data for the UV and OV voltages.
    uint8_t writeData[2];

    // Calculate the target raw value for over voltage.
    uint32_t targetRaw = (uint32_t(CELL_OV_TARGET) * 1000 - adcOffset * 1000) / adcGain;
    if (TEST_MAXIMUM_CELL_VOLTAGE) {
        println("TEST_MAXIMUM_CELL_VOLTAGE");
        targetRaw = 0x2000; // This is the lowest possible value for the over voltage protection.
    }
    // Check that the raw value is in range.
    if ((targetRaw & 0x3000) != 0x2000) {
        println("Target cell over voltage out of range!!!!");
        print("Target raw value: ");
        println(targetRaw);
        print("Max value: ");
        println((0x2FF8 * adcGain + adcOffset * 1000) / 1000);
        print("Min value: ");
        println((0x2008 * adcGain + adcOffset * 1000) / 1000);
    }
    // We only write the middle 8 bits of the 16 bits.
    // The first and last 4 are staticly set.
    // The first byte is the OV value.
    writeData[0] = (targetRaw >> 4) & 0xFF;

    // Calculate the target raw value for under voltage.
    targetRaw = (uint32_t(CELL_UV_TARGET) * 1000 - adcOffset * 1000) / adcGain;
    if (TEST_MINIMUM_CELL_VOLTAGE) {
        println("TEST_MINIMUM_CELL_VOLTAGE");
        targetRaw = 0x1FFF; // This is the highest possible value for the under voltage protection.
    }
    // Check that the raw value is in range.
    if ((targetRaw & 0x3000) != 0x1000) {
        println("Target cell under voltage out of range!!!!");
        print("Target raw value: ");
        println(targetRaw);
        print("Max value: ");
        println((0x1FF8 * adcGain + adcOffset * 1000) / 1000);
        print("Min value: ");
        println((0x1008 * adcGain + adcOffset * 1000) / 1000);
    }
    // We only write the middle 8 bits of the 16 bits.
    // The first and last 4 are staticly set.
    // The second byte is the UV value.
    writeData[1] = (targetRaw >> 4) & 0xFF;

    // Write to the OV and UV registers
    if (!writeBlock(BQ76920_REG09_OV_TRIP, writeData, 2)) {
        println("Failed to write OV and UV trip voltages!!!!");
    }

    // Read back the OV and UV trip voltages as a way to check the write/math.
    uint8_t blockData[2];
    if (!readBlock(BQ76920_REG09_OV_TRIP, blockData, 2)) {
        println("Failed to read OV and UV trip voltages!!!!");
        return;
    }

    // 0b10_blockData[0]_1000
    uint32_t ovRaw = uint32_t(1 << (4 + 8 + 1)) + (uint32_t(blockData[0]) << 4) + (1 << 3);
    cellOVMilliVoltage = (ovRaw * adcGain) / 1000 + adcOffset;
    print("Actual OV trip voltage: ");
    println(cellOVMilliVoltage);

    // uvRaw: 0b01_blockData[1]_0000
    uint32_t uvRaw = uint32_t(1 << (4 + 8)) + (uint32_t(blockData[1]) << 4);
    cellUVMilliVoltage = (uvRaw * adcGain) / 1000 + adcOffset;
    print("Actual UV trip voltage: ");
    println(cellUVMilliVoltage);
}

void BQ76920::shipMode() {
    stopCellBalancing();
    writeReg(BQ76920_REG04_SYS_CTRL1, 0x00);
    writeReg(BQ76920_REG04_SYS_CTRL1, 0x01);
    writeReg(BQ76920_REG04_SYS_CTRL1, 0x02);
}
