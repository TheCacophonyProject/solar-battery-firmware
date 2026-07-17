#include "m24c02.h"
#include "log_codes.h"
#include "util.h"

// List of PCB versions this firmware is compatible with.
// Add entries here as the PCB evolves without requiring firmware changes.
static const PcbVersion COMPATIBLE_VERSIONS[] = {
    {0, 2, 0}, // TODO. Test when we have a newer version of the PCB.
};

bool M24C02::begin() {
    uint8_t dummy;
    return i2c_.readReg(M24C02_ADDRESS, M24C02_DATA_ADDR, &dummy);
}

EepromResult M24C02::readData(EepromData *data) {
    uint8_t buf[EEPROM_DATA_LEN];
    if (!i2c_.read(M24C02_ADDRESS, M24C02_DATA_ADDR, buf, EEPROM_DATA_LEN)) {
        return EEPROM_I2C_ERR;
    }

    uint16_t storedCrc = (uint16_t)buf[10] | ((uint16_t)buf[11] << 8);
    if (crc16CCITT(buf, EEPROM_DATA_LEN - 2) != storedCrc) {
        return EEPROM_CRC_ERR;
    }

    data->version = buf[0];
    data->timestamp = (uint32_t)buf[1] | ((uint32_t)buf[2] << 8) | ((uint32_t)buf[3] << 16) | ((uint32_t)buf[4] << 24);
    data->id = (uint16_t)buf[5] | ((uint16_t)buf[6] << 8);
    data->pcb.major = buf[7];
    data->pcb.minor = buf[8];
    data->pcb.patch = buf[9];

    if (data->version != EEPROM_DATA_VERSION) {
        return EEPROM_VERSION_ERR;
    }
    return EEPROM_OK;
}

bool M24C02::isCompatible(const PcbVersion &version) {
    for (uint8_t i = 0; i < sizeof(COMPATIBLE_VERSIONS) / sizeof(COMPATIBLE_VERSIONS[0]); i++) {
        const PcbVersion &v = COMPATIBLE_VERSIONS[i];
        if (version.major == v.major && version.minor == v.minor && version.patch == v.patch) {
            return true;
        }
    }
    return false;
}
