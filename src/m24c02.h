#ifndef M24C02_H
#define M24C02_H

#include "i2c.h"

#define M24C02_ADDRESS 0x50   // A2=A1=A0=0
#define M24C02_DATA_ADDR 0x00 // EEPROM address where the data block starts

// Layout version of the EEPROM data block this firmware understands.
#define EEPROM_DATA_VERSION 1

// EEPROM data block layout (12 bytes at M24C02_DATA_ADDR, little-endian):
//   offset 0:  u8  version    — layout version (EEPROM_DATA_VERSION)
//   offset 1:  u32 timestamp  — unix epoch seconds when programmed
//   offset 5:  u16 id         — battery box ID
//   offset 7:  u8  pcb major
//   offset 8:  u8  pcb minor
//   offset 9:  u8  pcb patch
//   offset 10: u16 crc        — crc16CCITT over bytes 0-9
#define EEPROM_DATA_LEN 12

struct PcbVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

struct EepromData {
    uint8_t version;    // Data layout version
    uint32_t timestamp; // Unix epoch seconds when the EEPROM was programmed
    uint16_t id;        // Battery box ID, should match the label on the box
    PcbVersion pcb;
};

enum EepromResult {
    EEPROM_OK,
    EEPROM_I2C_ERR,     // Device not responding / read failed
    EEPROM_CRC_ERR,     // CRC mismatch (blank or corrupt EEPROM)
    EEPROM_VERSION_ERR, // Data layout version not supported
};

// pcbAtLeast reports whether v is >= major.minor.patch. Used to gate firmware behaviour that
// depends on which PCB revision is fitted (e.g. which sensor or resistor values are populated).
inline bool pcbAtLeast(const PcbVersion &v, uint8_t major, uint8_t minor, uint8_t patch) {
    if (v.major != major) {
        return v.major > major;
    }
    if (v.minor != minor) {
        return v.minor > minor;
    }
    return v.patch >= patch;
}

class M24C02 {
  public:
    bool begin();
    EepromResult readData(EepromData *data);
    bool isCompatible(const PcbVersion &version);

  private:
    I2C i2c_;
};

#endif
