#ifndef M24C02_H
#define M24C02_H

#include "i2c.h"

#define M24C02_ADDRESS      0x50  // A2=A1=A0=0
#define M24C02_VERSION_ADDR 0x00  // EEPROM address where major/minor/patch are stored

struct PcbVersion {
    uint8_t major;
    uint8_t minor;
    uint8_t patch;
};

class M24C02 {
  public:
    bool begin();
    bool readPcbVersion(PcbVersion *version);
    bool isCompatible(const PcbVersion &version);

  private:
    I2C i2c_;
};

#endif
