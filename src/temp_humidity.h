#ifndef TEMP_HUMIDITY_H
#define TEMP_HUMIDITY_H

#include "aht20.h"
#include "hdc2080.h"
#include "m24c02.h"

// PCB revisions before 0.3.0 have an AHT20 temperature/humidity sensor fitted; 0.3.0 onwards have
// an HDC2080 instead. TempHumiditySensor picks the right driver at begin() based on the PCB
// version read from the EEPROM, and forwards to it from then on.
#define HDC2080_MIN_PCB_MAJOR 0
#define HDC2080_MIN_PCB_MINOR 3
#define HDC2080_MIN_PCB_PATCH 0

class TempHumiditySensor {
  public:
    bool begin(const PcbVersion &pcb);

    bool trigger();
    bool readResult();

    float temperature() const;
    float humidity() const;

    bool usingHdc2080() const { return useHdc2080_; }

  private:
    AHT20 aht20_;
    HDC2080 hdc2080_;
    bool useHdc2080_ = false;
};

#endif
