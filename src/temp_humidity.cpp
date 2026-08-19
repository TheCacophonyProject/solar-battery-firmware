#include "temp_humidity.h"

bool TempHumiditySensor::begin(const PcbVersion &pcb) {
    useHdc2080_ = pcbAtLeast(pcb, HDC2080_MIN_PCB_MAJOR, HDC2080_MIN_PCB_MINOR, HDC2080_MIN_PCB_PATCH);
    return useHdc2080_ ? hdc2080_.begin() : aht20_.begin();
}

bool TempHumiditySensor::trigger() { return useHdc2080_ ? hdc2080_.trigger() : aht20_.trigger(); }

bool TempHumiditySensor::readResult() { return useHdc2080_ ? hdc2080_.readResult() : aht20_.readResult(); }

float TempHumiditySensor::temperature() const { return useHdc2080_ ? hdc2080_.temperature() : aht20_.temperature(); }

float TempHumiditySensor::humidity() const { return useHdc2080_ ? hdc2080_.humidity() : aht20_.humidity(); }
