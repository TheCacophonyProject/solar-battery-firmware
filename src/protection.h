#ifndef PROTECTION_H
#define PROTECTION_H

#include "aht20.h"
#include "bq25798.h"
#include "bq76920.h"
#include <Arduino.h>

#define PIN_EN_HEATER PIN_PC0 // Pin used to turn on the internal trace heater

// Discharging temperature limits are a bit different to charging.
// We need to reduce discharging when below 0C and then completely stop discharging when below -10C.
#define REDUCE_DISCHARGE_TEMP 0
#define STOP_DISCHARGE_TEMP -10

// Because we charge at most 0.5C we just need to stop charging when temperature is too high or too low.
// Set Charging limits following the JEITA standard.
//    ^
//    |-- No charging allowed. Too cold.
//    |
#define JEITA_T0 0
//    |
//    |-- Reduced charging current to 0.5C. With at least 2 cells populated at 2500mAh that gives a max charging current
//    of 2.5A. That is already the maximum charge current we support so nothing needs to be done here.
//    |
#define JEITA_T1 10
//    |
//    |-- Normal operating conditions.
//    |
#define JEITA_T3 50
//    |
//    |-- Reduced charging current to 0.5C. With at least 2 cells populated at 2500mAh that gives a max charging current
//    of 2.5A. That is already the maximum charge current we support so nothing needs to be done here.
//    |
#define JEITA_T5 60
//    |
//    |-- No charging/discharging allowed. Too hot.
//    V

// Humidity threshold above which charging and discharging are disabled.
#define HUMIDITY_MAX 80

class ProtectionState {
  public:
    ProtectionState(BQ25798 &charger, BQ76920 &balancer, AHT20 &aht20);
    void update();

    bool isChargingEnabled();
    bool isBalancingEnabled();
    bool isHeatingEnabled();
    uint8_t getStateFlags() const {
        return (healthy ? 0x01 : 0) | (chargeEnabled ? 0x02 : 0) | (dischargeEnabled ? 0x04 : 0) |
               (balancingEnabled ? 0x08 : 0);
    }

  private:
    bool cellUnderVoltageProtection = false;
    bool ocdScdProtection = false;
    bool chargeEnabled = true; // If false then the charge MOSFET controlled by the BQ76920 should be disabled and the
                               // BQ25798 should disable charging.
    bool dischargeEnabled = true; // If false then the discharge MOSFET controlled by the BQ76920 should be disabled.
    bool balancingEnabled = true; // If false then the balancing from the BQ76920 should be disabled.
    bool heatingEnabled = false;

    // TODO: uint16_t maxChargeCurrent = 2000;   // This should set the maximum charge current from the BQ25798.
    bool healthy = true; // If any of the states are changed from the default
                         // (healthy), this will be set false.
    BQ25798 &charger;
    BQ76920 &balancer;
    AHT20 &aht20;
};
#endif