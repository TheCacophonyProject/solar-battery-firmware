#ifndef PROTECTION_H
#define PROTECTION_H

#include "aht20.h"
#include "bq25798.h"
#include "bq76920.h"
#include <Arduino.h>

//    ^
//    |-- No charging allowed. Too cold.
//    |
#define TEMPERATURE_POINT_1 0
//    |
//    |-- Reduced charging/discharging current.
//    |
#define TEMPERATURE_POINT_2 10
//    |
//    |-- Normal operating conditions.
//    |
#define TEMPERATURE_POINT_3 50
//    |
//    |-- Reduced charging/discharging current.
//    |
#define TEMPERATURE_POINT_5 60
//    |
//    |-- No charging allowed. Too hot.
//    V

// Humidity threshold above which charging and discharging are disabled.
#define HUMIDITY_MAX 80

class ProtectionState {
  public:
    ProtectionState(BQ25798 &charger, BQ76920 &balancer, AHT20 &aht20);
    void update();

    bool isChargingEnabled();
    bool isBalancingEnabled();

  private:
    bool cellUnderVoltageProtection = false;
    bool ocdScdProtection = false;
    bool chargeEnabled = true; // If false then the charge MOSFET controlled by the BQ76920 should be disabled and the
                               // BQ25798 should disable charging.
    bool dischargeEnabled = true; // If false then the discharge MOSFET controlled by the BQ76920 should be disabled.
    bool balancingEnabled = true; // If false then the balancing from the BQ76920 should be disabled.

    // TODO: uint16_t maxChargeCurrent = 2000;   // This should set the maximum charge current from the BQ25798.
    bool healthy = true; // If any of the states are changed from the default
                         // (healthy), this will be set false.
    BQ25798 &charger;
    BQ76920 &balancer;
    AHT20 &aht20;
};
#endif