#ifndef PROTECTION_H
#define PROTECTION_H

#include <Arduino.h>

class ProtectionState {
    public:
        void disableCharge();
        void disableDischarge();
        void disableBalancing();
        void limitChargeCurrent(uint16_t mA);

        bool getChargingEnabled();
        bool getDischargingEnabled();
        bool getBalancingEnabled();
        uint16_t getChargeLimit();

        void logStateChange(ProtectionState newState);
    private:
        bool chargeEnabled = true;          // If false then the charge MOSFET controlled by the BQ76920 should be disabled and the BQ25798 should disable charging.
        bool dischargeEnabled = true;       // If false then the discharge MOSFET controlled by the BQ76920 should be disabled.
        bool balancingEnabled = true;       // If false then the balancing from the BQ76920 should be disabled.
        uint16_t maxChargeCurrent = 2000;   // This should set the maximum charge current from the BQ25798.
        bool healthy = true;                // If any of the states are changed from the default (healthy), this will be set false.
};
#endif