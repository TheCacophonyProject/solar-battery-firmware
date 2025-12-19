#include "util.h"
#include "protection.h"

// ==== Functions to set battery pack protection states ====

void ProtectionState::disableCharge() {
    chargeEnabled = false;
    healthy = false;
}

void ProtectionState::disableDischarge() {
    dischargeEnabled = false;
    healthy = false;
}

void ProtectionState::disableBalancing() {
    balancingEnabled = false;
    healthy = false;
}

void ProtectionState::limitChargeCurrent(uint16_t mA) {
    // Only allow to reduce the charge current.
    // This is to protect from one state setting it low then a other state setting it higher than it should be.
    maxChargeCurrent = min(chargeEnabled, mA);
    healthy = false;
}

// ==== Functions to check what the battery pack protection state should be ====

bool ProtectionState::getChargingEnabled() {
    return chargeEnabled;
}

bool ProtectionState::getDischargingEnabled() {
    return dischargeEnabled;
}

bool ProtectionState::getBalancingEnabled() {
    return balancingEnabled;
}

uint16_t ProtectionState::getChargeLimit() {
    return maxChargeCurrent;
}

// ==== Logging function ====
void ProtectionState::logStateChange(ProtectionState newState) {
    bool changes = false;
    if (newState.healthy != healthy) {
        changes = true;
        if (newState.healthy) {
            println("Health change: unhealthy -> healthy");
        } else {
            println("Health change: healthy -> unhealthy");
        }
    }
    if (newState.chargeEnabled != chargeEnabled) {
        changes = true;
        if (newState.chargeEnabled) {
            println("Charging change: disabled -> enabled");
        } else {
            println("Charging change: enabled -> disabled");
        }
    }
    if (newState.dischargeEnabled != dischargeEnabled) {
        changes = true;
        if (newState.dischargeEnabled) {
            println("Discharging change: disabled -> enabled");
        } else {
            println("Discharging change: enabled -> disabled");
        }
    }
    if (newState.balancingEnabled != balancingEnabled) {
        changes = true;
        if (newState.balancingEnabled) {
            println("Balancing change: disabled -> enabled");
        } else {
            println("Balancing change: enabled -> disabled");
        }
    }
    if (changes) {
        print("New state; Healthy: ");
        print(newState.healthy);
        print(", Balancing: ");
        print(newState.balancingEnabled);
        print(", Charging: ");
        print(newState.chargeEnabled);
        print(", Discharging: ");
        println(newState.dischargeEnabled);
    }
}
