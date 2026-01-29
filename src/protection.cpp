#include "protection.h"
#include "util.h"

ProtectionState::ProtectionState(BQ25798 &charger_, BQ76920 &balancer_) : charger(charger_), balancer(balancer_) {}

bool ProtectionState::isChargingEnabled() { return chargeEnabled; }

bool ProtectionState::isBalancingEnabled() { return balancingEnabled; }

// update will run the checks and update the protection state for the battery pack. If the battery pack is not charging
// it can be set to not run the checks for the charger (saving power consumption).
void ProtectionState::update(bool runChargerChecks) {

    // Set the new initial values as the default ones.
    bool newChargeEnabled = true;
    bool newDischargeEnabled = true;
    bool newBalancingEnabled = true;
    bool newHealthy = true;

    // Check if we have recovered from a cell under voltage.
    // This has some hysteresis to prevent cells close to under voltage turning
    // back on too quickly.
    if (cellUnderVoltageProtection && balancer.uvCellRecovered()) {
        println("UV cell recovered");
        cellUnderVoltageProtection = false;
    }
    if (cellUnderVoltageProtection) {
        newDischargeEnabled = false;
        // We don't need to limit balancing here as balancing will have its own
        // limit on how low it can discharge the cell to. // TODO, add` this
        newBalancingEnabled = false;
    }

    // Check the Over Voltage and Under Voltage state of the cells.
    BQ76920_OV_UV_STATE ovuvState = balancer.getOVUVState();
    switch (ovuvState) {
    case BQ76920_OV_UV_STATE::OVER_VOLTAGE:
        println("Cell OV");
        newChargeEnabled = false;
        break;
    case BQ76920_OV_UV_STATE::UNDER_VOLTAGE:
        println("Cell UV");
        cellUnderVoltageProtection = true;
        newDischargeEnabled = false;
        break;
    case BQ76920_OV_UV_STATE::UNDER_VOLTAGE_AND_OVER_VOLTAGE:
        // Some cells are under voltage and some are over voltage. This would suggest something has done very wrong.
        // TODO: Decide what we want to do here, beep some sort of error tone?
        println("Cell UV and OV");
        newChargeEnabled = false;
        newDischargeEnabled = false;
        break;
    }

    // Checking BQ76920 external temperature sensor.
    float temp1 = balancer.readTemp();
    if (temp1 <= TEMPERATURE_POINT_1 || temp1 >= TEMPERATURE_POINT_5) {
        print("BQ76920 temp out of range: ");
        println(temp1);
        newChargeEnabled = false;
        newDischargeEnabled = false;
        newBalancingEnabled = false;
    }

    // ===== Running the charger checks =======
    if (runChargerChecks) {
        // Pack Over Voltage check by reading hte VBAT_OVP_STAT reg from
        // FAULT_Status_0
        if (charger.vbatOvpStat()) {
            println("VBAT OVP");
            newChargeEnabled = false;
        }

        // Checking BQ25798 external temperature sensor.
        // It seams the BQ25798 charger will only set the state to COLD, COOL, WARM, or HOT if it has a good input
        // source for charging. We have other temperature sensors so we can check those. when we don't have a good input
        // source.
        // TODO: We can make manual temperature readings to see if it is COLD, COOL, WARM, or HOT so at some point we
        // should implement that.
        BQ25798_TEMP bq25798_temp_state = charger.getTemperatureStatus();
        if (bq25798_temp_state == BQ25798_TEMP::HOT || bq25798_temp_state == BQ25798_TEMP::COLD) {
            println("BQ25798 temp sensor out of range");
            newChargeEnabled = false;
            newDischargeEnabled = false;
            newBalancingEnabled = false;
        }
    }

    // TODO: Run more checks
    //      - Using the I2C temp/humidity sensor.
    //      - Using the internal temperature sensors on the chips that have them.

    // Log the changes from the old state to the new state
    bool changes = false;
    if (newHealthy != healthy) {
        changes = true;
        print("Health changed to: ");
        if (newHealthy) {
            println("healthy");
        } else {
            println("unhealthy");
        }
    }
    if (newChargeEnabled != chargeEnabled) {
        changes = true;
        print("Charging changed to: ");
        if (newChargeEnabled) {
            println("enabled");
        } else {
            println("disabled");
        }
    }
    if (newDischargeEnabled != dischargeEnabled) {
        changes = true;
        print("Discharging changed to: ");
        if (newDischargeEnabled) {
            println("enabled");
        } else {
            println("disabled");
        }
    }
    if (newBalancingEnabled != balancingEnabled) {
        changes = true;
        print("Balancing changed to: ");
        if (newBalancingEnabled) {
            println("enabled");
        } else {
            println("disabled");
        }
    }

    // Write the new state.
    healthy = newHealthy;
    chargeEnabled = newChargeEnabled;
    dischargeEnabled = newDischargeEnabled;
    balancingEnabled = newBalancingEnabled;

    // Write out the entire state if any changes were made.
    if (changes) {
        print("New state; Healthy: ");
        print(healthy);
        print(", Balancing: ");
        print(balancingEnabled);
        print(", Charging: ");
        print(chargeEnabled);
        print(", Discharging: ");
        println(dischargeEnabled);
    }

    // Apply the protections.
    if (balancingEnabled == false) {
        balancer.stopCellBalancing();
    }

    if (chargeEnabled) {
        charger.enable();
        balancer.enableCharging();
    } else {
        charger.disable();
        balancer.disableCharging();
    }

    if (dischargeEnabled) {
        balancer.enableDischarging();
    } else {
        balancer.disableDischarging();
    }
}
