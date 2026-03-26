#include "protection.h"
#include "util.h"

ProtectionState::ProtectionState(BQ25798 &charger_, BQ76920 &balancer_, AHT20 &aht20_)
    : charger(charger_), balancer(balancer_), aht20(aht20_) {}

bool ProtectionState::isChargingEnabled() { return chargeEnabled; }

bool ProtectionState::isBalancingEnabled() { return balancingEnabled; }

// update will run the checks and update the protection state for the battery pack. Charger checks are skipped when the
// charger is in sleep mode.
void ProtectionState::update() {

    // Set the new initial values as the default ones.
    bool newChargeEnabled = true;
    bool newDischargeEnabled = true;
    bool newBalancingEnabled = true;
    bool newHealthy = true;

    // Check if we have recovered from a cell under voltage.
    // This has some hysteresis to prevent cells close to under voltage turning
    // back on too quickly.
    if (cellUnderVoltageProtection && balancer.uvCellRecovered()) {
        println("UV recov");
        cellUnderVoltageProtection = false;
    }
    if (cellUnderVoltageProtection) {
        newDischargeEnabled = false;
        // We don't need to limit balancing here as balancing will have its own
        // limit on how low it can discharge the cell to. // TODO, add` this
        newBalancingEnabled = false;
    }

    // Check OCD/SCD hardware protection status and handle recovery.
    // After an OCD/SCD fault, the BQ76920 hardware automatically disables DSG.
    // Recovery requires: CHG disabled → load removed → clear fault → re-enable DSG.
    BQ76920_OCD_SCD_STATE ocdScdState = balancer.getOCDSCDState();
    if (ocdScdState != BQ76920_OCD_SCD_STATE::HEALTHY) {
        if (ocdScdState == BQ76920_OCD_SCD_STATE::SHORT_CIRCUIT) {
            println("Pack SCD");
        } else {
            println("Pack OCD");
        }
        ocdScdProtection = true;
    }
    if (ocdScdProtection) {
        newDischargeEnabled = false;
        newBalancingEnabled = false;
        // Charging must be disabled for LOAD_PRESENT detection to be active.
        newChargeEnabled = false;
        if (!balancer.isLoadPresent()) {
            balancer.clearOCDSCDFault();
            ocdScdProtection = false;
            println("OCD/SCD recov");
        }
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
    print("76920 T:");
    println(temp1);
    if (temp1 <= TEMPERATURE_POINT_1 || temp1 >= TEMPERATURE_POINT_5) {
        newChargeEnabled = false;
        newDischargeEnabled = false;
        newBalancingEnabled = false;
    }

    // ===== Running the charger checks =======
    if (!charger.isSleeping()) {
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
            println("25798 T err");
            newChargeEnabled = false;
            newDischargeEnabled = false;
            newBalancingEnabled = false;
        }
    }

    // Checking AHT20 humidity. High humidity risks condensation inside the pack.
    aht20.readResult();
    print("AHT20 T:");
    println(aht20.temperature());
    print("Hum:");
    println(aht20.humidity());
    if (aht20.humidity() >= HUMIDITY_MAX) {
        newChargeEnabled = false;
        newDischargeEnabled = false;
        newBalancingEnabled = false;
    }

    // TODO: Run more checks
    //      - Using the internal temperature sensors on the chips that have them.

    // Log the changes from the old state to the new state
    newHealthy = newHealthy && newChargeEnabled && newDischargeEnabled && newBalancingEnabled;
    bool changes = false;
    if (newHealthy != healthy) {
        changes = true;
        print("Health:");
        if (newHealthy) {
            println("ok");
        } else {
            println("bad");
        }
    }
    if (newChargeEnabled != chargeEnabled) {
        changes = true;
        print("Chg:");
        if (newChargeEnabled) {
            println("on");
        } else {
            println("off");
        }
    }
    if (newDischargeEnabled != dischargeEnabled) {
        changes = true;
        print("Dischg:");
        if (newDischargeEnabled) {
            println("on");
        } else {
            println("off");
        }
    }
    if (newBalancingEnabled != balancingEnabled) {
        changes = true;
        print("Bal:");
        if (newBalancingEnabled) {
            println("on");
        } else {
            println("off");
        }
    }

    // Write the new state.
    healthy = newHealthy;
    chargeEnabled = newChargeEnabled;
    dischargeEnabled = newDischargeEnabled;
    balancingEnabled = newBalancingEnabled;

    // Write out the entire state if any changes were made.
    if (changes) {
        print("State H:");
        print(healthy);
        print(" B:");
        print(balancingEnabled);
        print(" C:");
        print(chargeEnabled);
        print(" D:");
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
