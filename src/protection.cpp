#include "protection.h"
#include "log_codes.h"
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
    bool newHeatingEnabled = false;

    // Check if we have recovered from a cell under voltage.
    // This has some hysteresis to prevent cells close to under voltage turning
    // back on too quickly.
    if (cellUnderVoltageProtection && balancer.uvCellRecovered()) {
        balancer.clearUVFault();
        logCode(LOG_PROT_UV_RECOVERED);
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
        logCode(ocdScdState == BQ76920_OCD_SCD_STATE::SHORT_CIRCUIT ? LOG_PROT_SCD : LOG_PROT_OCD);
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
            logCode(LOG_PROT_OCD_SCD_CLEAR);
        }
    }

    // Check the Over Voltage and Under Voltage state of the cells.
    BQ76920_OV_UV_STATE ovuvState = balancer.getOVUVState();
    switch (ovuvState) {
    case BQ76920_OV_UV_STATE::OVER_VOLTAGE:
        logCode(LOG_PROT_CELL_OV);
        newChargeEnabled = false;
        break;
    case BQ76920_OV_UV_STATE::UNDER_VOLTAGE:
        logCode(LOG_PROT_CELL_UV);
        cellUnderVoltageProtection = true;
        newDischargeEnabled = false;
        break;
    case BQ76920_OV_UV_STATE::UNDER_VOLTAGE_AND_OVER_VOLTAGE:
        // Some cells are under voltage and some are over voltage. This would suggest something has done very wrong.
        // TODO: Decide what we want to do here, beep some sort of error tone?
        logCode(LOG_PROT_CELL_UV_OV);
        newChargeEnabled = false;
        newDischargeEnabled = false;
        break;
    }

    // Checking Battery temperatures.
    bool chargerWasSleeping = charger.isSleeping();
    float temp1 = balancer.readTemp();
    logCodeI16(LOG_PROT_BQ76920_TEMP, int16_t(temp1 * 10));
    float temp2 = charger.readTemp();
    logCodeI16(LOG_PROT_CHARGER_TEMP, int16_t(temp2 * 10));
    float cellMinTemp = min(temp1, temp2);
    float cellMaxTemp = max(temp1, temp2);

    // If the charger was in sleep mode then put it back in sleep mode.
    if (chargerWasSleeping) {
        charger.sleepMode();
    }

    if (cellMinTemp <= JEITA_T0) {
        // Cell temperature too low. Disable charging and enable heating.
        newHeatingEnabled = true;
        newChargeEnabled = false;
    }

    // Heater hysteresis.
    if (heatingEnabled && cellMinTemp <= JEITA_T0 + 1) {
        newHeatingEnabled = true;
    }

    if (cellMaxTemp >= JEITA_T5) {
        // Cell temperature too high. Disable charging, discharging and balancing.
        newChargeEnabled = false;
        newDischargeEnabled = false;
        newBalancingEnabled = false;
        newHeatingEnabled = false;
    }

    // Set if the temperature is low enough to require slower discharging.
    balancer.lowTempDischargeProtection(cellMinTemp <= REDUCE_DISCHARGE_TEMP);

    if (cellMinTemp <= STOP_DISCHARGE_TEMP) {
        // Cell temperature too low. Disable discharging.
        newDischargeEnabled = false;
    }

    if (charger.vbatOvpStat()) {
        logCode(LOG_PROT_VBAT_OVP);
        newChargeEnabled = false;
    }

    // Checking AHT20 humidity. High humidity risks condensation inside the pack.
    aht20.readResult();
    logCodeI16U16(LOG_PROT_AHT20, int16_t(aht20.temperature() * 10), uint16_t(aht20.humidity() * 10));
    if (aht20.humidity() >= HUMIDITY_MAX) {
        newChargeEnabled = false;
        newDischargeEnabled = false;
        newBalancingEnabled = false;
    }

    // TODO: Run more checks
    //      - Using the internal temperature sensors on the chips that have them.

    // Check that we are in a healthy state (no protection were triggered).
    bool newHealthy = newChargeEnabled && newDischargeEnabled && newBalancingEnabled;

    // Log the new state if anything changed.
    if (newHealthy != healthy || newChargeEnabled != chargeEnabled || newDischargeEnabled != dischargeEnabled ||
        newBalancingEnabled != balancingEnabled || newHeatingEnabled != heatingEnabled) {
        uint8_t flags = (newHealthy ? 0x01 : 0) | (newChargeEnabled ? 0x02 : 0) | (newDischargeEnabled ? 0x04 : 0) |
                        (newBalancingEnabled ? 0x08 : 0) | (newHeatingEnabled ? 0x10 : 0);
        logCodeU8(LOG_PROT_STATE, flags);
    }

    // Write the new state.
    healthy = newHealthy;
    chargeEnabled = newChargeEnabled;
    dischargeEnabled = newDischargeEnabled;
    balancingEnabled = newBalancingEnabled;
    heatingEnabled = newHeatingEnabled;

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

    if (heatingEnabled) {
        digitalWrite(PIN_EN_HEATER, HIGH);
    } else {
        digitalWrite(PIN_EN_HEATER, LOW);
    }
}
