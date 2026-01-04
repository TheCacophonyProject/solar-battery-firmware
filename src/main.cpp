#include <main.h>
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <Wire.h>

#include "bq25798.h"
#include "bq76920.h"
#include "protection.h"
#include "util.h"

#define WDT_DURATION WDTO_2S

BQ25798 charger;
BQ76920 balancer;

#define PIN_LED PIN_PB5
#define PIN_TS1 PIN_PA7       // BQ76920 TS1
#define PIN_ALERT PIN_PA6     // BQ76920 ALERT
#define PIN_INTERRUPT PIN_PA2 // BQ25798 Interrupt
#define PIN_CE PIN_PA5        // BQ25798 ~Charge Enable

uint32_t seconds = 0; // Don't need to worry about an overflow for this as it will last (2^32-1)/60/60/24/365 = 136 Years at 1 tick per second
uint32_t lastChargerUpdateSeconds = 0;
uint32_t lastBalancerUpdateSeconds = 0;
bool sleepModeEnabled = false;
uint32_t lastInputSourceTime = 0;
volatile bool chargerInterrupted = false;
volatile bool balancerInterrupted = false;


void enableSleepMode() {
  charger.shipMode();
  //balancer.shipMode(); // Can't put the balancer in ship mode as that disables the output.
  println("sleep mode");
  #if SERIAL_ENABLE
    Serial.flush();
  #endif
  sleepModeEnabled = true;
}

void setup() {
  // Setup WDT
  wdt_reset();
  _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_2KCLK_gc);

  // Set pins modes
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TS1, INPUT);
  pinMode(PIN_ALERT, INPUT);
  pinMode(PIN_INTERRUPT, INPUT_PULLUP);

  // Set pin initial states
  ledOff();

  #if SERIAL_ENABLE
    // Setup serial logging
    Serial.begin(9600);
    println("Starting...");
  #endif

  // Setup i2C
  Wire.begin();

  // Try to find the BQ25798 (MPPT charger)
  if (!charger.begin(PIN_CE)) {
    println(F("Could not find the BQ25798"));
    restart();
  }
  println(F("Charger BQ25798 found!"));

  // Try to find the BQ76920 (cell balancer)
  wakeUpBalancer();
  if (!balancer.begin()) {
    println(F("Could not find the BQ76920"));
    restart();
  }
  println(F("Balancer BQ76920 found!"));
  
  // Setup interrupts
  println("Setting up interrupts from BQ76920 ALERT and BQ25798 INT");
  attachInterrupt(digitalPinToInterrupt(PIN_ALERT), balancerInterrupt, RISING);
  attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), chargerInterrupt, FALLING);

  println("Setting up PIT");
  setupPIT();
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

void wakeUpBalancer() {
  pinMode(PIN_TS1, OUTPUT);
  digitalWrite(PIN_TS1, HIGH);
  delay(300);
  pinMode(PIN_TS1, INPUT);
  delay(300);
}

void chargerInterrupt() {
  chargerInterrupted = true;
}

void balancerInterrupt() {
  balancerInterrupted = true;
}

void ledOn() {
  digitalWrite(PIN_LED, HIGH);
}

void ledOff() {
  digitalWrite(PIN_LED, LOW);
}

void loop() {
  // Update the time in seconds.
  seconds = getSeconds();

  // Reset WDT to prevent it from triggering.
  wdt_reset();

  if (chargerInterrupted) {
    println("Charger interrupt");
    charger.dumpFlags();
    chargerInterrupted = false;
  }

  if (balancerInterrupted) {
    println("Balancer interrupt");
    balancerInterrupted = false;
  }


  // Run the main logic. This is done in a separate function so it can return early if needed.  
  if (sleepModeEnabled) {
    sleepMode();
  } else {
    mainLogic();
  }

  // When the charger reads the flags they get reset. So we store them to 
  // a local variable so they can be used throughout the loop but we then want to clear them.
  charger.clearFlags();
  
  // Make sure we flush the serial buffer before going to sleep.
  #if SERIAL_ENABLE
    Serial.flush();
  #endif

  // FLush the Wire, this makes sure that it goes into a proper sleep state.
  Wire.flush();

  // Go to sleep to reduce power. Can be woken up by:
  // - PIT interrupt (every second).
  // - BQ76920 ALERT pin.
  // - BQ25798 Interrupt pin.
  wdt_reset();
  sleep_mode();
}

ProtectionState protectionState = ProtectionState();

void sleepMode() {
  // Little heartbeat flash to show we are alive.
  if (seconds%30 == 0) {
    ledOn();
    delay(1);
    ledOff();
  }

  if (seconds % 10 == 0) {
    // Every 10 seconds we alow it to interrupt the attiny if there is a new source.
    // We don't want this always active as when there is a poor source it will constantly be 
    // waking up the attiny using up the power in the process.
    charger.enableVBUSWakeup();
  }

  if (seconds % 10 == 0 || chargerInterrupted || balancerInterrupted) {
    // ===== Checks to see if we need to wake up from sleep mode =====

    // Check if we have an input source.
    if (charger.haveInputSource()) {
      println("Have input source, leaving sleep mode");
      wakeUpBalancer();
      sleepModeEnabled = false;
      return;
    } else {
      // If we are not waking up from sleep the put the charger chip back into ship mode.
      charger.shipMode();
    }
  }

  if (charger.poorSourceFlag) {
    // If a poor source flag is set then we will disable it waking up from VBUS_PRESENT as
    // a poor source can continuously wake up the chip from VBUS_PRESENT.
    charger.disableVBUSWakeup();
  }
}

void mainLogic() {
  // Little heartbeat flash to show we are alive.
  ledOn();
  delay(1);
  ledOff();

  // ===== PROTECTION CHECKS =====

  // Create the protection state in the default (healthy) state.
  ProtectionState newProtectionState = ProtectionState(); 

  // Check the OV and UV state of the cells.
  BQ76920_OV_UV_STATE ovuvState = balancer.getOVUVState();
  if (ovuvState == BQ76920_OV_UV_STATE::OVER_VOLTAGE) {
    println("Cell OV");
    newProtectionState.disableCharge();
  }
  if (ovuvState == BQ76920_OV_UV_STATE::UNDER_VOLTAGE) {
    println("Cell UV");
    newProtectionState.disableDischarge();
  }
  if (ovuvState == BQ76920_OV_UV_STATE::UNDER_VOLTAGE_AND_OVER_VOLTAGE) {
    println("Cell UV and OV");
    newProtectionState.disableCharge();
    newProtectionState.disableDischarge();
  }
  
  // Pack Over Voltage check by reading hte VBAT_OVP_STAT reg from FAULT_Status_0 
  if (charger.vbatOvpStat()) {
    println("VBAT OVP");
    newProtectionState.disableCharge();
  }

  // Checking BQ76920 temperature
  float temp1 = balancer.readTemp();
  if (temp1 <= TEMPERATURE_POINT_1 || temp1 >= TEMPERATURE_POINT_5) {
    print("BQ76920 temp sensor out of range: ");
    println(temp1);
    newProtectionState.disableCharge();
    newProtectionState.disableDischarge();
    newProtectionState.disableBalancing();
  } else if (temp1 <= TEMPERATURE_POINT_2 || temp1 >= TEMPERATURE_POINT_3) {
    // TODO: Figure out what we want to do here.
    //  1) Try to mimic the behaviour as though the BQ25798 temperature sensor was in these ranges.
    //  2) Nothing?
  }

  // Checking BQ25798 temperature
  // It seams the BQ25798 charger will only set the state to COLD, COOL, WARM, or HOT if 
  // it has a good input source for charging. We have the other temperature sensor so we 
  // can use that for disableing the charger in a discharge state (powering the camera at night).
  // TODO: We can make manual temperature readings to see if it is COLD, COOL, WARM, or HOT
  //       so at some point we should implement that.
  BQ25798_TEMP bq25798_temp_state = charger.getTemperatureStatus();
  if (bq25798_temp_state == BQ25798_TEMP::HOT || bq25798_temp_state == BQ25798_TEMP::COLD) {
    println("BQ25798 temp sensor out of range");
    newProtectionState.disableCharge();
    newProtectionState.disableDischarge();
    newProtectionState.disableBalancing();
  }

  // Log changes in the protection state.
  protectionState.logStateChange(newProtectionState);

  // Update the protection state.
  protectionState = newProtectionState;

  // TODO: Checks to add later:
  // Check temperature of the temp+humidity sensor when we have the new board
  // Pack under voltage, This needs to be checked so we can go into a sleep state to reduce quiescent current.

  // ==== Apply protections ====
  if (!protectionState.getBalancingEnabled()) {
    // Balancing should not be enabled.
    balancer.stopCellBalancing();
  }

  if (protectionState.getChargingEnabled()) {
    charger.enable();
    balancer.enableCharging();
  } else {
    charger.disable();
    balancer.disableCharging();
  }

  if (protectionState.getDischargingEnabled()) {
    balancer.enableDischarging();
  } else {
    balancer.disableDischarging();
  }

  // Check if we can go into a deep sleep mode for the balancer and charger.

  // ===== UPDATE CHECKS =====

  // Update to the charger state.
  if (seconds - lastChargerUpdateSeconds > 5) {
    lastChargerUpdateSeconds = seconds;
    
    // Try to start charging again if allowed.
    if (protectionState.getChargingEnabled()) {
      charger.enable();
      charger.checkSourceAndMPPT();  
    }
    
    // Check the status.
    charger.checkStatus();
  }

  // Update to the balancing state.
  if (seconds - lastBalancerUpdateSeconds > 3) {
    lastBalancerUpdateSeconds = seconds;

    // Update the balancing state if balancing is allowed.
    if (protectionState.getBalancingEnabled()) {
      balancer.updateBalanceRoutine();
    }
  }

  // === Check if we need to go into a sleep mode.
  if (charger.haveInputSource() && !charger.inHighInputImpedance()) {
    lastInputSourceTime = seconds;
  }

  // Enable sleep mode if we don't have an input source for 20 seconds.
  if (seconds > lastInputSourceTime + 20) {
    println("No input for 20 seconds, going into sleep mode");
    enableSleepMode();
  }

  if (charger.poorSourceFlag) {
    println("Poor source, going into sleep mode");
    charger.disableVBUSWakeup();
    enableSleepMode();
  }
}

void restart() {
  #if SERIAL_ENABLE
    println("Restarting...");
    Serial.flush();
  #endif

  delay(100);

  cli();
  _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
  while (1) { }
}
