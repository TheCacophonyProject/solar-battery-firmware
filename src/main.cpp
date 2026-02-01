#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <main.h>

#include "bq25798.h"
#include "bq76920.h"
#include "protection.h"
#include "util.h"

#define WDT_DURATION WDTO_2S

BQ25798 charger;
BQ76920 balancer;
ProtectionState protectionState = ProtectionState(charger, balancer);

#define PIN_LED PIN_PB5
#define PIN_TS1 PIN_PA7       // BQ76920 TS1
#define PIN_ALERT PIN_PA6     // BQ76920 ALERT
#define PIN_INTERRUPT PIN_PA2 // BQ25798 Interrupt
#define PIN_CE PIN_PA5        // BQ25798 ~Charge Enable

uint32_t seconds = 0; // Don't need to worry about an overflow for this as it will last
                      // (2^32-1)/60/60/24/365 = 136 Years at 1 tick per second
uint32_t lastChargerUpdateSeconds = 0;
uint32_t lastBalancerUpdateSeconds = 0;
bool sleepModeEnabled = false;
uint32_t lastInputSourceTime = 0;
volatile bool chargerInterrupted = false;
volatile bool balancerInterrupted = false;
volatile bool cellUnderVoltageProtection = false;

void enableSleepMode() {
    charger.sleepMode();
    // balancer.shipMode(); // Can't put the balancer in ship mode as that
    // disables the output.
    println("sleep mode");
#if SERIAL_ENABLE
    Serial.flush();
#endif
    sleepModeEnabled = true;
}

unsigned long lastBeep = millis();

void waitUntilNextBeep() {
    while (millis() - lastBeep < 500) {
    }
    lastBeep = millis();
    wdt_reset();
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
    pinMode(BUZZER_PIN, OUTPUT);

    // Set pin initial states
    ledOff();

// Setup serial interface if enabled
#if SERIAL_ENABLE
    Serial.begin(9600);
    println("Starting");
#endif

    // TODO: Detect boot reason.

    // Start up buzzer noise.
    buzzer_pin_init();
    buzzer_beep();

    // Setup i2C
    Wire.begin();

    // Try to find the BQ25798 (MPPT charger)
    if (!charger.begin(PIN_CE)) {
        println(F("Could not find BQ25798"));
        restart();
    }
    println(F("BQ25798 found"));
    waitUntilNextBeep();
    buzzer_beep();

    // Try to find the BQ76920 (cell balancer)
    // The BQ76920 is powered from the battery pack voltage so we will wait until we can detect a battery pack before
    // trying to find the BQ76920.
    // println("Waiting to sense battery pack voltage");
    // charger.enable(); // Do we need this to "wake" up fresh cells in the battery pack? The protection chips might
    // prevent them from connecting until there is some external power.
    while (1) {
        if (charger.vbatPresent()) {
            // println("Detected battery pack voltage");
            break;
        }
        delay(100);
        wdt_reset();
    }
    waitUntilNextBeep();
    buzzer_beep();

    // Try to find the BQ76920 (cell balancer)
    // If it is the first power up we might need to wake it up first.
    // println("Connecting to balancer.");
    if (!balancer.begin()) {
        // println("waking up balancer");
        wakeUpBalancer();
        if (!balancer.begin()) {
            // println(F("Could not find the BQ76920"));
            restart();
        }
    }
    println(F("BQ76920 found"));
    waitUntilNextBeep();
    buzzer_beep();

    // Once we have found the BQ76920 we should turn off the charging so we can get a good read on what the cell
    // voltages are on start up.

    charger.disable();
    delay(1000);

    if (!balancer.properCellPopulation()) {
        // println("Incorrect cell population");
        waitUntilNextBeep();
        restart();
    }

    // Setup interrupts
    // println("Setting up interrupts and PIT");
    attachInterrupt(digitalPinToInterrupt(PIN_ALERT), balancerInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), chargerInterrupt, FALLING);
    setupPIT();
    waitUntilNextBeep();
    buzzer_beep();

    // Play setup finished noise
    waitUntilNextBeep();
    waitUntilNextBeep();
    start_up_buzz();

    set_sleep_mode(SLEEP_MODE_PWR_DOWN);
    sleep_enable();
}

// waleUpBalancer will go through the routine for waking up the BQ76920 when it has first been powered on.
void wakeUpBalancer() {
    pinMode(PIN_TS1, OUTPUT);
    digitalWrite(PIN_TS1, HIGH);
    delay(100);
    pinMode(PIN_TS1, INPUT);
    delay(100);
}

// Interrupt from the BQ25798 MPPT charger INT pin.
void chargerInterrupt() {
    // Setting charger interrupted flag to true.
    // This is a interrupt routine so we don't want to do any processing here.
    chargerInterrupted = true;
}

// Interrupt from the BQ76920 cell balancer ALERT pin.
void balancerInterrupt() {
    // Setting balancer interrupted flag to true.
    // This is a interrupt routine so we don't want to do any processing here.
    balancerInterrupted = true;
}

void ledOn() { digitalWrite(PIN_LED, HIGH); }

void ledOff() { digitalWrite(PIN_LED, LOW); }

void loop() {
    // Reset WDT to prevent it from triggering.
    wdt_reset();

    // Update the time in seconds by using the PIT.
    seconds = getSeconds();

    // Log any interrupts
    if (chargerInterrupted) {
        println("Charger int");
        // The flags get cleared when they get read so we read them out and store them locally so we can use them later,
        // gets cleared at the end of the loop.
        charger.readFlags();
    }
    if (balancerInterrupted) {
        println("Balancer int");
    }

    // Run the main logic depending on what mode the battery is in.
    if (sleepModeEnabled) {
        sleepMode();
    } else {
        mainMode();
    }

    // Update to the balancing state every 3 seconds if balancing is allowed.
    if (seconds - lastBalancerUpdateSeconds > 3) {
        lastBalancerUpdateSeconds = seconds;
        if (protectionState.isBalancingEnabled()) {
            balancer.updateBalanceRoutine();
        } else {
            balancer.stopCellBalancing();
        }
    }

    // Clear interrupt flags.
    noInterrupts(); // We need to disable interrupts while doing this to avoid race conditions.
    chargerInterrupted = false;
    balancerInterrupted = false;
    interrupts(); // Re-enable interrupts after we clear the flags.

    // When the charger reads the flags they get reset. So we store them to a local variable so they can be used
    // throughout the loop. Here we clear them.
    charger.clearFlags();

// Make sure we flush the serial buffer before going to sleep.
#if SERIAL_ENABLE
    Serial.flush();
#endif

    // FLush the Wire, this makes sure that the ATtiny goes into a proper sleep state.
    Wire.flush();

    // Go to sleep to reduce power. Can be woken up by:
    // - PIT interrupt (every second).
    // - BQ76920 ALERT pin.
    // - BQ25798 Interrupt pin.
    wdt_reset();
    sleep_mode();
}

void sleepMode() {
    // Little heartbeat flash to show we are alive.
    if (seconds % 30 == 0) {
        ledOn();
        delay(1);
        ledOff();
    }

    // Update the protection state, skipping the charger checks.
    protectionState.update(false);

    if (seconds % 10 == 0) {
        // Update the balancer routine every 10 seconds.
        balancer.updateBalanceRoutine();

        // Every 10 seconds we alow it to interrupt the attiny if there is a new source. We don't want this always
        // active as when there is a poor source it will constantly be waking up the attiny using up the power in the
        // process.
        charger.enableVBUSWakeup();
    }

    if (seconds % 10 == 0 || chargerInterrupted || balancerInterrupted) {
        // Check if we have an input source.
        if (charger.haveInputSource()) {
            println("Have input source");
            wakeUpBalancer();
            sleepModeEnabled = false;
            return;
        } else {
            // If we are not waking up from sleep the put the charger chip back
            // into sleep mode.
            charger.sleepMode();
        }
    }

    if (charger.poorSourceFlag) {
        // If a poor source flag is set then we will disable it waking up from VBUS_PRESENT as a poor source can
        // continuously wake up the chip from VBUS_PRESENT.
        charger.disableVBUSWakeup();
    }
}

void mainMode() {
    // Little heartbeat flash to show we are alive.
    ledOn();
    delay(1);
    ledOff();

    // Update the protection state, including the charger checks.
    protectionState.update(true);

    // Update to the charger state.
    if (seconds - lastChargerUpdateSeconds > 5) {
        lastChargerUpdateSeconds = seconds;
        if (protectionState.isChargingEnabled()) {
            charger.enable();
            charger.checkSourceAndMPPT();
        } else {
            charger.disable();
        }
        charger.checkStatus();
    }

    // === Check if we need to go into a sleep mode.
    if (charger.haveInputSource() && !charger.inHighInputImpedance()) {
        lastInputSourceTime = seconds;
    }

    // TODO: Some more testing is needed to be done to check stability of the system switching between sleep mode to
    //       normal mode.

    // Enable sleep mode if we don't have an input source for 20 seconds.
    if (seconds > lastInputSourceTime + 20) {
        println("No input for 20 seconds, going into sleep mode");
        enableSleepMode();
    }

    // Enable sleep mode if we don't have a poor power source. Constant checking of a poor power source can slowly drain
    // the battery.
    if (charger.poorSourceFlag) {
        println("Poor source, going into sleep mode");
        charger.disableVBUSWakeup();
        enableSleepMode();
    }
}

void restart() {
#if SERIAL_ENABLE
    println("Restarting.");
    Serial.flush();
#endif
    waitUntilNextBeep();
    buzzer_on(500);
    delay(400);
    buzzer_on(200);
    delay(400);
    buzzer_off();

    cli();
    _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
    while (1) {
    }
}
