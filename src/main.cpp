#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <main.h>

#include "aht20.h"
#include "bq25798.h"
#include "bq76920.h"
#include "log_codes.h"
#include "protection.h"
#include "util.h"

#define WDT_DURATION WDTO_2S

BQ25798 charger;
BQ76920 balancer;
AHT20 tempHumidity;
ProtectionState protectionState = ProtectionState(charger, balancer, tempHumidity);

#define PIN_LED PIN_PB5
#define PIN_TS1 PIN_PA7       // BQ76920 TS1
#define PIN_ALERT PIN_PA6     // BQ76920 ALERT
#define PIN_INTERRUPT PIN_PA2 // BQ25798 Interrupt
#define PIN_CE PIN_PA5        // BQ25798 ~Charge Enable

uint32_t seconds = 0; // Don't need to worry about an overflow for this as it will last
                      // (2^32-1)/60/60/24/365 = 136 Years at 1 tick per second
uint32_t lastChargerUpdateSeconds = 0;
uint32_t lastBalancerUpdateSeconds = 0;
uint32_t lastChargerWDTSeconds = 0;
uint32_t lastProtectionUpdateSeconds = (uint32_t)0 - 6; // Fire on the first loop iteration.
uint32_t lastStatusLogSeconds = 0;
bool sleepModeEnabled = false;
uint32_t lastInputSourceTime = 0;
volatile bool chargerInterrupted = false;
volatile bool balancerInterrupted = false;
volatile bool cellUnderVoltageProtection = false;

void enableSleepMode() {
    charger.sleepMode();
    // balancer.shipMode(); // Can't put the balancer in ship mode as that
    // disables the output.
    logCode(LOG_MAIN_SLEEP_MODE);
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

    // Untested/reviewed code.
    // // Mirror inverted UART TX (PB2) onto PC3 using CCL LUT1.
    // // LUT1 output pin is PC3; truth table 0x55 = NOT(IN0).
    // PORTC.DIR |= PIN3_bm;
    // CCL.LUT1CTRLB = CCL_INSEL0_USART0_gc; // IN0 = USART0 TX, IN1 = MASK
    // CCL.LUT1CTRLC = 0;                    // IN2 = MASK
    // CCL.TRUTH1    = 0x55;                 // NOT(IN0)
    // CCL.LUT1CTRLA = CCL_OUTEN_bm | CCL_ENABLE_bm;
    // CCL.CTRLA     = CCL_ENABLE_bm;

    logCode(LOG_MAIN_STARTING);
#endif

    // TODO: Detect boot reason.

    // Start up buzzer noise.
    buzzer_pin_init();
    buzzer_beep();

    // Setup i2C
    Wire.begin();

    // Try to find the BQ25798 (MPPT charger)
    if (!charger.begin(PIN_CE)) {
        logCode(LOG_MAIN_BQ25798_NOT_FOUND);
        restart();
    }
    logCode(LOG_MAIN_BQ25798_FOUND);
    waitUntilNextBeep();
    buzzer_beep();

    // Try to find the BQ76920 (cell balancer)
    // The BQ76920 is powered from the battery pack voltage so we will wait until we can detect a battery pack before
    // trying to find the BQ76920.
    while (1) {
        if (charger.vbatPresent()) {
            break;
        }
        delay(100);
        wdt_reset();
    }
    waitUntilNextBeep();
    buzzer_beep();

    // Try to find the BQ76920 (cell balancer)
    // If it is the first power up we might need to wake it up first.
    if (!balancer.begin()) {
        wakeUpBalancer();
        if (!balancer.begin()) {
            logCode(LOG_MAIN_BQ76920_NOT_FOUND);
            restart();
        }
    }
    logCode(LOG_MAIN_BQ76920_FOUND);
    waitUntilNextBeep();
    buzzer_beep();

    // Try to find the AHT20 (temperature and humidity sensor)
    if (!tempHumidity.begin()) {
        logCode(LOG_MAIN_AHT20_NOT_FOUND);
        restart();
    }
    logCode(LOG_MAIN_AHT20_FOUND);
    // Trigger the first AHT20 measurement now so it completes during the remaining startup sequence.
    tempHumidity.trigger();
    waitUntilNextBeep();
    buzzer_beep();

    // Once we have found the BQ76920 we should turn off the charging so we can get a good read on what the cell
    // voltages are on start up.
    charger.disable();
    delay(1000);

    if (!balancer.properCellPopulation()) {
        logCode(LOG_MAIN_CELL_POP_FAIL);
        waitUntilNextBeep();
        restart();
    }

    // Check that temperature readings are sensible at startup.
    // All sensors must read 15–35 °C and agree within 10 °C of each other.
    if (!tempHumidity.readResult()) {
        logCode(LOG_MAIN_AHT20_FAIL);
        waitUntilNextBeep();
        restart();
    }
    float ahtTemp = tempHumidity.temperature();
    float balancerTemp = balancer.readTemp();
    float chargerTemp = charger.readTemp();
    logCode3I16(LOG_MAIN_TEMPS, int16_t(ahtTemp * 10), int16_t(balancerTemp * 10), int16_t(chargerTemp * 10));
    float tMin = min(ahtTemp, min(balancerTemp, chargerTemp));
    float tMax = max(ahtTemp, max(balancerTemp, chargerTemp));
    if (tMin < 15.0f || tMax > 40.0f) {
        logCode(LOG_MAIN_TEMP_OOR);
        waitUntilNextBeep();
        restart();
    }
    if (tMax - tMin > 10.0f) {
        logCode(LOG_MAIN_TEMP_MISMATCH);
        waitUntilNextBeep();
        restart();
    }
    waitUntilNextBeep();
    buzzer_beep();

    // Setup interrupts
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
        logCode(LOG_MAIN_CHARGER_INT);
        // The flags get cleared when they get read so we read them out and store them locally so we can use them later,
        // gets cleared at the end of the loop.
        charger.readFlags();
    }
    if (balancerInterrupted) {
        logCode(LOG_MAIN_BALANCER_INT);
    }

    // Run the main logic depending on what mode the battery is in.
    if (sleepModeEnabled) {
        sleepMode();
    } else {
        mainMode();
    }

    // Update the protection state every 5 seconds.
    if (seconds - lastProtectionUpdateSeconds > 5) {
        lastProtectionUpdateSeconds = seconds;
        protectionState.update();
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

// Send periodic status snapshot every 10 seconds.
#if SERIAL_ENABLE
    if (seconds - lastStatusLogSeconds >= 10) {
        lastStatusLogSeconds = seconds;

        float bq76920Temp = balancer.readTemp();
        uint16_t cellMv[3] = {0, 0, 0};
        balancer.readCellMilliVoltages(cellMv);

        BQ25798ADC chargerADC = {};
        charger.readADCAll(chargerADC);
        int16_t ibat_cc_ma = balancer.readCurrentMA();

        int16_t tempAht = int16_t(tempHumidity.temperature() * 10);
        int16_t tempBq76920 = int16_t(bq76920Temp * 10);
        int16_t tempBq25798 = int16_t(chargerADC.tempC * 10);
        uint8_t humPct = uint8_t(tempHumidity.humidity());

        uint8_t chgStat[5] = {};
        charger.readStatusRegs(chgStat);

        uint8_t bqStat[4] = {};
        balancer.readStatusRegs(bqStat);

        Serial.write(LOG_STATUS);
        Serial.write((uint8_t *)&seconds, 4);
        Serial.write((uint8_t *)&tempAht, 2);
        Serial.write((uint8_t *)&tempBq76920, 2);
        Serial.write((uint8_t *)&tempBq25798, 2);
        Serial.write(humPct);
        Serial.write((uint8_t *)&cellMv[0], 2);
        Serial.write((uint8_t *)&cellMv[1], 2);
        Serial.write((uint8_t *)&cellMv[2], 2);
        Serial.write((uint8_t *)&chargerADC.vbus_mv, 2);
        Serial.write((uint8_t *)&chargerADC.ibus_ma, 2);
        Serial.write((uint8_t *)&chargerADC.vbat_mv, 2);
        Serial.write((uint8_t *)&chargerADC.ibat_ma, 2);
        Serial.write((uint8_t *)&ibat_cc_ma, 2); // BQ76920 CC current
        Serial.write(chgStat, 5);                // REG1B..1F
        Serial.write(bqStat, 4);                 // SYS_STAT,CELLBAL1,CTRL1,CTRL2
    }
#endif

// Make sure we flush the serial buffer before going to sleep.
#if SERIAL_ENABLE
    Serial.flush();
#endif

    // If the next loop will run a protection check, trigger the AHT20 now so the
    // measurement completes during sleep and the result is fresh when we read it.
    if (seconds - lastProtectionUpdateSeconds >= 5) {
        tempHumidity.trigger();
    }

    // Trigger a one-shot CC measurement one second before the status log so the result
    // is ready (250ms measurement) when we read it.
    if (seconds - lastStatusLogSeconds >= 9) {
        balancer.triggerCC();
    }

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
            logCode(LOG_MAIN_INPUT_SRC);
            // Re-initialize the charger since waking from ship mode causes a
            // power-on reset of the BQ25798, wiping all register configuration.
            charger.init();
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
        logCode(LOG_MAIN_NO_INPUT_SLEEP);
        enableSleepMode();
    }

    // Enable sleep mode if we don't have a poor power source. Constant checking of a poor power source can slowly drain
    // the battery.
    if (charger.poorSourceFlag) {
        logCode(LOG_MAIN_POOR_SRC_SLEEP);
        charger.disableVBUSWakeup();
        enableSleepMode();
    }
}

void restart() {
#if SERIAL_ENABLE
    logCode(LOG_MAIN_RESTARTING);
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
