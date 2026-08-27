#include <Arduino.h>
#include <Wire.h>
#include <avr/interrupt.h>
#include <avr/sleep.h>
#include <avr/wdt.h>

#include "bq25798.h"
#include "bq76920.h"
#include "error_codes.h"
#include "log_codes.h"
#include "m24c02.h"
#include "main.h"
#include "protection.h"
#include "temp_humidity.h"
#include "util.h"
#include "version.h"

#define WDT_DURATION WDTO_2S

BQ25798 charger;
BQ76920 balancer;
TempHumiditySensor tempHumidity;
M24C02 eeprom;
ProtectionState protectionState = ProtectionState(charger, balancer, tempHumidity);

#define PIN_LED PIN_PB5
#define PIN_TS1 PIN_PA7                // BQ76920 TS1
#define PIN_ALERT PIN_PA6              // BQ76920 ALERT
#define PIN_INTERRUPT PIN_PA2          // BQ25798 Interrupt
#define PIN_CE_N PIN_PA5               // BQ25798 /Charge Enable (Active low)
#define PIN_PULL_SENSE_LOW_OLD PIN_PC3 // Pin used on some older PCBs for pulling sense low
#define PIN_PIN_PULL_SENSE_LOW PIN_PC1 // Pin use on newer PCBs for pulling sense low
#define PIN_EN_HEATER PIN_PC0          // Pin used to turn on the internal trace heater
#define PIN_SENSE_HEATER PIN_PA1       // Pin used to sense that the heater switch is closed/soldered on.
#define PIN_BUZZER PIN_PA3

uint16_t batteryId = 0; // Battery box ID read from the EEPROM at startup, sent in the status payload.
uint32_t seconds = 0;   // Don't need to worry about an overflow for this as it will last
                        // (2^32-1)/60/60/24/365 = 136 Years at 1 tick per second
uint32_t lastChargerUpdateSeconds = 0;
uint32_t lastBalancerUpdateSeconds = 0;
uint32_t lastChargerWDTSeconds = 0;
uint32_t lastProtectionUpdateSeconds = (uint32_t)0 - 6; // Fire on the first loop iteration.
uint32_t lastStatusLogSeconds = 0;
uint32_t lastReconfigureSeconds = 0;
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
    Serial.flush();
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
    // ======== Setup WDT ========
    wdt_reset();
    _PROTECTED_WRITE(WDT.CTRLA, WDT_PERIOD_2KCLK_gc);

    // ======== Set Pins ========
    // Set pin modes and initial values.
    digitalWrite(PIN_CE_N, HIGH);
    pinMode(PIN_CE_N, OUTPUT);
    digitalWrite(PIN_LED, LOW);
    pinMode(PIN_LED, OUTPUT);
    digitalWrite(PIN_BUZZER, LOW);
    pinMode(PIN_BUZZER, OUTPUT);
    digitalWrite(PIN_EN_HEATER, LOW);
    pinMode(PIN_EN_HEATER, OUTPUT);

    pinMode(PIN_SENSE_HEATER, INPUT);
    pinMode(PIN_TS1, INPUT);
    pinMode(PIN_ALERT, INPUT);
    pinMode(PIN_INTERRUPT, INPUT_PULLUP);

    // ======== Pull sense low detect ========
    // Because of different versions of the board we need to first find out if PIN_PULL_SENSE_LOW_OLD and
    // PIN_PULL_SENSE_LOW are wired together. If we don't do this we risk having shorting PIN_PULL_SENSE_LOW to ground
    // through PIN_PULL_SENSE_LOW_OLD. To do this we will set PIN_PULL_SENSE_LOW_OLD as INPUT_PULLIP and drive
    // PIN_PULL_SENSE_LOW LOW. If PIN_PULL_SENSE_LOW_OLD is LOW then we know they are connected and we will set
    // PIN_PULL_SENSE_LOW_OLD as INPUT, no pullup. If PIN_PULL_SENSE_LOW_OLD is HIGH then we know they are not connected
    // and we will set PIN_PULL_SENSE_LOW_OLD to LOW to disable the MOSFET that drives the sense low.
    pinMode(PIN_PULL_SENSE_LOW_OLD, INPUT_PULLUP);
    pinMode(PIN_PIN_PULL_SENSE_LOW, OUTPUT);
    digitalWrite(PIN_PIN_PULL_SENSE_LOW, LOW);
    delayMicroseconds(10);
    if (digitalRead(PIN_PULL_SENSE_LOW_OLD) == LOW) {
        pinMode(PIN_PULL_SENSE_LOW_OLD, INPUT);
    } else {
        pinMode(PIN_PULL_SENSE_LOW_OLD, OUTPUT);
        digitalWrite(PIN_PULL_SENSE_LOW_OLD, LOW);
    }

    // ======== Setup Serial ========
    // Setup serial interface. Always on: it's used for both debug log codes (gated per-file by
    // DEBUGGING in util.h) and the periodic status snapshot in loop(), which is unconditional.
    Serial.begin(9600);

    // Mirror inverted UART TX onto PIN_PULL_SENSE_LOW using CCL LUT1.
    // LUT1 output pin options: PA7 (default) or PIN_PULL_SENSE_LOW (alternative, PORTMUX.CTRLA bit5=1).
    // PA7 is already used as PIN_TS1, so we use the alternative PIN_PULL_SENSE_LOW.
    // USART0 TXD is only available on INSEL1 (0xA); INSEL0 0xA gives XCK instead.
    // TRUTH1 = 0x01 = NOT(IN1): IN1=0 (TX low) → output HIGH, IN1=1 (TX high) → output LOW.
    // IN0 and IN2 are masked (tied low), so only rows 0 and 2 of the truth table are reachable.
    CCL.CTRLA = 0;     // Disable CCL peripheral (section 28.5.1: ENABLE bit)
    CCL.LUT1CTRLA = 0; // Disable LUT1; required before writing enable-protected registers (section 28.3.1, 28.5.3)
    PORTMUX.CTRLA |= (1 << 5); // LUT1 output to alternative pin PIN_PULL_SENSE_LOW (section 15.3.1)
    PORTC.DIR |= PIN1_bm; // Set PIN_PULL_SENSE_LOW as output direction (section 16.5.1: DIR register); OUTEN overrides
                          // PORT I/O controller anyway (section 28.5.3: OUTEN bit description)
    CCL.LUT1CTRLB =
        CCL_INSEL1_USART0_gc; // INSEL1[3:0]=0xA: USART0 TXD; INSEL0[3:0]=0x0: MASK (section 28.5.4, Table 5-1)
    CCL.LUT1CTRLC = 0;        // INSEL2[3:0]=0x0: MASK (section 28.5.5)
    CCL.TRUTH1 = 0x01; // We want to match when IN[2-0] are all 0, so that means just TRUTH[0] is True, meaning register
                       // value of 0x01 (section 28.3.2.2, Table 28-2)
    CCL.LUT1CTRLA =
        CCL_OUTEN_bm |
        CCL_ENABLE_bm; // OUTEN (bit 3): drive PIN_PULL_SENSE_LOW from LUT; ENABLE (bit 0): enable LUT1 (section 28.5.3)
    CCL.CTRLA = CCL_ENABLE_bm; // Enable CCL peripheral (section 28.3.2.1)

    // ======== Setup Buzzer ========
    buzzer_pin_init();
    buzzer_beep();

    // ======= Setup I2C ========
    Wire.begin();

    // ======= Setup EEPROM ========
    if (!eeprom.begin()) {
        logCode(LOG_MAIN_EEPROM_NOT_FOUND);
        restart(ERROR_NO_EEPROM_DATA);
    }
    logCode(LOG_MAIN_EEPROM_FOUND);
    EepromData eepromData = {};
    switch (eeprom.readData(&eepromData)) {
    case EEPROM_OK:
        break;
    case EEPROM_CRC_ERR:
        logCode(LOG_MAIN_EEPROM_CRC_ERR);
        restart(ERROR_NO_EEPROM_DATA);
        break;
    case EEPROM_VERSION_ERR:
        logCodeU8(LOG_MAIN_EEPROM_BAD_VER, eepromData.version);
        restart(ERROR_NO_EEPROM_DATA);
        break;
    default:
        logCode(LOG_MAIN_EEPROM_NOT_FOUND);
        restart(ERROR_NO_EEPROM_DATA);
        break;
    }
    batteryId = eepromData.id;
    logCodeU16(LOG_MAIN_BATTERY_ID, eepromData.id);
    logCodeBytes(LOG_MAIN_PCB_VERSION, (uint8_t *)&eepromData.pcb, 3);
    if (!eeprom.isCompatible(eepromData.pcb)) {
        logCodeBytes(LOG_MAIN_PCB_INCOMPAT, (uint8_t *)&eepromData.pcb, 3);
        restart(ERROR_FIRMWARE_NOT_COMPATIBLE_WITH_PCB);
    }

    // ====== Setup BQ25798 ========
    // Try to find the BQ25798 (MPPT charger)
    // PCB revisions before 0.3.0 have 5k/30k NTC divider resistors; 0.3.0 onwards has 5.23k/30.9k.
    bool pcbV2 = pcbAtLeast(eepromData.pcb, 0, 3, 0);
    float ntcR1 = pcbV2 ? BQ25798_NTC_R1_OHMS_V2 : BQ25798_NTC_R1_OHMS_V1;
    float ntcR2 = pcbV2 ? BQ25798_NTC_R2_OHMS_V2 : BQ25798_NTC_R2_OHMS_V1;
    if (!charger.begin(PIN_CE_N, ntcR1, ntcR2)) {
        logCode(LOG_MAIN_BQ25798_NOT_FOUND);
        restart(ERROR_MISSING_BQ25798);
    }
    logCode(LOG_MAIN_BQ25798_FOUND);
    waitUntilNextBeep();
    buzzer_beep();

    // Check that ther is power in
    BQ25798ADC chargerADC = {};
    charger.readADCAll(chargerADC);
    if (chargerADC.vbus_mv <= 1000) {
        // Input voltage is too low.
        restart(ERROR_LOW_INPUT_VOLTAGE);
    }

    // ====== Check that the Heater Switch is connected ========
    // If the heater switch is not connected we would read 0V here.
    // If it is soldered (and closed) it will read Vin / 11. At a 5V in that gives (5/11)/3.3*1023 = 140.
    // For this check to pass power needs to be connected.
    if (analogRead(PIN_SENSE_HEATER) < 10) {
        waitUntilNextBeep();
        restart(ERROR_MISSING_THERMAL_SWITCH);
    }

    // ======== First Temp/Humidity Checks ========
    // In this first check we read the temperature and humidity sensors that are available.
    // The balancer might not yet be available so we don't error if we can't get its temperature reading.
    // In the second temperature and humidity check we will make sure that we can read the balancer temperature.
    // We do this first check so we can not turn on the charger if the temperature is out of range.

    // Try to find the temperature/humidity sensor (AHT20 or HDC2080, depending on PCB version).
    if (!tempHumidity.begin(eepromData.pcb)) {
        logCode(tempHumidity.usingHdc2080() ? LOG_MAIN_HDC2080_NOT_FOUND : LOG_MAIN_AHT20_NOT_FOUND);
        restart(ERROR_MISSING_TEMP_HUMIDITY_SENSOR);
    }
    logCode(tempHumidity.usingHdc2080() ? LOG_MAIN_HDC2080_FOUND : LOG_MAIN_AHT20_FOUND);

    setupTempAndHumidityCheck(true);
    waitUntilNextBeep();
    buzzer_beep();

    // ========= Find the BQ76920 (cell balancer) =========
    findCells();
    waitUntilNextBeep();
    buzzer_beep();

    // ======== Second Temp/Humidity Checks ========
    // In this second check we read the temperature and humidity sensors from all sensors.
    // The balancer temperature reading is now mandatory.
    setupTempAndHumidityCheck(false);
    waitUntilNextBeep();
    buzzer_beep();

    // ======== Setup interrupts ========
    attachInterrupt(digitalPinToInterrupt(PIN_ALERT), balancerInterrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(PIN_INTERRUPT), chargerInterrupt, FALLING);
    setupPIT();

    // ======== Play setup finished noise ========
    waitUntilNextBeep();
    waitUntilNextBeep();
    start_up_buzz();

    // ======== Enable sleep mode ========
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

    // If the balancer lost power (e.g. during a short circuit), try to re-initialize it.
    if (!balancer.found) {
        logCode(LOG_MAIN_BQ76920_NOT_FOUND);
        wakeUpBalancer();
        if (balancer.begin()) {
            logCode(LOG_MAIN_BQ76920_FOUND);
        }
    }

    // Re-apply the charger and balancer configuration every 10 minutes. If either chip has reset
    // itself (brownout, its own watchdog) it will be sitting on power-on defaults while still
    // answering on I2C, so nothing else here would notice. Rewriting the registers on a timer means
    // it recovers on its own instead of running on the wrong settings until the next restart.
    if (seconds - lastReconfigureSeconds >= 600) {
        lastReconfigureSeconds = seconds;
        // Safe in either mode: the BQ76920 is never put into ship mode, and begin() does not touch
        // SYS_CTRL2 so the CHG/DSG state is left alone.
        balancer.begin();
        // Only in main mode. In sleep mode the BQ25798 is deliberately held in ship mode and init()
        // would wake it back up and drain the battery. sleepMode() already re-inits it on wake.
        if (!sleepModeEnabled) {
            charger.disable();
            charger.init();
            charger.enable();
        }
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

        uint8_t heaterOn = protectionState.isHeatingEnabled() ? 1 : 0;

        // Assemble the 40-byte payload contiguously so it can be CRC'd, then
        // send: LOG_STATUS, payload, CRC-16 (little-endian). The Go reader
        // (battery_serial.go) verifies the CRC and drops corrupt/misframed
        // messages.
        uint8_t payload[40];
        size_t o = 0;
        payload[o++] = FW_VERSION_BYTE; // major*100 + minor*10 + patch, see version.h
        memcpy(payload + o, &batteryId, 2);
        o += 2;
        memcpy(payload + o, &seconds, 4);
        o += 4;
        memcpy(payload + o, &tempAht, 2);
        o += 2;
        memcpy(payload + o, &tempBq76920, 2);
        o += 2;
        memcpy(payload + o, &tempBq25798, 2);
        o += 2;
        payload[o++] = humPct;
        memcpy(payload + o, &cellMv[0], 2);
        o += 2;
        memcpy(payload + o, &cellMv[1], 2);
        o += 2;
        memcpy(payload + o, &cellMv[2], 2);
        o += 2;
        memcpy(payload + o, &chargerADC.vbus_mv, 2);
        o += 2;
        memcpy(payload + o, &chargerADC.ibus_ma, 2);
        o += 2;
        memcpy(payload + o, &chargerADC.vbat_mv, 2);
        o += 2;
        memcpy(payload + o, &chargerADC.ibat_ma, 2);
        o += 2;
        memcpy(payload + o, &ibat_cc_ma, 2);
        o += 2; // BQ76920 CC current
        memcpy(payload + o, chgStat, 5);
        o += 5; // REG1B..1F
        memcpy(payload + o, bqStat, 4);
        o += 4; // SYS_STAT,CELLBAL1,CTRL1,CTRL2
        payload[o++] = heaterOn;

        uint16_t crc = crc16CCITT(payload, sizeof(payload));

        Serial.write(LOG_STATUS);
        Serial.write(payload, sizeof(payload));
        Serial.write((uint8_t *)&crc, 2);
    }

    // Make sure we flush the serial buffer before going to sleep.
    Serial.flush();

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

void restart(uint8_t errorCode) {
    // Disable the charger.
    digitalWrite(PIN_CE_N, HIGH);
    // Cut off the output. The balancer might not be available yet so these will just fail silently. That is OK.
    balancer.disableCharging();
    balancer.disableDischarging();
    logCode(LOG_MAIN_RESTARTING);
    Serial.flush();
    waitUntilNextBeep();
    buzzer_on(500);
    wdt_reset();
    delay(400);
    buzzer_on(200);
    wdt_reset();
    delay(400);
    buzzer_off();
    wdt_reset();
    delay(400);

    for (int i = 0; i < errorCode; i++) {
        buzzer_on(500);
        wdt_reset();
        delay(300);
        buzzer_off();
        wdt_reset();
        delay(300);
    }
    wdt_reset();
    delay(1000);

    cli();
    _PROTECTED_WRITE(RSTCTRL.SWRR, RSTCTRL_SWRE_bm);
    while (1) {
    }
}

// Check the temperature and humidity are in the acceptable ranges for the setup process.
// If they are not then the system will restart.
// Checking the balancer might be optional as it might not yet be present in the setup process.
void setupTempAndHumidityCheck(bool balancerOptional) {
    // Trigger the first measurement now so it completes during the remaining startup sequence.
    tempHumidity.trigger();
    waitUntilNextBeep();
    buzzer_beep();

    if (!tempHumidity.readResult()) {
        logCode(LOG_MAIN_TEMP_HUM_FAIL);
        waitUntilNextBeep();
        restart(ERROR_TEMP_READING_FAILED);
    }
    float tempHumTemp = tempHumidity.temperature();
    float chargerTemp = charger.readTemp();

    // If the balancer is optional only read it if it is found.
    float balancerTemp = balancerOptional ? chargerTemp : balancer.readTemp();
    if (balancerOptional && balancer.begin()) {
        balancerTemp = balancer.readTemp();
    }

    logCode3I16(LOG_MAIN_TEMPS, int16_t(tempHumTemp * 10), int16_t(balancerTemp * 10), int16_t(chargerTemp * 10));

    float tMin = min(tempHumTemp, min(balancerTemp, chargerTemp));
    float tMax = max(tempHumTemp, max(balancerTemp, chargerTemp));
    if (tMin < 5.0f) {
        logCode(LOG_MAIN_TEMP_OOR);
        waitUntilNextBeep();
        restart(ERROR_TEMP_TOO_LOW);
    }
    if (tMax > 35.0f) {
        logCode(LOG_MAIN_TEMP_OOR);
        waitUntilNextBeep();
        restart(ERROR_TEMP_TOO_HIGH);
    }
    if (tMax - tMin > 10.0f) {
        logCode(LOG_MAIN_TEMP_MISMATCH);
        waitUntilNextBeep();
        restart(ERROR_TEMP_DONT_MATCH);
    }
}

void findCells() {
    // For the initial power up we need to enable the charger before we can find the balancer and the cells.
    // This is because the individual cell protection disables output until a voltage is applied.
    // Because we don't want to charge the cells a lot we reduce the charge current output to 50mA.
    // So we will enable the charger at 50mA and then find the balancer and cells. This can take a couple of seconds.

    // First lets see if we can already find the cells and balancer.
    bool balancerFound = balancer.begin();
    bool cellsFound = balancerFound && balancer.properCellPopulation();
    if (cellsFound) {
        // Cells (and balancer) are already available so we don't need to power up the charger and can just exit.
        return;
    }

    // Set the charger to a reduced charging current of 50mA (lowest current supported from chip) and power it up for 2
    // seconds.
    charger.minimumCharging();
    digitalWrite(PIN_CE_N, LOW); // Enable charger
    delay(1000);
    waitUntilNextBeep();
    buzzer_beep();
    delay(1000);
    waitUntilNextBeep();
    buzzer_beep();
    digitalWrite(PIN_CE_N, HIGH);
    charger.maximumCharging();

    if (!balancerFound) {
        if (balancer.begin()) {
            logCode(LOG_MAIN_BQ76920_NOT_FOUND);
            restart(ERROR_MISSING_BQ76920);
        }
    }

    if (balancer.properCellPopulation()) {
        logCode(LOG_MAIN_CELL_POP_FAIL);
        restart(ERROR_MISSING_CELLS);
    }

    if (!charger.vbatPresent()) {
        logCode(LOG_MAIN_CELL_POP_FAIL);
        restart(ERROR_MISSING_CELLS);
    }
}
