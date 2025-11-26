#include <main.h>
#include <Arduino.h>
#include <avr/sleep.h>
#include <avr/wdt.h>
#include <avr/interrupt.h>
#include <Wire.h>

#include "bq25798.h"
#include "bq76920.h"

#include <Adafruit_BQ25798.h>
Adafruit_BQ25798 bq;


#define WDT_DURATION WDTO_2S

BQ25798 charger;
BQ76920 balancer;

//#define PIN_I2C_SCL PIN_PB0
//#define PIN_I2C_SDA PIN_PB1

//#define PIN_TX PIN_PB2
//#define PIN_RX PIN_PB3

#define PIN_LED PIN_PB5
#define PIN_TS1 PIN_PA7       // BQ76920 TS1
#define PIN_ALERT PIN_PA6     // BQ76920 ALERT
#define PIN_INTERRUPT PIN_PA4 // BQ25798 Interrupt
#define PIN_CE PIN_PA5        // BQ25798 ~CHarge Enable

volatile unsigned long pitTime = 0; // Don't need to worry about an overflow for this as it will last (2^32-1)/60/60/24/365 = 136 Years at 1 tick per second

void setupPIT() {
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // Set clock source to the internal 32.768kHz oscillator
  RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC32768_gc; // Enable PIT and set period to 1 second
  RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupt
}

ISR(RTC_PIT_vect) {
  RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag, otherwise it will constantly trigger.
  pitTime += 1;
}

unsigned long getPitTime() {
  noInterrupts();
  unsigned long time = pitTime;
  interrupts();
  return time;
}

void setup() {
  // Setup WDT
  wdt_reset();
  wdt_disable();
  wdt_enable(WDT_DURATION);

  // Set pins modes
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TS1, INPUT);
  pinMode(PIN_ALERT, INPUT);
  pinMode(PIN_CE, OUTPUT);

  // Set pin initial states
  ledOff();
  //disableCharger();

  // Setup debug
  Serial.begin(9600);
  Serial.println("");
  Serial.println("Starting...");

  // Setup i2C
  Wire.begin();

  // Try to find the BQ25798 (MPPT charger)
  if (!charger.begin()) {
    Serial.println(F("Could not find the BQ25798"));
    // Will get restarted from the WDT
    while (1);
  }
  Serial.println(F("Charger BQ25798 found!"));

  // Try to find the BQ76920 (cell balancer)
  if (!balancer.begin()) {
    Serial.println("Could not see BQ76920, will try driving TS1 low to try enabling it.");
    // Set to output, drive high, then back to input.
    pinMode(PIN_TS1, OUTPUT);
    digitalWrite(PIN_TS1, HIGH);
    delay(100);
    pinMode(PIN_TS1, INPUT);
    delay(100);
    if (!balancer.begin()) {
      Serial.println(F("Could not find the BQ76920"));
      // Will get restarted from the WDT
      while (1);  
    }
  }
  Serial.println(F("Balancer BQ76920 found!"));

  
  balancer.updateBalanceRoutine();
  balancer.readStatus();
  balancer.readChargeDischargeBits();
  
  balancer.clearOVandUVTrip();
  balancer.enableChargeAndDischarge();
  
  Serial.println("Setting up PIT");
  setupPIT();
  Serial.println("Set up PIT");
  set_sleep_mode(SLEEP_MODE_PWR_DOWN);
  sleep_enable();
}

void enableCharger() {
  digitalWrite(PIN_CE, LOW);
}

void disableCharger() {
  digitalWrite(PIN_CE, HIGH);
}

void ledOn() {
  digitalWrite(PIN_LED, HIGH);
}

void ledOff() {
  digitalWrite(PIN_LED, LOW);
}


unsigned long lastChargerUpdateSeconds = 0;
unsigned long lastBalancerUpdateSeconds = 0;



void loop() {
  // Reset WDT to prevent it from triggering.
  wdt_reset(); 

  // Need to disable interrupts while we read the time as it is changed in the PIT interrupt
  noInterrupts();
  unsigned long seconds = pitTime;
  interrupts();

  if (seconds - lastChargerUpdateSeconds > 5) {
    //Serial.println("Charger chip update");
    lastChargerUpdateSeconds = seconds;
    
    // Force source retry for the charger. (If it fails it will not retry for about 6 minutes but we want it to check more often.)
    // TODO: Only force source retry when the source has failed.
    charger.sourceRetry();
    
    // Enable MPPT again (if the source fails, MPPT gets disabled so we need to enable it again)
    // TODO: Log when it actually updates the state rather than just enabling it always
    charger.enableMPPT();

    // Set VOC (how often it checks the open circuit voltage) rate to 30s
    // TODO: Reduce the verbose of the logging in the checkStatus
    charger.checkStatus();
  }

  if (seconds - lastBalancerUpdateSeconds > 3) {
    lastBalancerUpdateSeconds = seconds;
    // TODO, if temperature is out of range, disable or reduce charging.
    balancer.ReadTemp();
  }

  ledOn();
  delay(1);
  ledOff();

  Serial.flush();
  sleep_mode();
}

