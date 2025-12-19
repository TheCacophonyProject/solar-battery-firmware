#include "util.h"

namespace {
  volatile uint32_t pitCount = 0;  
}

// Setup the RTC_PIT_interrupt to trigger every  second.
void setupPIT() {
  RTC.CLKSEL = RTC_CLKSEL_INT32K_gc; // Set clock source to the internal 32.768kHz oscillator
  RTC.PITCTRLA = RTC_PITEN_bm | RTC_PERIOD_CYC32768_gc; // Enable PIT and set period to 1 second
  RTC.PITINTCTRL = RTC_PI_bm; // Enable PIT interrupt
}

ISR(RTC_PIT_vect) {
  noInterrupts();
  RTC.PITINTFLAGS = RTC_PI_bm; // Clear interrupt flag, otherwise it will constantly trigger.
  pitCount += 1;
  interrupts();
}

uint32_t getSeconds() {
  noInterrupts();
  uint32_t seconds = pitCount;
  interrupts();
  return seconds;
}
