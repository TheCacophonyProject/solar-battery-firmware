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

// TODO: Review this code for the buzzer
#ifndef F_CPU
#define F_CPU 20000000UL
#endif

// Buzzer MOSFET gate on PA3
static constexpr uint8_t BUZZER_bm = PIN3_bm;

// We’ll use TCB1 to generate interrupts at 2*freq (toggle each interrupt => freq output)
static volatile bool buzzer_running = false;

static inline void buzzer_pin_init() {
  // PA3 output
  PORTA.DIRSET = BUZZER_bm;
  // start low
  PORTA.OUTCLR = BUZZER_bm;
}

static inline void tcb1_stop() {
  TCB1.CTRLA = 0;               // disable
  TCB1.INTCTRL = 0;             // disable interrupt
  TCB1.INTFLAGS = TCB_CAPT_bm;  // clear pending
  buzzer_running = false;
  PORTA.OUTCLR = BUZZER_bm;     // ensure off
}

static bool tcb1_start_toggle(uint32_t freq_hz) {
  if (freq_hz == 0) return false;

  // Choose a prescaler that makes TOP fit in 16 bits.
  // We need interrupt rate = 2*freq_hz (because we toggle each time).
  struct PrescChoice {
    uint16_t div;
    uint8_t  clksel_gc;
  };

  const PrescChoice choices[] = {
    {1,   TCB_CLKSEL_DIV1_gc},
    {2,   TCB_CLKSEL_DIV2_gc},
  };

  uint16_t top = 0;
  uint8_t clksel = 0;
  bool found = false;

  for (auto c : choices) {
    // TOP = F_CPU / (presc * (2*freq)) - 1
    uint32_t denom = (uint32_t)c.div * (uint32_t)(2UL * freq_hz);
    uint32_t t = (F_CPU / denom);
    if (t == 0) continue;
    if (t - 1 <= 0xFFFF) {
      top = (uint16_t)(t - 1);
      clksel = c.clksel_gc;
      found = true;
      break;
    }
  }

  if (!found) return false;

  // Stop + reset configuration
  TCB1.CTRLA = 0;
  TCB1.CTRLB = 0;
  TCB1.INTCTRL = 0;
  TCB1.INTFLAGS = TCB_CAPT_bm;

  // Periodic Interrupt mode: interrupt when counter reaches TOP, then restarts. :contentReference[oaicite:3]{index=3}
  TCB1.CTRLB = TCB_CNTMODE_INT_gc;

  // Set TOP in CCMP (Periodic Interrupt uses CCMP as TOP)
  TCB1.CCMP = top;

  // Enable interrupt on CAPT flag (Periodic Interrupt sets CAPT when reaching TOP). :contentReference[oaicite:4]{index=4}
  TCB1.INTCTRL = TCB_CAPT_bm;

  // Enable timer with chosen prescaler
  TCB1.CTRLA = clksel | TCB_ENABLE_bm;

  buzzer_running = true;
  return true;
}

ISR(TCB1_INT_vect) {
  // Clear interrupt flag (write 1 to clear). :contentReference[oaicite:5]{index=5}
  TCB1.INTFLAGS = TCB_CAPT_bm;

  // Toggle PA3 output. :contentReference[oaicite:6]{index=6}
  PORTA.OUTTGL = BUZZER_bm;
}

void buzzer_on(uint32_t freq_hz) {
  buzzer_pin_init();
  cli();
  tcb1_stop();
  bool ok = tcb1_start_toggle(freq_hz);
  sei();

  if (!ok) {
    println("Couldn’t fit TOP value (frequency too low/high for chosen prescalers)");
    tcb1_stop();
  }
}

void buzzer_off() {
  cli();
  tcb1_stop();
  sei();
}
