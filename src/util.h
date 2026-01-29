#ifndef UTIL_H
#define UTIL_H

#include <Arduino.h>

void setupPIT();
uint32_t getSeconds();
void buzzer_on(uint32_t freq_hz);
void buzzer_off();
void buzzer_pin_init();
void start_up_buzz();
void buzzer_beep();

#define SERIAL_ENABLE 1

#if SERIAL_ENABLE
#define println(...) Serial.println(__VA_ARGS__)
#define print(...) Serial.print(__VA_ARGS__)
/*
RAM:   [==        ]  16.5% (used 338 bytes from 2048 bytes)
Flash: [========= ]  92.3% (used 15130 bytes from 16384 bytes)
*/
#else
#define println(...)                                                                                                   \
    do {                                                                                                               \
    } while (0)
#define print(...)                                                                                                     \
    do {                                                                                                               \
    } while (0)
/*
RAM:   [==        ]  16.5% (used 338 bytes from 2048 bytes)
Flash: [======    ]  57.1% (used 9351 bytes from 16384 bytes)
*/
#endif

#endif
