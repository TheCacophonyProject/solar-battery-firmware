#ifndef UTIL_H
#define UTIL_H

#include "log_codes.h"
#include <Arduino.h>

void setupPIT();
uint32_t getSeconds();

// CRC-16/CCITT-FALSE (poly 0x1021, init 0xFFFF, no reflection). Must match
// crc16CCITT() in the Go reader (battery_serial.go).
inline uint16_t crc16CCITT(const uint8_t *data, size_t len) {
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t b = 0; b < 8; b++) {
            if (crc & 0x8000) {
                crc = (crc << 1) ^ 0x1021;
            } else {
                crc <<= 1;
            }
        }
    }
    return crc;
}
void buzzer_on(uint32_t freq_hz);
void buzzer_off();
void buzzer_pin_init();
void start_up_buzz();
void buzzer_beep();

// DEBUGGING gates the logCode* helpers below, not Serial itself — Serial is always used (the
// periodic status snapshot in main.cpp's loop() sends over it every 10 seconds regardless). The
// logCode* calls are the verbose one-off/debug codes (LOG_MAIN_*, LOG_PROT_*, LOG_BQ_*, etc.),
// which cost enough flash across all the .cpp files that enabling them everywhere doesn't fit on
// the ATtiny1616.
//
// DEBUGGING is per translation unit: it defaults to off, but any .cpp file can opt in by defining
// it before including this header, e.g. to enable logging in main.cpp only:
//
//   #define DEBUGGING 1
//   #include "util.h"
//
// This works because the helpers below are declared `static inline`, so each .cpp file that
// includes this header gets its own private copy rather than one shared external-linkage
// definition — otherwise the linker would only keep one of the (possibly differing) definitions
// across translation units, which is undefined behaviour (ODR violation).
#ifndef DEBUGGING
#define DEBUGGING 0
#endif

// ── Binary log helpers ────────────────────────────────────────────────────────
// Send a one-byte code, optionally followed by a little-endian payload.
// All functions are no-ops in a file where DEBUGGING == 0.
#if DEBUGGING
static inline void logCode(uint8_t code) { Serial.write(code); }
static inline void logCodeU16(uint8_t code, uint16_t val) {
    Serial.write(code);
    Serial.write((uint8_t *)&val, 2);
}
static inline void logCodeI16(uint8_t code, int16_t val) {
    Serial.write(code);
    Serial.write((uint8_t *)&val, 2);
}
static inline void logCodeU8U8(uint8_t code, uint8_t a, uint8_t b) {
    Serial.write(code);
    Serial.write(a);
    Serial.write(b);
}
static inline void debug(uint8_t id, uint8_t value) { logCodeU8U8(LOG_DEBUG, id, value); }
// payload: u8 a, u16 b, u16 c
static inline void logCodeU8U16U16(uint8_t code, uint8_t a, uint16_t b, uint16_t c) {
    Serial.write(code);
    Serial.write(a);
    Serial.write((uint8_t *)&b, 2);
    Serial.write((uint8_t *)&c, 2);
}
// payload: u8 a
static inline void logCodeU8(uint8_t code, uint8_t a) {
    Serial.write(code);
    Serial.write(a);
}
// payload: i16 a, u16 b
static inline void logCodeI16U16(uint8_t code, int16_t a, uint16_t b) {
    Serial.write(code);
    Serial.write((uint8_t *)&a, 2);
    Serial.write((uint8_t *)&b, 2);
}
// payload: N bytes
static inline void logCodeBytes(uint8_t code, const uint8_t *data, uint8_t len) {
    Serial.write(code);
    Serial.write(data, len);
}
// payload: i16 a, i16 b, i16 c
static inline void logCode3I16(uint8_t code, int16_t a, int16_t b, int16_t c) {
    Serial.write(code);
    Serial.write((uint8_t *)&a, 2);
    Serial.write((uint8_t *)&b, 2);
    Serial.write((uint8_t *)&c, 2);
}
#else
static inline void logCode(uint8_t) {}
static inline void logCodeU16(uint8_t, uint16_t) {}
static inline void logCodeI16(uint8_t, int16_t) {}
static inline void logCodeU8U8(uint8_t, uint8_t, uint8_t) {}
static inline void debug(uint8_t, uint8_t) {}
static inline void logCodeU8U16U16(uint8_t, uint8_t, uint16_t, uint16_t) {}
static inline void logCodeU8(uint8_t, uint8_t) {}
static inline void logCodeI16U16(uint8_t, int16_t, uint16_t) {}
static inline void logCodeBytes(uint8_t, const uint8_t *, uint8_t) {}
static inline void logCode3I16(uint8_t, int16_t, int16_t, int16_t) {}
#endif

#endif
