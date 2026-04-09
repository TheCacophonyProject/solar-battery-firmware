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

#define SERIAL_ENABLE true

// ── Binary log helpers ────────────────────────────────────────────────────────
// Send a one-byte code, optionally followed by a little-endian payload.
// All functions are no-ops when SERIAL_ENABLE == 0.
#if SERIAL_ENABLE
inline void logCode(uint8_t code) { Serial.write(code); }
inline void logCodeU16(uint8_t code, uint16_t val) {
    Serial.write(code);
    Serial.write((uint8_t *)&val, 2);
}
inline void logCodeI16(uint8_t code, int16_t val) {
    Serial.write(code);
    Serial.write((uint8_t *)&val, 2);
}
// payload: u8 a, u16 b, u16 c
inline void logCodeU8U16U16(uint8_t code, uint8_t a, uint16_t b, uint16_t c) {
    Serial.write(code);
    Serial.write(a);
    Serial.write((uint8_t *)&b, 2);
    Serial.write((uint8_t *)&c, 2);
}
// payload: u8 a
inline void logCodeU8(uint8_t code, uint8_t a) {
    Serial.write(code);
    Serial.write(a);
}
inline void logCodeU8U8(uint8_t code, uint8_t a, uint8_t b) {
    Serial.write(code);
    Serial.write(a);
    Serial.write(b);
}
// payload: i16 a, u16 b
inline void logCodeI16U16(uint8_t code, int16_t a, uint16_t b) {
    Serial.write(code);
    Serial.write((uint8_t *)&a, 2);
    Serial.write((uint8_t *)&b, 2);
}
// payload: N bytes
inline void logCodeBytes(uint8_t code, const uint8_t *data, uint8_t len) {
    Serial.write(code);
    Serial.write(data, len);
}
// payload: i16 a, i16 b, i16 c
inline void logCode3I16(uint8_t code, int16_t a, int16_t b, int16_t c) {
    Serial.write(code);
    Serial.write((uint8_t *)&a, 2);
    Serial.write((uint8_t *)&b, 2);
    Serial.write((uint8_t *)&c, 2);
}
#else
inline void logCode(uint8_t) {}
inline void logCodeU16(uint8_t, uint16_t) {}
inline void logCodeI16(uint8_t, int16_t) {}
inline void logCodeU8U16U16(uint8_t, uint8_t, uint16_t, uint16_t) {}
inline void logCodeU8(uint8_t, uint8_t) {}
inline void logCodeU8U8(uint8_t, uint8_t, uint8_t) {}
inline void logCodeI16U16(uint8_t, int16_t, uint16_t) {}
inline void logCodeBytes(uint8_t, const uint8_t *, uint8_t) {}
inline void logCode3I16(uint8_t, int16_t, int16_t, int16_t) {}
#endif

#endif
