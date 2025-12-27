#include <Arduino.h>

typedef struct {
    float tempC;      // degrees C
    float resKohm;   // resistance in kΩ
} NtcPoint;

// NCP□XH103 10k, B = 3380K
// Temp (°C) , Resistance (kΩ)
static const NtcPoint ntcTable[] = {
    { -40.0f, 195.652f },
    { -35.0f, 148.171f },
    { -30.0f, 113.347f },
    { -25.0f,  87.559f },
    { -20.0f,  68.237f },
    { -15.0f,  53.650f },
    { -10.0f,  42.506f },
    {  -5.0f,  33.892f },
    {   0.0f,  27.219f },
    {   5.0f,  22.021f },
    {  10.0f,  17.926f },
    {  15.0f,  14.674f },
    {  20.0f,  12.081f },
    {  25.0f,  10.000f },
    {  30.0f,   8.315f },
    {  35.0f,   6.948f },
    {  40.0f,   5.834f },
    {  45.0f,   4.917f },
    {  50.0f,   4.161f },
    {  55.0f,   3.535f },
    {  60.0f,   3.014f },
    {  65.0f,   2.586f },
    {  70.0f,   2.228f },
    {  75.0f,   1.925f },
    {  80.0f,   1.669f },
    {  85.0f,   1.452f },
    {  90.0f,   1.268f },
    {  95.0f,   1.110f },
    { 100.0f,   0.974f },
    { 105.0f,   0.858f },
    { 110.0f,   0.758f },
    { 115.0f,   0.672f },
    { 120.0f,   0.596f },
    { 125.0f,   0.531f },
};

#define NTC_TABLE_SIZE (sizeof(ntcTable) / sizeof(ntcTable[0]))

// Convert resistance (ohms) to temperature (°C)
float ntcTempFromResistance(uint32_t rOhms)
{
    //Serial.println(r_ohms);
    if (rOhms <= 0.0f) {
        return NAN;
    }

    float rKohm = rOhms / 1000.0f;

    // Clamp outside table range
    if (rKohm >= ntcTable[0].resKohm) {
        return ntcTable[0].tempC;                             // <= -40°C
    }
    if (rKohm <= ntcTable[NTC_TABLE_SIZE - 1].resKohm) {
        return ntcTable[NTC_TABLE_SIZE - 1].tempC;            // >= 125°C
    }

    // Find the two surrounding points (R decreases as T increases)
    uint8_t i;
    for (i = 0; i < (uint8_t)NTC_TABLE_SIZE - 1; i++) {
        float r1 = ntcTable[i].resKohm;
        float r2 = ntcTable[i + 1].resKohm;

        if (rKohm <= r1 && rKohm >= r2) {
            // Log-linear interpolation in resistance
            float logR  = logf(rKohm);
            float logR1 = logf(r1);
            float logR2 = logf(r2);

            float t = (logR - logR1) / (logR2 - logR1);  // 0..1
            float temp = ntcTable[i].tempC +
                         t * (ntcTable[i + 1].tempC - ntcTable[i].tempC);
            return temp;
        }
    }

    // Should never get here if table is monotonic
    return NAN;
}
