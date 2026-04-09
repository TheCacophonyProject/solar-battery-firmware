#ifndef LOG_CODES_H
#define LOG_CODES_H

// Binary log protocol — firmware sends a code byte, optionally followed by a
// little-endian payload. Keep this file in sync with the host-side decoder.
//
// Payload key:
//   i16 = int16_t  (2 bytes, little-endian)
//   u16 = uint16_t (2 bytes, little-endian)
//   u8  = uint8_t  (1 byte)

// ── Main (0x01–0x1F) ─────────────────────────────────────────────────────────
#define LOG_MAIN_STARTING           0x01  // Boot started
#define LOG_MAIN_BQ25798_FOUND      0x02  // BQ25798 found
#define LOG_MAIN_BQ25798_NOT_FOUND  0x03  // BQ25798 not found
#define LOG_MAIN_BQ76920_FOUND      0x04  // BQ76920 found
#define LOG_MAIN_BQ76920_NOT_FOUND  0x05  // BQ76920 not found
#define LOG_MAIN_AHT20_FOUND        0x06  // AHT20 found
#define LOG_MAIN_AHT20_NOT_FOUND    0x07  // AHT20 not found
#define LOG_MAIN_AHT20_FAIL         0x08  // AHT20 read failed at startup
#define LOG_MAIN_TEMPS              0x09  // Startup temps;  payload: i16 aht, i16 bal, i16 chg (all °C × 10)
#define LOG_MAIN_TEMP_OOR           0x0A  // Startup temp out of range
#define LOG_MAIN_TEMP_MISMATCH      0x0B  // Startup temps disagree > 10 °C
#define LOG_MAIN_CELL_POP_FAIL      0x0C  // Cell population check failed
#define LOG_MAIN_SLEEP_MODE         0x0D  // Entering sleep mode
#define LOG_MAIN_INPUT_SRC          0x0E  // Input source detected, waking
#define LOG_MAIN_NO_INPUT_SLEEP     0x0F  // No input for 20 s, sleeping
#define LOG_MAIN_POOR_SRC_SLEEP     0x10  // Poor source, sleeping
#define LOG_MAIN_RESTARTING         0x11  // Software restart
#define LOG_MAIN_CHARGER_INT        0x12  // Charger interrupt fired
#define LOG_MAIN_BALANCER_INT       0x13  // Balancer interrupt fired

// ── Protection (0x20–0x2F) ───────────────────────────────────────────────────
#define LOG_PROT_UV_RECOVERED   0x20  // Cell UV recovered
#define LOG_PROT_SCD            0x21  // Short circuit discharge
#define LOG_PROT_OCD            0x22  // Overcurrent discharge
#define LOG_PROT_OCD_SCD_CLEAR  0x23  // OCD/SCD fault cleared
#define LOG_PROT_CELL_OV        0x24  // Cell over voltage
#define LOG_PROT_CELL_UV        0x25  // Cell under voltage
#define LOG_PROT_CELL_UV_OV     0x26  // Cell UV and OV simultaneously
#define LOG_PROT_BQ76920_TEMP   0x27  // BQ76920 temp reading;   payload: i16 (°C × 10)
#define LOG_PROT_VBAT_OVP       0x28  // VBAT over voltage protection
#define LOG_PROT_BQ25798_T_ERR  0x29  // BQ25798 temperature fault
#define LOG_PROT_AHT20          0x2A  // AHT20 readings;         payload: i16 temp (°C × 10), u16 humidity (% × 10)
#define LOG_PROT_STATE          0x2B  // State change;           payload: u8 flags (bit0=healthy, bit1=chg, bit2=discharge, bit3=bal)
#define LOG_PROT_CHARGER_TEMP   0x2C  // BQ25798 NTC temp;       payload: i16 (°C × 10)

// ── AHT20 (0x50–0x5F) ────────────────────────────────────────────────────────
#define LOG_AHT_STATUS_FAIL            0x50  // Status read failed
#define LOG_AHT_INIT_FAIL              0x51  // Init command failed
#define LOG_AHT_STATUS_AFTER_INIT_FAIL 0x52  // Status read after init failed
#define LOG_AHT_NOT_CALIBRATED         0x53  // Not calibrated after init
#define LOG_AHT_TRIGGER_FAIL           0x54  // Trigger command failed
#define LOG_AHT_NO_TRIGGER             0x55  // readResult called without trigger
#define LOG_AHT_BUSY                   0x56  // Sensor still busy
#define LOG_AHT_CRC_ERR                0x57  // CRC mismatch
#define LOG_AHT_TIMEOUT                0x58  // Measurement timeout
#define LOG_AHT_STATUS                 0x59  // Status byte at begin(); payload: u8 status

// ── Util (0x60–0x6F) ─────────────────────────────────────────────────────────
#define LOG_UTIL_BUZZER_ERR     0x60  // Buzzer frequency out of range

// ── BQ25798 (0x70–0x7F) ──────────────────────────────────────────────────────
#define LOG_CHG_BAD_PART        0x70  // Bad part number;               payload: u8 regData
#define LOG_CHG_BAD_CELL_CNT    0x71  // Bad cell count
#define LOG_CHG_BAD_PWM_FREQ    0x72  // Bad PWM frequency
#define LOG_CHG_INIT            0x73  // Charger init started
#define LOG_CHG_DIS_HIZ         0x74  // HIZ mode disabled (source retry)
#define LOG_CHG_EN_MPPT         0x75  // MPPT enabled
#define LOG_CHG_DIS_VBUS_WK     0x76  // VBUS wakeup disabled
#define LOG_CHG_EN_VBUS_WK      0x77  // VBUS wakeup enabled
#define LOG_CHG_CHARGE_STATUS   0x78  // Charge status changed;         payload: u8 (0-7)
#define LOG_CHG_VBUS_STATUS     0x79  // VBUS status changed;           payload: u8 (0-0xF)
#define LOG_CHG_FAULT0          0x7A  // FAULT_Status_0 non-zero;       payload: u8
#define LOG_CHG_FAULT1          0x7B  // FAULT_Status_1 non-zero;       payload: u8
#define LOG_CHG_FLAGS           0x7C  // Flag regs REG22-REG27;         payload: 6×u8
#define LOG_CHG_WR_ERR          0x7D  // writeWord block write failed
#define LOG_CHG_RD_ERR          0x7E  // writeWord readback failed
#define LOG_CHG_WR_MISMATCH     0x7F  // writeWord value mismatch;      payload: u8 reg, u16 written, u16 read

// ── BQ76920 (0x30–0x4F) ──────────────────────────────────────────────────────
#define LOG_BQ_TEMP_OOR     0x30  // Temperature out of range;   payload: i16 (°C × 10)
#define LOG_BQ_CRC_ERR      0x31  // CRC mismatch (readReg)
#define LOG_BQ_CRC0_ERR     0x32  // CRC[0] mismatch (readBlock)
#define LOG_BQ_CRCN_ERR     0x33  // CRC[n] mismatch (readBlock)
#define LOG_BQ_CELL_MISSING 0x34  // Expected cell not detected
#define LOG_BQ_CELL_EXTRA   0x35  // Unexpected cell detected
#define LOG_BQ_CELL_V_ERR   0x36  // Cell voltage read failed
#define LOG_BQ_BAL_ON       0x37  // Balancing started
#define LOG_BQ_BAL_OFF      0x38  // Balancing stopped
#define LOG_BQ_BAL_DIFF     0x39  // Balance voltage diff;        payload: u16 (mV)
#define LOG_BQ_BAL_CELL     0x3A  // Balancing cell;              payload: u8 cell, u16 maxMV, u16 minMV
#define LOG_BQ_MIN_V        0x3B  // Min cell voltage (debug);    payload: u16 (mV)
#define LOG_BQ_OV_OOR       0x3C  // OV trip raw value out of range
#define LOG_BQ_UV_OOR       0x3D  // UV trip raw value out of range
#define LOG_BQ_OV_TRIP      0x3E  // OV trip voltage set;         payload: u16 (mV)
#define LOG_BQ_UV_TRIP      0x3F  // UV trip voltage set;         payload: u16 (mV)
#define LOG_BQ_OVUV_WR_ERR  0x40  // OV/UV register write failed
#define LOG_BQ_OVUV_RD_ERR  0x41  // OV/UV register read failed
#define LOG_BQ_PROT1_ERR    0x42  // PROTECT1 write failed
#define LOG_BQ_PROT2_ERR    0x43  // PROTECT2 write failed
#define LOG_BQ_PROT3_ERR    0x44  // PROTECT3 write failed
#define LOG_BQ_TEST_MAX_V   0x45  // Test mode: max cell voltage active
#define LOG_BQ_TEST_MIN_V   0x46  // Test mode: min cell voltage active
#define LOG_BQ_CELL_MV      0x47  // Cell mV (debug, sent once per cell 0-4); payload: u16 (mV)

// ── Debug (0xD0) ─────────────────────────────────────────────────────────────
#define LOG_DEBUG  0xD0  // Generic debug; payload: u8 id, u8 value

// ── I2C (0x80–0x8F) ──────────────────────────────────────────────────────────
#define LOG_I2C_QUEUE_ERR   0x80  // Failed to queue register byte in read()
#define LOG_I2C_ERR         0x81  // endTransmission error;  payload: u8 errCode

// ── Periodic status snapshot (0x90) ──────────────────────────────────────────
// Sent every 10 seconds. Payload (all little-endian, 34 bytes total):
//   u32 seconds          — time since boot
//   i16 temp_aht_x10     — AHT20 temperature  (°C × 10)
//   i16 temp_bq76920_x10 — BQ76920 NTC temp   (°C × 10)
//   i16 temp_bq25798_x10 — BQ25798 NTC temp   (°C × 10)
//   u8  humidity_pct     — AHT20 humidity      (whole %)
//   u16 cell1_mv         — Cell 1 voltage      (mV)
//   u16 cell2_mv         — Cell 2 voltage      (mV)
//   u16 cell3_mv         — Cell 3 voltage      (mV)
//   u16 vbus_mv          — Input voltage        (mV)
//   u16 ibus_ma          — Input current        (mA)
//   u16 vbat_mv          — Pack voltage         (mV)
//   i16 ibat_ma          — Battery current      (mA, +ve=charging, -ve=discharging)
//   u8[5] chg_stat       — BQ25798 REG1B..REG1F (STATUS_0..4)
//   u8[4] bq_stat        — BQ76920 SYS_STAT, CELLBAL1, SYS_CTRL1, SYS_CTRL2
#define LOG_STATUS  0x90

#endif
