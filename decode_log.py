#!/usr/bin/env python3
"""
Battery manager serial log decoder.

Reads binary log codes from the ATtiny1616 over UART and prints
human-readable output. Codes and payload formats must match log_codes.h.

Usage:
    python3 decode_log.py [port] [baud]

    port  - serial device, default /dev/ttyAMA0
    baud  - baud rate,     default 9600

Install dependency:
    pip3 install pyserial
"""

import struct
import sys
import time

import serial

# ── Code table ────────────────────────────────────────────────────────────────
# Each entry: code -> (label, struct_fmt, field_names)
#
# struct_fmt uses little-endian prefix '<' and standard type codes:
#   b = int8, B = uint8, h = int16, H = uint16
#
# field_names drive the pretty-printer; names ending in '_mv' are shown as mV,
# '_c_x10' is divided by 10 and shown as °C.

CODES = {
    # Main ── 0x01–0x1F
    0x01: ("Starting",                         None,    []),
    0x02: ("BQ25798 found",                    None,    []),
    0x03: ("BQ25798 not found",                None,    []),
    0x04: ("BQ76920 found",                    None,    []),
    0x05: ("BQ76920 not found",                None,    []),
    0x06: ("AHT20 found",                      None,    []),
    0x07: ("AHT20 not found",                  None,    []),
    0x08: ("AHT20 read failed at startup",     None,    []),
    0x09: ("Startup temps",                    "<hhh",  ["aht_c_x10", "bal_c_x10", "chg_c_x10"]),
    0x0A: ("Startup temp out of range",        None,    []),
    0x0B: ("Startup temps disagree > 10 °C",  None,    []),
    0x0C: ("Cell population check failed",     None,    []),
    0x0D: ("Entering sleep mode",              None,    []),
    0x0E: ("Input source detected, waking",    None,    []),
    0x0F: ("No input for 20 s, sleeping",      None,    []),
    0x10: ("Poor source, sleeping",            None,    []),
    0x11: ("Restarting",                       None,    []),
    0x12: ("Charger interrupt fired",          None,    []),
    0x13: ("Balancer interrupt fired",         None,    []),

    # Protection ── 0x20–0x2F
    0x20: ("UV cell recovered",                None,    []),
    0x21: ("Short circuit discharge",          None,    []),
    0x22: ("Overcurrent discharge",            None,    []),
    0x23: ("OCD/SCD fault cleared",            None,    []),
    0x24: ("Cell over voltage",                None,    []),
    0x25: ("Cell under voltage",               None,    []),
    0x26: ("Cell UV and OV simultaneously",    None,    []),
    0x27: ("BQ76920 temp",                     "<h",    ["temp_c_x10"]),
    0x28: ("VBAT over voltage protection",     None,    []),
    0x29: ("BQ25798 temperature fault",        None,    []),
    0x2A: ("AHT20 reading",                    "<hH",   ["temp_c_x10", "humidity_pct_x10"]),
    0x2B: ("Protection state change",          "<B",    ["state_flags"]),

    # AHT20 ── 0x50–0x58
    0x50: ("AHT20 status read failed",         None,    []),
    0x51: ("AHT20 init command failed",        None,    []),
    0x52: ("AHT20 status after init failed",   None,    []),
    0x53: ("AHT20 not calibrated after init",  None,    []),
    0x54: ("AHT20 trigger failed",             None,    []),
    0x55: ("AHT20 readResult without trigger", None,    []),
    0x56: ("AHT20 busy",                       None,    []),
    0x57: ("AHT20 CRC mismatch",               None,    []),
    0x58: ("AHT20 measurement timeout",        None,    []),

    # Util ── 0x60
    0x60: ("Buzzer frequency error",           None,    []),

    # I2C ── 0x80–0x81
    0x80: ("I2C failed to queue reg byte",     None,    []),
    0x81: ("I2C endTransmission error",        "<B",    ["err_code"]),

    # Periodic status ── 0x90 (handled specially in run())
    0x90: ("Status", "<IhhhBHHHHHHh5s4s",
           ["seconds", "temp_aht_x10", "temp_bq76920_x10", "temp_bq25798_x10",
            "humidity_pct", "cell1_mv", "cell2_mv", "cell3_mv",
            "vbus_mv", "ibus_ma", "vbat_mv", "ibat_ma",
            "chg_stat", "bq_stat"]),

    # BQ25798 ── 0x70–0x7F
    0x70: ("CHG bad part number",              "<B",    ["reg_data"]),
    0x71: ("CHG bad cell count",               None,    []),
    0x72: ("CHG bad PWM frequency",            None,    []),
    0x73: ("CHG init",                         None,    []),
    0x74: ("CHG HIZ disabled",                 None,    []),
    0x75: ("CHG MPPT enabled",                 None,    []),
    0x76: ("CHG VBUS wakeup disabled",         None,    []),
    0x77: ("CHG VBUS wakeup enabled",          None,    []),
    0x78: ("CHG charge status",                "<B",    ["chg_status"]),
    0x79: ("CHG VBUS status",                  "<B",    ["vbus_status"]),
    0x7A: ("CHG FAULT_Status_0",               "<B",    ["fault0"]),
    0x7B: ("CHG FAULT_Status_1",               "<B",    ["fault1"]),
    0x7C: ("CHG flags",                        "<6B",   ["r22", "r23", "r24", "r25", "r26", "r27"]),
    0x7D: ("CHG write error",                  None,    []),
    0x7E: ("CHG read error",                   None,    []),
    0x7F: ("CHG write mismatch",               "<BHH",  ["reg", "written", "read_back"]),

    # BQ76920 ── 0x30–0x4F
    0x30: ("BQ76920 temp out of range",       "<h",   ["temp_c_x10"]),
    0x31: ("BQ76920 CRC err (readReg)",        None,   []),
    0x32: ("BQ76920 CRC[0] err (readBlock)",   None,   []),
    0x33: ("BQ76920 CRC[n] err (readBlock)",   None,   []),
    0x34: ("BQ76920 cell missing",             None,   []),
    0x35: ("BQ76920 cell extra",               None,   []),
    0x36: ("BQ76920 cell voltage read failed", None,   []),
    0x37: ("BQ76920 balancing started",        None,   []),
    0x38: ("BQ76920 balancing stopped",        None,   []),
    0x39: ("BQ76920 balance diff",             "<H",   ["diff_mv"]),
    0x3A: ("BQ76920 balancing cell",           "<BHH", ["cell", "max_mv", "min_mv"]),
    0x3B: ("BQ76920 min cell voltage",         "<H",   ["min_mv"]),
    0x3C: ("BQ76920 OV trip out of range",     None,   []),
    0x3D: ("BQ76920 UV trip out of range",     None,   []),
    0x3E: ("BQ76920 OV trip set",              "<H",   ["trip_mv"]),
    0x3F: ("BQ76920 UV trip set",              "<H",   ["trip_mv"]),
    0x40: ("BQ76920 OV/UV write failed",       None,   []),
    0x41: ("BQ76920 OV/UV read failed",        None,   []),
    0x42: ("BQ76920 PROTECT1 write failed",    None,   []),
    0x43: ("BQ76920 PROTECT2 write failed",    None,   []),
    0x44: ("BQ76920 PROTECT3 write failed",    None,   []),
    0x45: ("BQ76920 test mode: max cell V",    None,   []),
    0x46: ("BQ76920 test mode: min cell V",    None,   []),
    0x47: ("BQ76920 cell mV",                  "<H",   ["mv"]),
}

LOG_BQ_CELL_MV = 0x47
CELL_COUNT = 5

LOG_CHG_FLAGS = 0x7C
LOG_STATUS = 0x90

# (reg_index, bit_mask): flag_name
_CHG_FLAG_BITS = {
    (0, 0x01): "VBUS_PRESENT", (0, 0x02): "VAC1_PRESENT", (0, 0x04): "VAC2_PRESENT",
    (0, 0x08): "POWER_GOOD",   (0, 0x10): "POORSRC",      (0, 0x20): "ADC_DONE",
    (1, 0x01): "ICO_DONE",     (1, 0x02): "ICO_FAIL",     (1, 0x04): "IINDPM",
    (1, 0x08): "VINDPM",       (1, 0x10): "TREG",
    (2, 0x01): "CHG_DONE",     (2, 0x02): "RECHARGE",     (2, 0x04): "PRECHARGE",
    (2, 0x08): "FASTCHARGE",   (2, 0x10): "TOPOFF",       (2, 0x20): "TERMINATION",
    (3, 0x08): "TS_COLD",      (3, 0x04): "TS_COOL",      (3, 0x02): "TS_WARM",
    (3, 0x01): "TS_HOT",
    (4, 0x01): "VBUS_OVP",     (4, 0x02): "IBUS_OCP",     (4, 0x04): "IBAT_OCP",
    (4, 0x08): "VSYS_OVP",     (4, 0x10): "VBAT_OVP",     (4, 0x20): "VBAT_UVP",
    (5, 0x01): "TS_FAULT",     (5, 0x02): "TSHUT",        (5, 0x04): "WATCHDOG",
    (5, 0x08): "SAFETY_TIMER",
}

CHG_CHARGE_STATUS = {
    0: "not charging", 1: "trickle", 2: "pre-charge", 3: "fast charge (CC)",
    4: "taper (CV)", 5: "reserved", 6: "top-off", 7: "done",
}

CHG_VBUS_STATUS = {
    0x0: "no input", 0x1: "USB SDP", 0x2: "USB CDP", 0x3: "USB DCP",
    0x4: "HVDCP", 0x5: "unknown adapter", 0x6: "non-standard adapter",
    0x7: "OTG", 0x8: "bad adapter", 0xB: "direct VBUS", 0xC: "backup",
}


def fmt_payload(fields, values):
    parts = []
    for name, val in zip(fields, values):
        if name.endswith("_c_x10"):
            label = name.replace("_c_x10", "")
            parts.append(f"{label}={val / 10:.1f}°C")
        elif name.endswith("_pct_x10"):
            label = name.replace("_pct_x10", "")
            parts.append(f"{label}={val / 10:.1f}%")
        elif name == "state_flags":
            parts.append(f"healthy={'Y' if val & 0x01 else 'N'}  "
                         f"chg={'Y' if val & 0x02 else 'N'}  "
                         f"discharge={'Y' if val & 0x04 else 'N'}  "
                         f"bal={'Y' if val & 0x08 else 'N'}")
        elif name == "seconds":
            parts.append(f"t={val}s")
        elif name.endswith("_mv"):
            parts.append(f"{name}={val}mV")
        elif name.endswith("_ma"):
            label = name.replace("_ma", "")
            parts.append(f"{label}={val}mA")
        elif name == "chg_status":
            parts.append(f"chg={CHG_CHARGE_STATUS.get(val, f'0x{val:02X}')}")
        elif name == "vbus_status":
            parts.append(f"vbus={CHG_VBUS_STATUS.get(val, f'0x{val:02X}')}")
        elif name in ("fault0", "fault1", "reg_data", "reg", "err_code"):
            parts.append(f"{name}=0x{val:02X}")
        elif name in ("written", "read_back"):
            parts.append(f"{name}=0x{val:04X}")
        elif name.startswith("r2") and len(name) == 3:
            pass  # CHG flags raw bytes — decoded separately in run()
        else:
            parts.append(f"{name}={val}")
    return ("  " + "  ".join(parts)) if parts else ""


def run(port, baud):
    print(f"Opening {port} at {baud} baud …")
    with serial.Serial(port, baud, timeout=2) as ser:
        print("Listening. Press Ctrl-C to stop.\n")
        cell_idx = 0

        while True:
            raw = ser.read(1)
            if not raw:
                continue

            code = raw[0]
            ts = time.strftime("%H:%M:%S")

            if code not in CODES:
                print(f"[{ts}] 0x{code:02X}  (unknown)")
                cell_idx = 0
                continue

            label, fmt, fields = CODES[code]

            values = ()
            if fmt is not None:
                size = struct.calcsize(fmt)
                payload = ser.read(size)
                if len(payload) < size:
                    print(f"[{ts}] 0x{code:02X}  short read (got {len(payload)}/{size} bytes)")
                    cell_idx = 0
                    continue
                values = struct.unpack(fmt, payload)

            # Special case: CHG flags — decode each set bit by name
            if code == LOG_CHG_FLAGS:
                cell_idx = 0
                set_flags = [name for (ri, mask), name in _CHG_FLAG_BITS.items()
                             if values[ri] & mask]
                flag_str = ("  " + " ".join(set_flags)) if set_flags else "  (none)"
                print(f"[{ts}] CHG flags{flag_str}")
                continue

            # Special case: status snapshot — print on multiple lines
            if code == LOG_STATUS:
                cell_idx = 0
                (secs, t_aht, t_bal, t_chg, hum,
                 c1, c2, c3, vbus, ibus, vbat, ibat,
                 chg_stat, bq_stat) = values
                chg_a = ibat / 1000.0 if ibat > 0 else 0.0
                dsg_a = abs(ibat) / 1000.0 if ibat < 0 else 0.0

                # BQ25798 STATUS_0..4 (REG1B..1F)
                s0, s1, s2, s3, s4 = chg_stat
                chg_status_str = CHG_CHARGE_STATUS.get((s1 >> 5) & 0x07, f"0x{(s1>>5)&0x07:X}")
                vbus_type_str  = CHG_VBUS_STATUS.get((s1 >> 1) & 0x0F, f"0x{(s1>>1)&0x0F:X}")

                # BQ76920 SYS_STAT, CELLBAL1, SYS_CTRL1, SYS_CTRL2
                sys_stat, cellbal, ctrl1, ctrl2 = bq_stat
                bq_flags = []
                if sys_stat & 0x08: bq_flags.append("UV")
                if sys_stat & 0x04: bq_flags.append("OV")
                if sys_stat & 0x02: bq_flags.append("SCD")
                if sys_stat & 0x01: bq_flags.append("OCD")
                bal_cells = [i for i in range(5) if cellbal & (1 << i)]

                print(f"[{ts}] STATUS  t={secs}s  "
                      f"temp: aht={t_aht/10:.1f}°C bal={t_bal/10:.1f}°C chg={t_chg/10:.1f}°C  "
                      f"hum={hum}%")
                print(f"         cells: {c1}mV {c2}mV {c3}mV  "
                      f"vbus={vbus}mV({vbus_type_str}) ibus={ibus}mA  "
                      f"vbat={vbat}mV chg={chg_a:.2f}A dsg={dsg_a:.2f}A  "
                      f"chg_stat={chg_status_str}")
                print(f"         bq25798: "
                      f"s0=0x{s0:02X} s1=0x{s1:02X} s2=0x{s2:02X} s3=0x{s3:02X} s4=0x{s4:02X}  "
                      f"bq76920: stat=0x{sys_stat:02X}({' '.join(bq_flags) or 'OK'}) "
                      f"bal={bal_cells} ctrl2=0x{ctrl2:02X}(chg={'Y' if ctrl2&0x01 else 'N'} "
                      f"dsg={'Y' if ctrl2&0x02 else 'N'})")
                continue

            # Special case: cell mV messages arrive in a run of CELL_COUNT
            if code == LOG_BQ_CELL_MV:
                print(f"[{ts}]   cell[{cell_idx}] = {values[0]} mV")
                cell_idx = (cell_idx + 1) % CELL_COUNT
            else:
                cell_idx = 0
                print(f"[{ts}] {label}{fmt_payload(fields, values)}")


if __name__ == "__main__":
    _port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyUSB0"
    _baud = int(sys.argv[2]) if len(sys.argv) > 2 else 9600
    try:
        run(_port, _baud)
    except KeyboardInterrupt:
        print("\nStopped.")
    except serial.SerialException as e:
        print(f"Serial error: {e}", file=sys.stderr)
        sys.exit(1)
