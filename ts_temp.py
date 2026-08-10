#!/usr/bin/env python3
"""
Convert a BQ25798 TS pin voltage-divider percentage into a temperature reading.

Mirrors the calculation in src/bq25798.cpp (BQ25798::readTemp /
BQ25798::readADCAll) and the NTC lookup table in src/ntcTemp.cpp, so the
result here should match what the firmware computes for the same reading.

Usage:
    ./ts_temp.py 65.5          # single percentage (e.g. TS_COOL default = 68.4)
    ./ts_temp.py 68.4 44.8     # multiple values at once
    ./ts_temp.py               # interactive: enter one value per line
"""

import math
import sys

# From src/bq25798.h
# R1_OHMS = 5000.0
# R1_OHMS = 5230.0
R2_OHMS = 30000.0

# From src/ntcTemp.cpp (NCP□XH103 10k, B = 3380K)
NTC_TABLE = [
    (-40.0, 195.652), (-35.0, 148.171), (-30.0, 113.347), (-25.0, 87.559),
    (-20.0, 68.237),  (-15.0, 53.650),  (-10.0, 42.506),  (-5.0, 33.892),
    (0.0, 27.219),    (5.0, 22.021),    (10.0, 17.926),   (15.0, 14.674),
    (20.0, 12.081),   (25.0, 10.000),   (30.0, 8.315),    (35.0, 6.948),
    (40.0, 5.834),    (45.0, 4.917),    (50.0, 4.161),    (55.0, 3.535),
    (60.0, 3.014),    (65.0, 2.586),    (70.0, 2.228),    (75.0, 1.925),
    (80.0, 1.669),    (85.0, 1.452),    (90.0, 1.268),    (95.0, 1.110),
    (100.0, 0.974),   (105.0, 0.858),   (110.0, 0.758),   (115.0, 0.672),
    (120.0, 0.596),   (125.0, 0.531),
]


def rt_from_percentage(percentage: float) -> float:
    """percentage is a fraction (0.0-1.0) of REGN, matching `percentage` in readTemp()."""
    r1, r2 = R1_OHMS, R2_OHMS
    return (percentage * r1 * r2) / (r2 - percentage * (r1 + r2))


def temp_from_resistance(r_ohms: float) -> float:
    if r_ohms <= 0:
        return math.nan

    r_kohm = r_ohms / 1000.0

    if r_kohm >= NTC_TABLE[0][1]:
        return NTC_TABLE[0][0]  # <= -40C
    if r_kohm <= NTC_TABLE[-1][1]:
        return NTC_TABLE[-1][0]  # >= 125C

    for (t1, r1), (t2, r2) in zip(NTC_TABLE, NTC_TABLE[1:]):
        if r_kohm <= r1 and r_kohm >= r2:
            log_r, log_r1, log_r2 = math.log(r_kohm), math.log(r1), math.log(r2)
            t = (log_r - log_r1) / (log_r2 - log_r1)
            return t1 + t * (t2 - t1)

    return math.nan  # unreachable if table is monotonic


def temp_from_percentage(percentage_pct: float) -> float:
    rt = rt_from_percentage(percentage_pct / 100.0)
    return temp_from_resistance(rt)


def main(argv):
    values = argv[1:]
    if values:
        for v in values:
            pct = float(v)
            print(f"{pct:6.2f}%  ->  {temp_from_percentage(pct):6.2f} C")
        return

    print("Enter TS percentage (e.g. 68.4), Ctrl-D to exit:")
    while True:
        try:
            line = input("> ").strip()
        except EOFError:
            break
        if not line:
            continue
        try:
            pct = float(line)
        except ValueError:
            print("not a number")
            continue
        print(f"{temp_from_percentage(pct):.2f} C")


if __name__ == "__main__":
    main(sys.argv)
