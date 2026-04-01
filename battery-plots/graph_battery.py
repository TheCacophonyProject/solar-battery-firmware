#!/usr/bin/env python3
"""
Battery data grapher.

Reads a CSV produced by decode_log.py and plots:
  Left  Y axis : Pack voltage + cell voltages (V)
  Right Y axis : Input power and output power (W)
  X axis       : Time (seconds from firmware boot)

Usage:
    python3 graph_battery.py <csv_file>

Install dependencies:
    pip3 install matplotlib pandas
"""

import sys

import matplotlib.pyplot as plt
import matplotlib.dates as mdates
import pandas as pd


# Plot a cell voltage line, switching to dashed where balancing is active.
# balancing is a boolean Series aligned with t and v.
def _plot_cell(ax, t, v, balancing, label, color):
    t_arr = t.to_numpy()
    v_arr = v.to_numpy()
    b_arr = balancing.to_numpy()

    # Draw the full line solid at low zorder as a base
    (base,) = ax.plot(t_arr, v_arr, label=label, color=color, linewidth=1.0, zorder=2)

    # Overlay dashed segments where balancing is active
    added_legend = False
    i = 0
    while i < len(b_arr):
        if b_arr[i]:
            j = i
            while j < len(b_arr) and b_arr[j]:
                j += 1
            kwargs = dict(color=color, linewidth=2.5, linestyle="--", zorder=3)
            if not added_legend:
                ax.plot(t_arr[i:j], v_arr[i:j], label=f"{label} balancing", **kwargs)
                added_legend = True
            else:
                ax.plot(t_arr[i:j], v_arr[i:j], **kwargs)
            i = j
        else:
            i += 1

    return base


def main(csv_path):
    df = pd.read_csv(csv_path)

    # Parse wall_time as today's date + HH:MM:SS
    df["wall_time"] = pd.to_datetime(df["wall_time"], format="%H:%M:%S", errors="coerce")

    numeric_cols = ["cell1_mv", "cell2_mv", "cell3_mv",
                    "vbat_mv", "vbus_mv", "ibus_ma", "ibat_ma", "ibat_cc_ma",
                    "temp_aht_c", "temp_bq76920_c", "temp_bq25798_c"]
    df[numeric_cols] = df[numeric_cols].apply(pd.to_numeric, errors="coerce")
    df = df.dropna(subset=["wall_time"] + numeric_cols)

    # Parse cellbal register: bit 0 = cell1, bit 1 = cell2, bit 4 = cell3 (VC5)
    cellbal = df["cellbal_hex"].apply(lambda x: int(x, 16) if isinstance(x, str) else 0)
    bal1 = (cellbal & 0x01) != 0
    bal2 = (cellbal & 0x02) != 0
    bal3 = (cellbal & 0x10) != 0

    t = df["wall_time"]

    # Voltages in V
    pack_v  = df["vbat_mv"]  / 1000.0
    cell1_v = df["cell1_mv"] / 1000.0
    cell2_v = df["cell2_mv"] / 1000.0
    cell3_v = df["cell3_mv"] / 1000.0

    # Input power: VBUS voltage × IBUS current (W)
    input_w = df["vbus_mv"] / 1000.0 * df["ibus_ma"] / 1000.0

    # Output power: pack voltage × discharge current from BQ76920 CC (W)
    # ibat_cc_ma is negative when discharging; clamp to >= 0
    dsg_ma   = df["ibat_cc_ma"].clip(upper=0).abs()
    output_w = df["vbat_mv"] / 1000.0 * dsg_ma / 1000.0

    fig, (ax_v, ax_t) = plt.subplots(2, 1, figsize=(14, 9), sharex=True,
                                      gridspec_kw={"height_ratios": [2, 1]})
    ax_p = ax_v.twinx()

    # ── Voltage reference lines ──────────────────────────────────────────────
    ax_v.axhline(9.0,  color="tab:red",   linewidth=1.0, linestyle=":", label="Cutoff 9.0V")
    ax_v.axhline(12.3, color="tab:olive", linewidth=1.0, linestyle=":", label="Max charge 12.3V")

    # ── Voltage traces (left axis) ──────────────────────────────────────────
    ax_v.plot(t, pack_v, label="Pack voltage (V)", color="black", linewidth=1.5)
    _plot_cell(ax_v, t, cell1_v, bal1, "Cell 1 (V)", "tab:blue")
    _plot_cell(ax_v, t, cell2_v, bal2, "Cell 2 (V)", "tab:orange")
    _plot_cell(ax_v, t, cell3_v, bal3, "Cell 3 (V)", "tab:green")

    ax_v.set_ylim(bottom=0)
    ax_v.set_ylabel("Voltage (V)")

    # ── Power traces (right axis) ────────────────────────────────────────────
    ax_p.plot(t, input_w,  label="Input power (W)",  color="tab:red",    linewidth=1.0, linestyle="--")
    ax_p.plot(t, output_w, label="Output power (W)", color="tab:purple", linewidth=1.0, linestyle="--")

    ax_p.set_ylim(bottom=0)
    ax_p.set_ylabel("Power (W)")

    # ── Temperature subplot ───────────────────────────────────────────────────
    ax_t.axhline(0.0,  color="tab:blue", linewidth=1.0, linestyle=":", label="Min temp 0°C")
    ax_t.axhline(60.0, color="tab:red",  linewidth=1.0, linestyle=":", label="Max temp 60°C")
    ax_t.plot(t, df["temp_aht_c"],      label="AHT20 (°C)",   color="tab:blue",   linewidth=1.0)
    ax_t.plot(t, df["temp_bq76920_c"],  label="BQ76920 (°C)", color="tab:orange", linewidth=1.0)
    ax_t.plot(t, df["temp_bq25798_c"],  label="BQ25798 (°C)", color="tab:green",  linewidth=1.0)

    ax_t.set_ylabel("Temperature (°C)")
    ax_t.set_xlabel("Time")
    ax_t.xaxis.set_major_formatter(mdates.DateFormatter("%H:%M:%S"))
    ax_t.legend(loc="upper center", bbox_to_anchor=(0.5, -0.3), ncol=3)
    fig.autofmt_xdate()

    # ── Legend for voltage/power plot ─────────────────────────────────────────
    lines_v, labels_v = ax_v.get_legend_handles_labels()
    lines_p, labels_p = ax_p.get_legend_handles_labels()
    ax_v.legend(lines_v + lines_p, labels_v + labels_p,
                loc="upper center", bbox_to_anchor=(0.5, -0.05), ncol=4)

    fig.suptitle(f"Battery log — {csv_path}")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <csv_file>")
        sys.exit(1)
    main(sys.argv[1])
