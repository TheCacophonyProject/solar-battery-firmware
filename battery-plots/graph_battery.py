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

    df["wall_time"] = pd.to_datetime(df["wall_time"], format="%Y-%m-%d %H:%M:%S", errors="coerce")

    numeric_cols = ["cell1_mv", "cell2_mv", "cell3_mv",
                    "vbat_mv", "vbus_mv", "ibus_ma", "ibat_ma", "ibat_cc_ma",
                    "temp_aht_c", "temp_bq76920_c", "temp_bq25798_c"]
    df[numeric_cols] = df[numeric_cols].apply(pd.to_numeric, errors="coerce")
    df = df.dropna(subset=["wall_time"] + numeric_cols)

    # Parse cellbal register: bit 0 = cell1, bit 1 = cell2, bit 4 = cell3 (VC5)
    cellbal = df["cellbal_hex"].apply(lambda x: int(str(x), 16) if pd.notna(x) else 0).astype(int)
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

    fig, (ax_pack, ax_cell, ax_t) = plt.subplots(3, 1, figsize=(14, 11), sharex=True,
                                                  gridspec_kw={"height_ratios": [2, 2, 1]})
    ax_p = ax_pack.twinx()

    # ── Pack voltage subplot ─────────────────────────────────────────────────
    ax_pack.axhline(9.0,  color="tab:red",   linewidth=1.0, linestyle=":", label="Cutoff 9.0V")
    ax_pack.axhline(12.3, color="tab:olive", linewidth=1.0, linestyle=":", label="Max charge 12.3V")
    ax_pack.plot(t, pack_v, label="Pack voltage (V)", color="black", linewidth=1.5)

    pack_lo = min(8.5,  pack_v.min() - 0.1)
    pack_hi = max(12.8, pack_v.max() + 0.1)
    ax_pack.set_ylim(pack_lo, pack_hi)
    ax_pack.set_ylabel("Pack voltage (V)")

    # ── Power traces (right axis of pack subplot) ────────────────────────────
    ax_p.plot(t, input_w,  label="Input power (W)",  color="tab:red",    linewidth=1.0, linestyle="--")
    ax_p.plot(t, output_w, label="Output power (W)", color="tab:purple", linewidth=1.0, linestyle="--")
    ax_p.set_ylim(bottom=0)
    ax_p.set_ylabel("Power (W)")

    lines_pack, labels_pack = ax_pack.get_legend_handles_labels()
    lines_p,    labels_p    = ax_p.get_legend_handles_labels()
    ax_pack.legend(lines_pack + lines_p, labels_pack + labels_p,
                   loc="upper center", bbox_to_anchor=(0.5, -0.05), ncol=4)

    # ── Cell voltage subplot ─────────────────────────────────────────────────
    ax_cell.axhline(3.0, color="tab:red",   linewidth=1.0, linestyle=":", label="Min cell 3.0V")
    ax_cell.axhline(4.1, color="tab:blue",  linewidth=1.0, linestyle=":", label="4.1V")
    ax_cell.axhline(4.2, color="tab:olive", linewidth=1.0, linestyle=":", label="Max cell 4.2V")
    _plot_cell(ax_cell, t, cell1_v, bal1, "Cell 1 (V)", "tab:blue")
    _plot_cell(ax_cell, t, cell2_v, bal2, "Cell 2 (V)", "tab:orange")
    _plot_cell(ax_cell, t, cell3_v, bal3, "Cell 3 (V)", "tab:green")

    all_cells = pd.concat([cell1_v, cell2_v, cell3_v])
    cell_lo = min(2.5, all_cells.min() - 0.05)
    cell_hi = max(4.3, all_cells.max() + 0.05)
    ax_cell.set_ylim(cell_lo, cell_hi)
    ax_cell.set_ylabel("Cell voltage (V)")
    ax_cell.legend(loc="upper center", bbox_to_anchor=(0.5, -0.05), ncol=4)

    # ── Temperature subplot ───────────────────────────────────────────────────
    ax_t.axhline(0.0,  color="tab:blue", linewidth=1.0, linestyle=":", label="Min temp 0°C")
    ax_t.axhline(60.0, color="tab:red",  linewidth=1.0, linestyle=":", label="Max temp 60°C")
    ax_t.plot(t, df["temp_aht_c"],      label="AHT20 (°C)",   color="tab:blue",   linewidth=1.0)
    ax_t.plot(t, df["temp_bq76920_c"],  label="BQ76920 (°C)", color="tab:orange", linewidth=1.0)
    ax_t.plot(t, df["temp_bq25798_c"],  label="BQ25798 (°C)", color="tab:green",  linewidth=1.0)

    ax_t.set_ylabel("Temperature (°C)")
    ax_t.set_xlabel("Time")
    ax_t.xaxis.set_major_formatter(mdates.ConciseDateFormatter(ax_t.xaxis.get_major_locator()))
    ax_t.legend(loc="upper center", bbox_to_anchor=(0.5, -0.3), ncol=3)
    fig.autofmt_xdate()

    fig.suptitle(f"Battery log — {csv_path}")
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <csv_file>")
        sys.exit(1)
    main(sys.argv[1])
