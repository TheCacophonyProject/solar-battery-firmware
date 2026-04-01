# Battery Plots

Graphs battery log data captured by `decode_log.py`.

## Setup

### 1. Create a virtual environment

```bash
python3 -m venv .venv
```

### 2. Activate the virtual environment

```bash
source .venv/bin/activate
```

### 3. Install dependencies

```bash
pip install -r requirements.txt
```

## Usage

Place CSV files (produced by `decode_log.py`) in the `data/` folder, then run:

```bash
python3 graph_battery.py data/battery_20240101_120000.csv
```

### What is plotted

| Axis | Series |
|------|--------|
| Left — Voltage (V) | Pack voltage, Cell 1, Cell 2, Cell 3 |
| Right — Power (W) | Input power (VBUS × IBUS), Output power (VBAT × CC discharge current) |

## Deactivating the virtual environment

```bash
deactivate
```
