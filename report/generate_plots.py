"""
generate_plots.py — produce before/after filter plots for the report.

Reads data/sensor_log.csv, applies siglib filters offline (via pybind11),
and generates figures showing:

  Fig 1: Accel raw vs low-pass filtered (full 60s)
  Fig 2: Accel raw vs MA filtered (full 60s)
  Fig 3: Zoom around the 157ms dropout at t=27.416s
          showing how each filter behaves across the gap
  Fig 4: Encoder raw vs low-pass filtered (full 60s)

All filter parameters derived from FINDINGS.md — no round numbers:
  fc       = 20 Hz   (retains 97.4% of signal power)
  MA T     = 25 ms   (time-window mode)
  dropout  = 157.272 ms at t=27.416s, row 5412
"""

import sys
import csv
import math
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec

# Add siglib_py build directory to path if needed
# When running from the repo root after building with -DBUILD_BINDINGS=ON:
#   python report/generate_plots.py
SIGLIB_BUILD = Path(__file__).parent.parent / 'siglib' / 'build_py' / 'bindings'
if SIGLIB_BUILD.exists():
    sys.path.insert(0, str(SIGLIB_BUILD))

import siglib_py

# ── Load sensor_log.csv ───────────────────────────────────────────────────────
DATA_PATH = Path(__file__).parent.parent / 'data' / 'sensor_log.csv'

print(f'Loading {DATA_PATH}...')
timestamps = []
encoder    = []
accel      = []

with open(DATA_PATH, newline='') as f:
    reader = csv.DictReader(f)
    for row in reader:
        timestamps.append(float(row['timestamp_s']))
        encoder.append(int(row['encoder_count']))
        accel.append(float(row['accel_x_mss']))

timestamps = np.array(timestamps)
encoder    = np.array(encoder,    dtype=np.int32)
accel      = np.array(accel,      dtype=np.float32)

# Convert timestamps to nanoseconds (int64) for siglib
ts_ns = (timestamps * 1e9).astype(np.int64)

print(f'Loaded {len(timestamps)} samples, '
      f't=[{timestamps[0]:.3f}, {timestamps[-1]:.3f}]s')

# ── Apply filters ─────────────────────────────────────────────────────────────
print('Applying filters...')

# Low-pass: fc=20Hz (from FINDINGS.md)
lp_accel   = siglib_py.LowPassF32(20.0)
lp_encoder = siglib_py.LowPassI32(20.0)

# Moving average: time-window 25ms (from FINDINGS.md T=25ms)
ma_accel   = siglib_py.MovingAverageF32(capacity=10, window_ns=25_000_000)
ma_encoder = siglib_py.MovingAverageI32(capacity=5,  window_ns=25_000_000)

accel_lp   = lp_accel.process(ts_ns, accel)
accel_ma   = ma_accel.process(ts_ns, accel)
encoder_lp = lp_encoder.process(ts_ns, encoder)
encoder_ma = ma_encoder.process(ts_ns, encoder)

print('Filters applied.')

# ── Dropout region ────────────────────────────────────────────────────────────
# From FINDINGS.md: dropout at t=27.416s, row 5412, duration=157.272ms
DROPOUT_T   = 27.416
DROPOUT_DUR = 0.157272
ZOOM_START  = DROPOUT_T - 0.5   # 0.5s before
ZOOM_END    = DROPOUT_T + 1.0   # 1.0s after

zoom_mask = (timestamps >= ZOOM_START) & (timestamps <= ZOOM_END)
t_zoom    = timestamps[zoom_mask]

# ── Plot styling ──────────────────────────────────────────────────────────────
plt.rcParams.update({
    'figure.facecolor': 'white',
    'axes.grid': True,
    'grid.alpha': 0.3,
    'lines.linewidth': 1.0,
    'font.size': 10,
})

OUT_DIR = Path(__file__).parent / 'figures'
OUT_DIR.mkdir(exist_ok=True)

# ── Fig 1: Accel full trace — raw vs low-pass ─────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(timestamps, accel,    color='#aaaaaa', linewidth=0.5, label='raw', alpha=0.7)
ax.plot(timestamps, accel_lp, color='#e74c3c', linewidth=1.2, label='low-pass (fc=20Hz)')
ax.axvline(DROPOUT_T, color='orange', linestyle='--', linewidth=1.0, label=f'dropout t={DROPOUT_T}s')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Acceleration (m/s²)')
ax.set_title('Accel: raw vs low-pass filter (fc=20 Hz, variable-dt IIR)')
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / 'accel_lowpass_full.png', dpi=150)
plt.close(fig)
print('Saved accel_lowpass_full.png')

# ── Fig 2: Accel full trace — raw vs MA ───────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(timestamps, accel,    color='#aaaaaa', linewidth=0.5, label='raw', alpha=0.7)
ax.plot(timestamps, accel_ma, color='#2980b9', linewidth=1.2, label='moving avg (T=25ms)')
ax.axvline(DROPOUT_T, color='orange', linestyle='--', linewidth=1.0, label=f'dropout t={DROPOUT_T}s')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Acceleration (m/s²)')
ax.set_title('Accel: raw vs moving average (time-window T=25 ms)')
ax.legend(loc='upper right')
fig.tight_layout()
fig.savefig(OUT_DIR / 'accel_mavg_full.png', dpi=150)
plt.close(fig)
print('Saved accel_mavg_full.png')

# ── Fig 3: Dropout zoom — all filters ────────────────────────────────────────
fig, axes = plt.subplots(2, 1, figsize=(10, 7), sharex=True)

# Accel zoom
axes[0].plot(t_zoom, accel[zoom_mask],    color='#aaaaaa', linewidth=0.8,
             label='raw', alpha=0.8)
axes[0].plot(t_zoom, accel_lp[zoom_mask], color='#e74c3c', linewidth=1.5,
             label='low-pass (fc=20Hz)')
axes[0].plot(t_zoom, accel_ma[zoom_mask], color='#2980b9', linewidth=1.5,
             label='moving avg (T=25ms)')
axes[0].axvspan(DROPOUT_T, DROPOUT_T + DROPOUT_DUR,
                color='orange', alpha=0.2, label=f'dropout {DROPOUT_DUR*1000:.0f}ms')
axes[0].set_ylabel('Acceleration (m/s²)')
axes[0].set_title(f'Filter behaviour across 157ms dropout at t={DROPOUT_T}s')
axes[0].legend(loc='upper right', fontsize=8)

# Encoder zoom
axes[1].plot(t_zoom, encoder[zoom_mask],    color='#aaaaaa', linewidth=0.8,
             label='raw', alpha=0.8)
axes[1].plot(t_zoom, encoder_lp[zoom_mask], color='#e74c3c', linewidth=1.5,
             label='low-pass (fc=20Hz)')
axes[1].plot(t_zoom, encoder_ma[zoom_mask], color='#2980b9', linewidth=1.5,
             label='moving avg (T=25ms)')
axes[1].axvspan(DROPOUT_T, DROPOUT_T + DROPOUT_DUR,
                color='orange', alpha=0.2, label=f'dropout {DROPOUT_DUR*1000:.0f}ms')
axes[1].set_xlabel('Time (s)')
axes[1].set_ylabel('Encoder (counts)')
axes[1].legend(loc='upper right', fontsize=8)

fig.tight_layout()
fig.savefig(OUT_DIR / 'dropout_zoom.png', dpi=150)
plt.close(fig)
print('Saved dropout_zoom.png')

# ── Fig 4: Encoder full trace ─────────────────────────────────────────────────
fig, ax = plt.subplots(figsize=(12, 4))
ax.plot(timestamps, encoder,    color='#aaaaaa', linewidth=0.5, label='raw', alpha=0.7)
ax.plot(timestamps, encoder_lp, color='#e74c3c', linewidth=1.2, label='low-pass (fc=20Hz)')
ax.plot(timestamps, encoder_ma, color='#2980b9', linewidth=1.2, label='moving avg (T=25ms)')
ax.axvline(DROPOUT_T, color='orange', linestyle='--', linewidth=1.0,
           label=f'dropout t={DROPOUT_T}s')
ax.set_xlabel('Time (s)')
ax.set_ylabel('Encoder (counts)')
ax.set_title('Encoder: raw vs filtered (full 60s)')
ax.legend(loc='upper left')
fig.tight_layout()
fig.savefig(OUT_DIR / 'encoder_full.png', dpi=150)
plt.close(fig)
print('Saved encoder_full.png')

print(f'\nAll figures saved to {OUT_DIR}/')
print('Include dropout_zoom.png in report — shows filter reconvergence after gap.')
