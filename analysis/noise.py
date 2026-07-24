import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from scipy import signal

df = pd.read_csv("data/sensor_log.csv")
t = df["timestamp_s"].to_numpy()

DROP_ROW = 5412                      # sample just before the 157 ms dropout
t_clean = t[:DROP_ROW]
accel   = df["accel_x_mss"].to_numpy()[:DROP_ROW]
enc     = df["encoder_count"].to_numpy()[:DROP_ROW]

fs = 1.0 / np.median(np.diff(t_clean))
print(f"Analysis fs ~ {fs:.1f} Hz on {len(t_clean)} clean samples\n")

# ---------- FLOAT CHANNEL: accelerometer ----------
t_uni = np.arange(t_clean[0], t_clean[-1], 1/fs)
accel_uni = np.interp(t_uni, t_clean, accel)

f, Pxx = signal.welch(accel_uni, fs=fs, nperseg=1024)

fig, ax = plt.subplots(2, 2, figsize=(13, 8))
ax[0,0].semilogy(f, Pxx)
ax[0,0].set_title("Accel PSD (Welch) - knee = signal left, noise floor right")
ax[0,0].set_xlabel("Hz"); ax[0,0].set_ylabel("PSD")
ax[0,0].grid(True, which="both", alpha=0.3)

resid = accel_uni - signal.savgol_filter(accel_uni, 51, 3)
print(f"Accel residual std (noise estimate): {resid.std():.5f} m/s^2")
ac = np.correlate(resid, resid, mode="full")
ac = ac[ac.size//2:]; ac = ac/ac[0]
ax[0,1].plot(ac[:60])
ax[0,1].set_title("Accel residual autocorrelation (delta-like = white)")
ax[0,1].set_xlabel("lag (samples)"); ax[0,1].grid(True, alpha=0.3)

# ---------- INT CHANNEL: encoder ----------
denc = np.diff(enc)
nonzero = np.abs(denc[denc != 0])
q = nonzero.min() if len(nonzero) else 0
print(f"\nEncoder smallest count step q = {q}")
print(f"Ideal quantization noise std = q/sqrt(12) = {q/np.sqrt(12):.3f} counts")

ax[1,0].plot(t_clean, enc)
ax[1,0].set_title("Encoder count - raw integer stream")
ax[1,0].set_xlabel("time (s)"); ax[1,0].set_ylabel("counts")
ax[1,0].grid(True, alpha=0.3)

ax[1,1].hist(denc, bins=40)
ax[1,1].set_title("Encoder step sizes (quantization visible)")
ax[1,1].set_xlabel("delta counts"); ax[1,1].grid(True, alpha=0.3)

plt.tight_layout()
plt.savefig("analysis/figures/noise.png", dpi=110)
print("\nSaved analysis/figures/noise.png")
