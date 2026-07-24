import numpy as np, pandas as pd
from scipy import signal

df = pd.read_csv("data/sensor_log.csv")
t   = df["timestamp_s"].to_numpy()
DROP = 5412
tc, accel, enc = t[:DROP], df["accel_x_mss"].to_numpy()[:DROP], df["encoder_count"].to_numpy()[:DROP]
fs = 1.0/np.median(np.diff(tc))

print("=== ENCODER: true quantization step ===")
denc = np.abs(np.diff(enc)); denc = denc[denc != 0]
q_gcd = int(np.gcd.reduce(denc))
print(f"GCD of all count steps      : {q_gcd}   <-- the real quantization step")
print(f"GCD of raw counts           : {int(np.gcd.reduce(enc[enc!=0]))}")
print(f"min step (misleading)       : {denc.min()}")
print(f"quantization noise std      : q/sqrt(12) = {q_gcd/np.sqrt(12):.3f} counts")

print("\n=== ACCEL: noise, cross-checked ===")
print(f"signal std (for context)    : {accel.std():.4f} m/s^2")
# Estimator A: sensitivity of savgol residual to window length
for w in (11, 21, 51, 101):
    r = accel - signal.savgol_filter(accel, w, 3)
    print(f"  savgol win={w:3d} residual std: {r.std():.4f}")
# Estimator B: for white noise on a smooth signal, std(diff)/sqrt(2)
print(f"  diff-based estimate       : {np.diff(accel).std()/np.sqrt(2):.4f}")
# Estimator C: noise floor from the PSD's flat high-frequency region
t_uni = np.arange(tc[0], tc[-1], 1/fs)
au = np.interp(t_uni, tc, accel)
f, P = signal.welch(au, fs=fs, nperseg=1024)
hf = P[f > 0.8*f.max()]
print(f"  PSD floor -> std          : {np.sqrt(np.median(hf)*f.max()):.4f}")
print(f"\nTotal signal power below 5/10/20 Hz:")
for cut in (5, 10, 20, 40):
    frac = P[f<=cut].sum()/P.sum()
    print(f"  <= {cut:2d} Hz : {100*frac:.1f}% of power")
