import numpy as np
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("data/sensor_log.csv")
print("Rows:", len(df))
print(df.describe())

t = df["timestamp_s"].to_numpy()
dt = np.diff(t)

print(f"\n--- Timing ---")
print(f"median dt : {np.median(dt)*1000:.3f} ms  ->  {1/np.median(dt):.1f} Hz")
print(f"min dt    : {dt.min()*1000:.3f} ms")
print(f"max dt    : {dt.max()*1000:.3f} ms")
print(f"dt std    : {dt.std()*1000:.3f} ms")

# Where is the dropout? (a dt much bigger than the rest)
gap_idx = np.argmax(dt)
print(f"\nLargest gap: {dt[gap_idx]*1000:.1f} ms at t={t[gap_idx]:.3f}s (row {gap_idx})")

# Plot dt over time so to SEE the jitter and the dropout
fig, ax = plt.subplots(2, 1, figsize=(11, 7))
ax[0].plot(t[1:], dt*1000, ".", markersize=2)
ax[0].axhline(5.0, color="r", ls="--", label="nominal 5 ms (200 Hz)")
ax[0].set_ylabel("dt (ms)"); ax[0].set_xlabel("time (s)")
ax[0].set_title("Inter-sample time — jitter around 5ms + the dropout spike")
ax[0].legend()

ax[1].hist(dt*1000, bins=80)
ax[1].set_xlabel("dt (ms)"); ax[1].set_ylabel("count")
ax[1].set_title("Distribution of dt — how much does timing wander?")

plt.tight_layout()
plt.savefig("analysis/figures/timing.png", dpi=110)
print("\nSaved analysis/figures/timing.png")
