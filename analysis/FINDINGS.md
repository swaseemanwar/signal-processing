# sensor_log.csv characterization

## Timing (analysis/explore.py)
- median dt = 5.001 ms -> 200.0 Hz nominal
- jitter std = 1.559 ms = 31% of nominal period
- min dt = 3.644 ms (samples arrive early too - jitter is two-sided)
- dropout = 157.272 ms at t = 27.416 s (row 5412), ~31x normal spacing
=> fixed-dt filters are wrong on nearly every sample. Per-sample dt required.

## Encoder (int stream) - analysis/noise2.py
- GCD of all counts and all count steps = 1 -> LSB is 1 count
- quantization noise std = 1/sqrt(12) = 0.289 counts
- method: GCD, not min-step (min-step gave 66, which measures velocity not
  quantization - rejected)

## Accel (float stream)
- signal std = 1.0339 m/s^2
- noise std ~= 0.05-0.06 m/s^2, from two independent methods:
    (a) PSD floor above 60 Hz -> 0.052
    (b) power-fraction: 0.2% of 1.069 over 60% of band -> 0.060
- savgol residual method REJECTED: estimate grew with window (0.20->0.95),
  i.e. it was counting real motion as noise
- SNR ~= 25 dB

## Spectral content (Welch, interpolated to uniform grid)
| band     | cumulative power |
|----------|------------------|
| <= 5 Hz  | 44.5%            |
| <= 10 Hz | 77.9%            |
| <= 20 Hz | 97.4%            |
| <= 40 Hz | 99.8%            |
Caveat: Welch assumes uniform sampling; data was linearly interpolated onto a
uniform grid first. Lomb-Scargle (non-uniform-native) is the rigorous check.

## Derived filter parameters
- low-pass cutoff f_c = 20 Hz  (retains 97.4% of signal power, rejects floor)
- moving average N = 0.443/(f_c*dt) = 0.443/(20*0.005) = 4.4 -> N = 5 samples
- time-windowed equivalent T = 25 ms  <- the configured parameter
