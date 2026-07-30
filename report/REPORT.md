# Signal Processing Pipeline — Project Report

**Author:** Waseem Anwar
**Task:** Robotics Software Engineer — Signal Processing Pipeline (ROS2)

---

## 4.1 Standard Sections

### Design philosophy

The single organizing principle of this project is **strict separation between ROS-independent processing logic and ROS integration**. The core filters live in `siglib/`, a standalone C++ library that builds and tests with plain CMake and has zero ROS dependencies. The ROS2 packages in `ros2_ws/` are thin adapters that subscribe to topics, hand samples to `siglib`, and republish the results. ROS2 is treated purely as a transport and deployment layer, never as a place where signal-processing logic lives.

This separation is not cosmetic. It is what allows the library to be unit-tested without a ROS environment, reused in non-ROS contexts (simulation, offline analysis), and wrapped cleanly by pybind11 without dragging ROS headers into Python.

### The measurement that drives every decision

Before any filter was written, the provided `sensor_log.csv` was characterized (see 4.2). The dominant finding — **timing jitter with a standard deviation of 31% of the nominal sample period** — invalidates the textbook fixed-coefficient filter and forces a per-sample, timestamp-driven design throughout. Every subsequent decision (per-sample α, time-windowed averaging, dropout handling, the `Sample<T>` carrying its own timestamp) is a consequence of taking that measurement seriously.

### Project structure

```
signal-pipeline/
├── siglib/                     # standalone C++ library (no ROS)
│   ├── include/siglib/         #   header-only, template filters
│   │   ├── sample.hpp          #   Sample<T> — timestamped value
│   │   ├── filter.hpp          #   Filter<T> interface + self-registering factory
│   │   ├── low_pass.hpp        #   variable-dt IIR (float + fixed-point paths)
│   │   ├── moving_average.hpp  #   count- and time-window modes
│   │   └── median.hpp          #   extensibility demo — added with zero edits
│   ├── bindings/               #   pybind11 module (siglib_py)
│   ├── tests/                  #   GoogleTest suite
│   └── CMakeLists.txt          #   builds standalone: cmake .. && make
├── ros2_ws/src/
│   ├── sig_msgs/               #   StampedInt64, StampedFloat64 messages
│   ├── sig_publisher/          #   synthetic + replay publisher (Python)
│   ├── sig_proc_cpp/           #   C++ processing node (links siglib directly)
│   └── sig_proc_py/            #   Python processing node (uses siglib_py)
├── analysis/                   #   noise characterization scripts + FINDINGS.md
├── report/                     #   this report + figures
└── data/sensor_log.csv         #   provided dataset
```

### Prerequisites

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS2 | Jazzy |
| Compiler | GCC 13+ / C++17 |
| Python | 3.12 |
| CMake | ≥ 3.16 |
| Python libs (analysis) | numpy, pandas, scipy, matplotlib |

GoogleTest and pybind11 are fetched automatically at configure time via CMake `FetchContent` — no system install required.

### Build instructions

**Standalone library (proves ROS-independence):**
```bash
cd siglib
mkdir build && cd build
cmake ..
make -j4
ctest --output-on-failure          # runs the full test suite
```

**Python bindings (separate, opt-in build):**
```bash
cd siglib
mkdir build_py && cd build_py
cmake .. -DBUILD_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
make -j4                            # produces siglib_py*.so in build_py/bindings/
```

**Full ROS2 workspace:**
```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### Execution instructions

**Synthetic mode with jitter and drops:**
```bash
ros2 run sig_publisher sig_publisher --ros-args \
  -p mode:=synthetic -p jitter:=0.31 -p drop_rate:=0.02
```

**Replay mode (real sensor_log.csv at original timing):**
```bash
ros2 run sig_publisher sig_publisher --ros-args \
  -p mode:=replay -p replay:=/absolute/path/to/data/sensor_log.csv
```

**Processing nodes:**
```bash
ros2 run sig_proc_cpp sig_proc_cpp_node          # C++ path
ros2 run sig_proc_py  sig_proc_py                # Python (pybind11) path
```

**Parity check (C++ vs Python numerical match):**
```bash
# sig_proc_py output is remapped to /py/... so the two nodes' outputs can be compared
ros2 run sig_proc_py sig_proc_py --ros-args \
  -r /filtered/encoder/lowpass:=/py/filtered/encoder/lowpass \
  -r /filtered/encoder/mavg:=/py/filtered/encoder/mavg \
  -r /filtered/accel/lowpass:=/py/filtered/accel/lowpass \
  -r /filtered/accel/mavg:=/py/filtered/accel/mavg
ros2 run sig_proc_py parity_check
```

### Verification steps

```bash
ros2 topic list                                  # see /sensor/* and /filtered/* topics
ros2 topic echo /filtered/accel/lowpass          # inspect filtered output live
ros2 topic hz /sensor/accel                      # confirm ~200 Hz (with jitter spread)
ros2 run sig_proc_py parity_check                # prints "X/Y passed (Z%)" — expect 100%
```

The parity check is the end-to-end correctness proof: it correlates C++ and Python outputs by `header.stamp` and verifies integer streams are **bit-exact** and float streams match within **rtol = 1e-6**. Observed result on the real log: **100.0% passed across thousands of samples**.

### Message design decision: two topics, not one custom message

Encoder (int) and accel (float) are published on **two separate topics** (`/sensor/encoder`, `/sensor/accel`) using two custom message types (`StampedInt64`, `StampedFloat64`), rather than one combined message. Justification: the two streams are logically independent, a consumer may want only one of them, and the type-specific messages keep the integer stream genuinely integer end-to-end (no float field forcing a promotion). Each message carries a `std_msgs/Header` so the sensor-acquisition timestamp travels with the data.

### QoS decision

All publishers and subscribers use **Reliable + KeepLast(100)**. Reliable is chosen deliberately: best-effort delivery would re-introduce random drops at the transport layer, contaminating the controlled `--drop-rate` experiments where the drop probability is a known input. KeepLast(100) buffers bursts without unbounded memory growth.

### Timestamp decision (the most important integration choice)

Both processing nodes compute `dt` from `msg.header.stamp` — the **sensor acquisition time** — and never from `rclcpp::now()` / `rclpy.clock.now()`. Arrival time equals acquisition time *plus transport jitter* (serialization, middleware, scheduling), which varies per message. Since the filter's smoothing coefficient α is derived from `dt`, using arrival time would inject transport jitter into every α computation and corrupt the output. Using the header timestamp keeps `dt` equal to the true inter-sample interval.

---

## 4.2 Data-Grounded Filter Justification

All filter parameters are derived from measurements of the provided `sensor_log.csv` (11,808 samples, ~60 s). Analysis scripts: `analysis/explore.py` (timing), `analysis/noise2.py` (noise + spectrum).

### Timing characterization

| Metric | Value | Implication |
|---|---|---|
| Median dt | 5.001 ms | Nominal rate = 200.0 Hz |
| dt standard deviation | 1.559 ms | **31% of the period** — severe jitter |
| Min dt | 3.644 ms | Two-sided: samples arrive early too |
| Largest gap | 157.3 ms at t = 27.4 s (row 5412) | ~31× normal spacing — the dropout |

**Consequence:** a filter that computes its coefficient once from the nominal 5 ms period would be wrong on nearly every sample. The jitter is not small noise around a stable rate — it is 31% of the quantity the filter depends on. This mandates **per-sample dt**, recomputed from timestamps, everywhere in the pipeline.

### Encoder channel (integer stream): quantization

Method: the greatest common divisor of all non-zero count *steps* was computed. GCD = 1, meaning the least significant bit is 1 count — the encoder is not quantized more coarsely than its raw unit. The naïve "minimum step" heuristic gave 66, but that measures the slowest observed velocity, not the quantization step, and was rejected. Resulting quantization noise (uniform over one LSB): **std = 1/√12 = 0.289 counts**.

### Accel channel (float stream): noise

The noise standard deviation was estimated by two independent methods that agree:

- **PSD floor** (flat high-frequency region of the Welch spectrum above 60 Hz): σ ≈ 0.052 m/s²
- **Power-fraction** (0.2% of total power spread over the top 60% of the band): σ ≈ 0.060 m/s²

A Savitzky-Golay residual estimator was **rejected** because its estimate grew with window length (0.20 → 0.95), i.e. it was counting real motion as noise. Signal std ≈ 1.034 m/s², giving an SNR of roughly **25 dB**.

### Spectral content and the choice of cutoff

Welch PSD (data linearly interpolated to a uniform grid first, since Welch assumes uniform sampling) gives cumulative signal power by band:

| Band | Cumulative power |
|---|---|
| ≤ 5 Hz | 44.5% |
| ≤ 10 Hz | 77.9% |
| **≤ 20 Hz** | **97.4%** |
| ≤ 40 Hz | 99.8% |

**Chosen low-pass cutoff: fc = 20 Hz.** This retains 97.4% of the true signal power while attenuating the noise floor, which lives above 60 Hz. Going higher (40 Hz) admits more noise for only 2.4% more signal; going lower begins to cut real motion. This is a data-driven choice, not a round number.

**Chosen moving-average window: T = 25 ms.** Derived from the equivalent-smoothing relation N = 0.443/(fc·dt) = 0.443/(20·0.005) ≈ 4.4 → 5 samples, which at 200 Hz corresponds to a 25 ms time window. The **time-windowed** form (not a fixed sample count) is the one actually configured, because under 31% jitter "5 samples back" spans anywhere from ~15 ms to ~35 ms of real time; a time window covers exactly 25 ms regardless of how samples happen to be spaced.

### Before/after on the real log

The following figures (in `report/figures/`) show filter behavior on the real data, including across the dropout:

- `encoder_full.png` — raw vs low-pass vs moving-average on the integer encoder stream
- `accel_lowpass_full.png` — raw vs low-pass on the accel stream
- `accel_mavg_full.png` — raw vs moving-average on the accel stream
- `dropout_zoom.png` — zoomed view at t ≈ 27.4 s showing filter behavior across the 157 ms gap

Across the dropout, the low-pass output does not diverge or produce NaN: the large `dt` drives α ≈ 0.952, so the filter tracks the first post-gap sample aggressively and re-converges within a few samples. The time-windowed moving average evicts all pre-gap samples (they fall outside the 25 ms window) and resumes cleanly from the first post-gap sample. Neither requires special-case dropout code — see 4.1 and the dropout discussion below.

### Dropout policy (150 ms gap): dt-extended, no special case

The filters treat the gap as a single large `dt` rather than resetting or interpolating. Justification: the continuous-time IIR model already encodes "a lot of time passed" through α = dt/(τ+dt) → 1. Resetting would discard valid state; interpolating would invent data that was never measured. A hardcoded dropout threshold would introduce an arbitrary magic number and a discontinuity in behavior right at the threshold. Trusting the math generalizes correctly to a 50 ms gap, a 157 ms gap, or a 2 s gap with no boundary to tune. This is verified by `tests/test_gap.cpp`, which uses the exact measured 157.272 ms gap and asserts no NaN/Inf, bounded output, and re-convergence within 10 samples.

---

## 4.3 Benchmark Notes (Bonus)

A formal latency/throughput sweep across 10 Hz → 20 kHz was not run as a standalone harness for this submission. The architecture nonetheless anticipates the expected bottleneck, and two design features directly target it:

- **Batch `.process()` binding with GIL released.** The per-sample `.update()` call has Python↔C++ boundary overhead on every sample; for a 12,000-sample run this dominates. The batch API loops entirely in C++ and releases the GIL, so at high rates the Python-bound path is not paying per-sample crossing costs. The expected bottleneck at the highest rates is therefore the Python/C++ boundary for `.update()`, and cache/copy behavior for `.process()` — not allocation, because of the next point.
- **Zero steady-state allocation.** Both moving-average buffers are `reserve()`-d at construction; no `new`/`malloc`/vector growth occurs in the update path. This is asserted by `tests/test_alloc.cpp`, which overrides global `operator new` to count allocations and requires zero across 1,000 steady-state updates. This removes allocation as a throughput bottleneck by construction.

---

## Appendix: Correctness test suite

The `siglib` test suite verifies each non-obvious design claim rather than just "it runs":

| Test | Proves |
|---|---|
| `test_jitter.cpp` | Variable-dt output matches an independent reference IIR (±40% jitter) |
| `test_gap.cpp` | Survives and re-converges across the exact 157 ms dropout |
| `test_overflow.cpp` | int64 accumulator does not wrap at INT32_MAX inputs (bit-budget) |
| `test_rounding.cpp` | Integer paths do not bias toward −∞ (truncate-toward-zero) |
| `test_alloc.cpp` | Zero heap allocations in steady state |
| `test_factory.cpp` | A new filter (median) registers via new file only, no existing edits |

Together with the live C++/Python parity check (100% on the real log), these form the evidence base for the correctness claims above.
