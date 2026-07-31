# Signal Processing Pipeline (ROS2)

A modular signal-processing pipeline with strict separation between a ROS-independent core library (`siglib/`) and ROS2 integration (`ros2_ws/`). Filters are data-driven — all parameters were derived from characterizing the provided `sensor_log.csv` before writing a single line of filter code.

## Repository layout

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
│   └── CMakeLists.txt
├── ros2_ws/src/
│   ├── sig_msgs/               #   StampedInt64, StampedFloat64 messages
│   ├── sig_publisher/          #   synthetic + replay publisher (Python)
│   ├── sig_proc_cpp/           #   C++ processing node (links siglib directly)
│   └── sig_proc_py/            #   Python processing node (uses siglib_py)
├── analysis/                   #   noise characterization scripts + FINDINGS.md
├── report/                     #   project report + figures
├── scripts/                    #   C++/Python output comparison
└── data/sensor_log.csv         #   provided dataset
```

## Prerequisites

| Component | Version |
|---|---|
| OS | Ubuntu 24.04 |
| ROS2 | Jazzy |
| Compiler | GCC 13+ / C++17 |
| Python | 3.12 |
| CMake | ≥ 3.16 |
| Python libs (analysis) | `numpy`, `pandas`, `scipy`, `matplotlib` |

> **Note:** GoogleTest and pybind11 are fetched automatically via CMake `FetchContent` — no manual install required.

---

## Build

### 1. Standalone library (no ROS required)

```bash
cd siglib
mkdir build && cd build
cmake ..
make -j4
ctest --output-on-failure    # runs the full test suite
```

### 2. Python bindings (optional)

```bash
cd siglib
mkdir build_py && cd build_py
cmake .. -DBUILD_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
make -j4
# produces siglib_py*.so in build_py/bindings/
```

### 3. Full ROS2 workspace

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash
```

---

## Running

### Publisher — synthetic mode (with jitter and simulated drops)

```bash
ros2 run sig_publisher sig_publisher --ros-args \
  -p mode:=synthetic -p jitter:=0.31 -p drop_rate:=0.02
```

### Publisher — replay mode (real sensor_log.csv at original timing)

```bash
ros2 run sig_publisher sig_publisher --ros-args \
  -p mode:=replay -p replay:=/absolute/path/to/data/sensor_log.csv
```

### Processing nodes (run each in a separate terminal)

```bash
ros2 run sig_proc_cpp sig_proc_cpp_node    # C++ path
ros2 run sig_proc_py  sig_proc_py          # Python/pybind11 path
```

### C++ vs Python parity check

Run both processing nodes first, then:

```bash
# Remap Python node outputs to /py/... namespace for comparison
ros2 run sig_proc_py sig_proc_py --ros-args \
  -r /filtered/encoder/lowpass:=/py/filtered/encoder/lowpass \
  -r /filtered/encoder/mavg:=/py/filtered/encoder/mavg \
  -r /filtered/accel/lowpass:=/py/filtered/accel/lowpass \
  -r /filtered/accel/mavg:=/py/filtered/accel/mavg

# Then run the parity checker
ros2 run sig_proc_py parity_check
# Expected output: 100.0% passed
```

---

## Verification

```bash
ros2 topic list                          # lists /sensor/* and /filtered/* topics
ros2 topic echo /filtered/accel/lowpass  # inspect live filtered output
ros2 topic hz /sensor/accel              # confirm ~200 Hz (with jitter spread)
ros2 run sig_proc_py parity_check        # prints "X/Y passed (Z%)" — expect 100%
```

---

## Test suite

Run from `siglib/build/` after building:

```bash
ctest --output-on-failure
```

| Test | What it proves |
|---|---|
| `test_jitter.cpp` | Variable-dt output matches reference IIR under ±40% jitter |
| `test_gap.cpp` | Survives and re-converges across the exact 157 ms dropout |
| `test_overflow.cpp` | int64 accumulator does not wrap at INT32_MAX inputs |
| `test_rounding.cpp` | Integer paths do not bias toward −∞ |
| `test_alloc.cpp` | Zero heap allocations in steady state |
| `test_factory.cpp` | A new filter (median) registers via new file only — no existing edits |

---

## Design notes

**Why variable-dt?** The sensor log shows timing jitter with σ = 31% of the nominal 5 ms period. A fixed-coefficient filter would be wrong on nearly every sample. All filters recompute their coefficients from `msg.header.stamp` on each sample.

**Why two topics?** Encoder (int) and accel (float) are published separately on `/sensor/encoder` and `/sensor/accel` using `StampedInt64` and `StampedFloat64`. This keeps the integer stream genuinely integer end-to-end and lets consumers subscribe to only what they need.

**Why header timestamp, not arrival time?** Both processing nodes compute `dt` from `msg.header.stamp` (sensor acquisition time), not `rclcpp::now()`. Arrival time includes transport jitter, which would corrupt the filter's smoothing coefficient.

**Dropout handling (157 ms gap at t ≈ 27.4 s):** No special-case code. The large `dt` drives α → 1 in the IIR, so the filter tracks the first post-gap sample aggressively and re-converges within a few samples. The time-windowed moving average evicts all pre-gap samples naturally. Verified by `test_gap.cpp`.

See [`report/REPORT.md`](report/REPORT.md) for the full data analysis and design rationale.
