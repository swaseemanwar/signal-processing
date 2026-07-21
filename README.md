# Signal Processing Pipeline (ROS2)

Modular signal-processing pipeline with strict separation between a
ROS-independent core library (`siglib/`) and ROS2 integration (`ros2_ws/`).

## Problem
[ Non-uniform dt, integer overflow safety, no steady-state allocation,
extensibility without editing existing files, data-grounded tuning.]
## Repository layout
- `siglib/`    - standalone C++ library + pybind11 bindings (builds without ROS)
- `ros2_ws/`   - ROS2 packages: publisher, C++ node, Python node
- `analysis/`  - noise characterization of sensor_log.csv (Part 4.2)
- `report/`    - project report
- `scripts/`   - C++/Python output comparison

