# fix_env.sh — workaround for a colcon/ament_cmake quirk on this machine.
#
# sig_msgs and sig_proc_cpp's generated package.dsv files are missing the
# ament_prefix_path hook registration (present for sig_proc_py/sig_publisher,
# absent for these two ament_cmake packages), so `source install/setup.bash`
# alone doesn't add them to AMENT_PREFIX_PATH, and `ros2 run`/`ros2 pkg`
# can't find them even though they built successfully.
#
# Usage, in every terminal pane, after sourcing install/setup.bash:
#   source install/setup.bash
#   source fix_env.sh
export AMENT_PREFIX_PATH="$PWD/install/sig_msgs:$PWD/install/sig_proc_cpp:$AMENT_PREFIX_PATH"
echo "fix_env.sh: added sig_msgs and sig_proc_cpp to AMENT_PREFIX_PATH"

# siglib_py (the pybind11 extension module) is built separately via plain
# CMake in siglib/build_py — it is NOT part of the colcon/ROS2 workspace,
# so it never lands on PYTHONPATH automatically. sig_proc_py's
# `import siglib_py` needs this directory added manually.
export PYTHONPATH="$PWD/../siglib/build_py/bindings:$PYTHONPATH"
echo "fix_env.sh: added siglib_py build dir to PYTHONPATH"
