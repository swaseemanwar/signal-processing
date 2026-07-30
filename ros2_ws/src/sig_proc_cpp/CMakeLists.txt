cmake_minimum_required(VERSION 3.16)
project(sig_proc_cpp)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sig_msgs REQUIRED)
find_package(std_msgs REQUIRED)

# ── siglib (header-only, no ROS dependency) ───────────────────────────────────
# We add siglib directly from the repo rather than installing it system-wide.
# This keeps siglib completely ROS-independent — it can be tested with plain
# cmake even without a ROS environment.
add_library(siglib INTERFACE)
target_include_directories(siglib INTERFACE
  ${CMAKE_CURRENT_SOURCE_DIR}/../../../siglib/include)
target_compile_features(siglib INTERFACE cxx_std_17)

# ── Processing node ───────────────────────────────────────────────────────────
add_executable(sig_proc_cpp_node src/proc_node.cpp)
target_link_libraries(sig_proc_cpp_node siglib)
ament_target_dependencies(sig_proc_cpp_node rclcpp sig_msgs std_msgs)

install(TARGETS sig_proc_cpp_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
