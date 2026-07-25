# ── GoogleTest via FetchContent ───────────────────────────────────────────────
# FetchContent downloads and builds GoogleTest at configure time.
# This means no system-level gtest install is required — the library is
# entirely self-contained.  Zero ROS dependencies.
include(FetchContent)
FetchContent_Declare(
  googletest
  GIT_REPOSITORY https://github.com/google/googletest.git
  GIT_TAG        v1.14.0
)
# Prevent GoogleTest from overriding compiler/linker settings
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)

# ── Helper macro: add_siglib_test(name) ──────────────────────────────────────
# Creates an executable from tests/test_<name>.cpp, links siglib + gtest,
# and registers it with ctest.
macro(add_siglib_test name)
  add_executable(test_${name} test_${name}.cpp)
  target_link_libraries(test_${name} PRIVATE siglib::siglib GTest::gtest_main)
  target_compile_features(test_${name} PRIVATE cxx_std_17)
  gtest_discover_tests(test_${name})
endmacro()

add_siglib_test(overflow)    # MA accumulator overflow at INT32_MAX inputs
add_siglib_test(rounding)    # negative values don't bias toward -∞
add_siglib_test(jitter)      # synthetic non-uniform timestamps vs reference
add_siglib_test(gap)         # 157ms dropout — filter survives and reconverges
add_siglib_test(alloc)       # zero heap calls after warmup
add_siglib_test(factory)     # self-registering factory: median appears with no edits
