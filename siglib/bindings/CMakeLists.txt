# bindings/CMakeLists.txt
#
# Builds the pybind11 Python extension module: siglib_py.so
#
# To build:
#   cd siglib && mkdir build_py && cd build_py
#   cmake .. -DBUILD_BINDINGS=ON -DCMAKE_BUILD_TYPE=Release
#   make -j4
#   # siglib_py.so appears in build_py/bindings/
#
# The main siglib CMakeLists.txt includes this directory only when
# BUILD_BINDINGS=ON to keep the default zero-dependency build intact.

# ── pybind11 via FetchContent ─────────────────────────────────────────────────
include(FetchContent)
FetchContent_Declare(
  pybind11
  GIT_REPOSITORY https://github.com/pybind/pybind11.git
  GIT_TAG        v2.12.0
)
FetchContent_MakeAvailable(pybind11)

# ── Extension module ──────────────────────────────────────────────────────────
pybind11_add_module(siglib_py MODULE siglib_bindings.cpp)
target_link_libraries(siglib_py PRIVATE siglib::siglib)
target_compile_features(siglib_py PRIVATE cxx_std_17)
