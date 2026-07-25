cmake_minimum_required(VERSION 3.16)
project(siglib VERSION 0.1 LANGUAGES CXX)

# ── Library target ────────────────────────────────────────────────────────────
# INTERFACE = header-only: no compiled object files.
# Templates must be fully defined where they are instantiated, so all code
# lives in headers. The library target carries include paths and flags that
# propagate to any consumer via target_link_libraries.
add_library(siglib INTERFACE)
add_library(siglib::siglib ALIAS siglib)          # :: prefix catches typos

target_compile_features(siglib INTERFACE cxx_std_17)

target_include_directories(siglib INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

# Warnings only when building (not when installed as a dependency)
target_compile_options(siglib INTERFACE
  $<BUILD_INTERFACE:-Wall -Wextra -Wconversion -Wshadow>)

# ── Tests ─────────────────────────────────────────────────────────────────────
enable_testing()
add_subdirectory(tests)
