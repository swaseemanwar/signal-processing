#pragma once
#include <cstdint>

namespace siglib {

/// A single sensor reading paired with its acquisition timestamp.
///
/// Why int64_t nanoseconds?
///   - ROS2 header.stamp encodes time as int64_t ns.
///   - Integer subtraction is exact: dt_ns = b.timestamp_ns - a.timestamp_ns
///     has zero floating-point rounding error.
///   - int64_t holds ~292 years of nanoseconds before overflow.
///   - Passing dt_ns to filter math (which divides by tau_ns + dt_ns) keeps
///     everything in integer arithmetic until we deliberately convert.
///
/// Why a template?
///   - Encoder stream: int32_t counts.
///   - Accel stream:   float m/s².
///   - One type, zero code duplication.
template <typename T>
struct Sample {
    int64_t timestamp_ns;   ///< acquisition time, nanoseconds since epoch
    T       value;          ///< sensor reading in stream-native units
};

}  // namespace siglib
