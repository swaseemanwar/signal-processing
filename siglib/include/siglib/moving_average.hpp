#pragma once
#include <cassert>
#include <cstdint>
#include <stdexcept>
#include <type_traits>
#include <vector>

#include "filter.hpp"
#include "sample.hpp"

namespace siglib {

// ── Accumulator type selection ────────────────────────────────────────────────
//
// Bit-budget proof for the integer path:
//   int32_t max value = 2^31 - 1 ≈ 2.1×10⁹
//   Sum of N such values needs: 31 + ceil(log2(N)) bits
//   For N=5: 31 + 3 = 34 bits  → int64_t (63 usable bits) is safe.
//   Safe for any N ≤ 2^32 when values ≤ INT32_MAX.
//
// For float: double accumulator avoids catastrophic cancellation at large N.
//
// std::conditional_t picks at compile time — one template, zero duplication.
template <typename T>
using AccumType = std::conditional_t<std::is_integral_v<T>, int64_t, double>;

// ── MovingAverage<T> ──────────────────────────────────────────────────────────
//
// Two eviction modes, selected at construction:
//
//   COUNT mode  (window_ns == 0):
//     Keep the last `capacity` samples by count.
//     When full, the oldest sample is dropped before adding the new one.
//     Output = sum / count.
//
//   TIME-WINDOW mode  (window_ns > 0):
//     Keep all samples where timestamp ≥ (newest_timestamp − window_ns).
//     This is what "T = 25 ms" from FINDINGS.md means.
//     Why time-window instead of count?  Jitter is 31% of the period, so
//     "5 samples back in count" spans anywhere from ~15ms to ~35ms of real
//     time.  Time-window is exact.
//     Output = sum / count.
//
// No-allocation invariant after construction:
//   buf_ and ts_ are reserved to `capacity` in the constructor.
//   erase(begin()) on a reserved vector shifts elements but never reallocates
//   (size goes down by 1, then push_back brings it back up — always ≤ capacity).
//   Proved by the operator-new override in tests/test_alloc.cpp.
template <typename T>
class MovingAverage : public Filter<T> {
public:
    /// @param capacity   Maximum number of samples to buffer.  Must be ≥ 1.
    ///                   In COUNT mode: fixed window size.
    ///                   In TIME mode:  upper bound (prevents unbounded growth
    ///                   if caller violates time-ordering assumption).
    /// @param window_ns  0 → COUNT mode.  > 0 → TIME-WINDOW mode (nanoseconds).
    explicit MovingAverage(std::size_t capacity, int64_t window_ns = 0)
        : capacity_(capacity), window_ns_(window_ns) {
        if (capacity == 0) {
            throw std::invalid_argument("MovingAverage: capacity must be >= 1");
        }
        // All heap activity happens here. After this, no more allocations.
        buf_.reserve(capacity);
        ts_.reserve(capacity);
    }

    T update(const Sample<T>& s) override {
        // Step 1: evict stale entries
        if (window_ns_ > 0) {
            // TIME mode: remove anything older than window_ns before newest
            const int64_t cutoff = s.timestamp_ns - window_ns_;
            while (!buf_.empty() && ts_.front() < cutoff) {
                accum_ -= static_cast<AccumType<T>>(buf_.front());
                buf_.erase(buf_.begin());   // O(N) shift, N≤5 in spec — fine
                ts_.erase(ts_.begin());
            }
        } else {
            // COUNT mode: drop oldest if at capacity
            if (buf_.size() == capacity_) {
                accum_ -= static_cast<AccumType<T>>(buf_.front());
                buf_.erase(buf_.begin());
                ts_.erase(ts_.begin());
            }
        }

        // Step 2: append new sample (push_back never allocates; we reserved)
        accum_ += static_cast<AccumType<T>>(s.value);
        buf_.push_back(s.value);
        ts_.push_back(s.timestamp_ns);

        // Step 3: compute output
        // Integer division truncates toward zero.
        // For symmetric data this is unbiased.
        // The rounding test verifies there is no systematic -∞ bias on negative
        // values (contrast: floor division would skew negative values down).
        return static_cast<T>(accum_ / static_cast<AccumType<T>>(buf_.size()));
    }

    void reset() override {
        buf_.clear();   // clears without freeing reserved memory
        ts_.clear();
        accum_ = AccumType<T>{0};
    }

    // Accessors for tests
    std::size_t size()     const { return buf_.size(); }
    std::size_t capacity() const { return capacity_; }

private:
    const std::size_t capacity_;
    const int64_t     window_ns_;

    std::vector<T>       buf_;       // values in arrival order, front = oldest
    std::vector<int64_t> ts_;        // timestamps in arrival order
    AccumType<T>         accum_{0};  // running sum, avoids recomputing each time
};

}  // namespace siglib
