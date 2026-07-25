#pragma once
#include <algorithm>
#include <cstdint>
#include <stdexcept>
#include <vector>

#include "filter.hpp"
#include "sample.hpp"

namespace siglib {

// ── Median<T> — sliding-window median filter ──────────────────────────────────
//
// Design rationale:
//   Moving average and IIR filters are linear — an outlier (e.g., a spike
//   during the 157ms dropout) shifts the output and takes several samples to
//   recover.  Median is non-linear: it returns the middle value regardless of
//   outlier magnitude, so a single bad sample leaves zero trace.
//
// Algorithm:
//   1. Maintain a circular window of the last `capacity` samples.
//   2. On update(), append the new value, evict the oldest if full.
//   3. Copy the window to a sort buffer, std::nth_element to find the median.
//   4. Return the middle element (lower median for even-sized windows).
//
// std::nth_element is O(N) average — faster than full sort for small N.
//
// No-allocation invariant:
//   Both buf_ (ring) and sort_ (scratch) are reserved at construction.
//   erase(begin()) and push_back() within capacity never reallocate.
//
// Factory registration:
//   This file registers four variants (I32/F32 × two names) automatically
//   via file-scope Registrar statics.  Zero edits to any other file required.
//   This proves the Open/Closed design: add Median by adding ONE file.

template <typename T>
class Median : public Filter<T> {
public:
    explicit Median(std::size_t capacity = 5)
        : capacity_(capacity) {
        if (capacity == 0) throw std::invalid_argument("Median: capacity must be >= 1");
        buf_.reserve(capacity);
        sort_.reserve(capacity);
    }

    T update(const Sample<T>& s) override {
        // Evict oldest if at capacity
        if (buf_.size() == capacity_) {
            buf_.erase(buf_.begin());  // O(N) shift, N≤5 in practice
        }
        buf_.push_back(s.value);

        // Copy to scratch buffer and find median in O(N) average time
        sort_.assign(buf_.begin(), buf_.end());
        const std::size_t mid = sort_.size() / 2;
        std::nth_element(sort_.begin(), sort_.begin() + static_cast<std::ptrdiff_t>(mid), sort_.end());
        return sort_[mid];
    }

    void reset() override {
        buf_.clear();
        sort_.clear();
    }

private:
    const std::size_t capacity_;
    std::vector<T> buf_;   // window in arrival order
    std::vector<T> sort_;  // scratch for nth_element (avoids in-place sort on buf_)
};

}  // namespace siglib

// ── Self-registration (the extensibility demo) ────────────────────────────────
//
// These four static objects are constructed before main() runs.
// Their constructors call FilterFactory<T>::register_creator(), inserting
// "median_i32" and "median_f32" into the factory map.
//
// Result: any code that includes this header gets Median in the factory.
// No edits to filter.hpp, moving_average.hpp, low_pass.hpp, or CMakeLists.txt.
namespace {
siglib::Registrar<int32_t, siglib::Median<int32_t>> _reg_median_i32("median_i32");
siglib::Registrar<float,   siglib::Median<float>>   _reg_median_f32("median_f32");
}  // anonymous namespace
