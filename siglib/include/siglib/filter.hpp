#pragma once
#include <cstdint>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include "sample.hpp"

namespace siglib {

// ── Abstract filter interface ─────────────────────────────────────────────────
//
// Every filter exposes one operation: feed a Sample, get back the filtered
// value. The timestamp inside the sample is used to compute dt — the filter
// never trusts a fixed clock interval because jitter is 31% of the period.
//
// Why pure virtual / interface class?
//   The ROS2 node can hold a  unique_ptr<Filter<T>>  and call update() without
//   knowing whether it has a MovingAverage, LowPass, or Median underneath.
//   This is the classic Strategy pattern.
template <typename T>
class Filter {
public:
    virtual ~Filter() = default;

    /// Feed one sample; return the filtered output.
    /// @param s  Sample with timestamp_ns and value.
    /// @return   Filtered value in the same units as s.value.
    virtual T update(const Sample<T>& s) = 0;

    /// Reset internal state to "as if just constructed".
    virtual void reset() = 0;
};

// ── Self-registering factory ──────────────────────────────────────────────────
//
// Goal: add a new filter type (e.g. Median) by creating ONE new header file,
// with ZERO edits to any existing file. This is the Open/Closed Principle.
//
// How it works:
//   1. FilterFactory<T> holds a static map: name → creator function.
//   2. Each filter header defines a local Registrar struct whose constructor
//      calls FilterFactory<T>::register_creator().
//   3. A file-scope static Registrar instance is created at program start
//      (before main()), so registration happens automatically when the header
//      is included.
//
// Consequence: to add Median, just write median.hpp with a Registrar — done.

template <typename T>
class FilterFactory {
public:
    using CreatorFn = std::function<std::unique_ptr<Filter<T>>()>;

    static void register_creator(const std::string& name, CreatorFn fn) {
        registry()[name] = std::move(fn);
    }

    static std::unique_ptr<Filter<T>> create(const std::string& name) {
        auto& r = registry();
        auto it = r.find(name);
        if (it == r.end()) {
            throw std::runtime_error("FilterFactory: unknown filter '" + name + "'");
        }
        return it->second();
    }

private:
    // Meyer's singleton: constructed on first use, avoids static-init-order
    // fiasco (the map exists before any Registrar constructor runs).
    static std::unordered_map<std::string, CreatorFn>& registry() {
        static std::unordered_map<std::string, CreatorFn> r;
        return r;
    }
};

// ── Registrar helper ──────────────────────────────────────────────────────────
//
// Each filter header declares ONE static instance of this in anonymous
// namespace. Its constructor fires at static-init time and inserts the
// creator into the factory map.
//
// Usage in (e.g.) median.hpp:
//
//   namespace { siglib::Registrar<float, MedianF> reg_median_f("median_f"); }
//
template <typename T, typename ConcreteFilter>
struct Registrar {
    explicit Registrar(const std::string& name) {
        FilterFactory<T>::register_creator(
            name, []() { return std::make_unique<ConcreteFilter>(); });
    }
};

}  // namespace siglib
