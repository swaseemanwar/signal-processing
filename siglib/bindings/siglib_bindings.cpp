// siglib_bindings.cpp
//
// pybind11 module exposing siglib filters to Python.
//
// Exposed types:
//   siglib_py.MovingAverageI32   — MovingAverage<int32_t>
//   siglib_py.MovingAverageF32   — MovingAverage<float>
//   siglib_py.LowPassI32         — LowPass<int32_t>
//   siglib_py.LowPassF32         — LowPass<float>
//
// Each type has:
//   .update(timestamp_ns: int, value: T) -> T        per-sample API
//   .process(timestamps: np.ndarray,
//            values: np.ndarray) -> np.ndarray       batch API (GIL released)
//   .reset()                                         clear internal state
//
// Design decisions:
//
//   py::arg().noconvert() on integer types:
//     Without it, Python would silently accept a float where int32 is expected
//     (1.0 → 1).  With noconvert(), pybind11 raises TypeError immediately.
//     Callers must pass np.int32(x), not a plain Python float.
//     We apply it only to integer-typed filters.
//
//   Batch API (py::array_t<T>):
//     Per-sample .update() has Python→C++ call overhead on every sample.
//     .process() loops entirely in C++, returning a NumPy array — much faster
//     for 12,000-sample runs (60s × 200Hz).
//
//   GIL release (py::gil_scoped_release):
//     Python's Global Interpreter Lock blocks all Python threads while C++ runs.
//     Releasing it for the duration of the batch loop lets ROS2 callbacks
//     and other Python threads run concurrently.

#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <cstdint>
#include <stdexcept>

#include "siglib/moving_average.hpp"
#include "siglib/low_pass.hpp"
#include "siglib/sample.hpp"

namespace py = pybind11;
using namespace siglib;

// ── Batch process helper ───────────────────────────────────────────────────────
// Shared by MA and LowPass — loops C++, releases GIL, returns numpy array.
template <typename FilterT, typename T>
py::array_t<T> batch_process(
    FilterT& filter,
    py::array_t<int64_t> timestamps,
    py::array_t<T>       values)
{
    auto ts_buf  = timestamps.request();
    auto val_buf = values.request();

    if (ts_buf.ndim != 1 || val_buf.ndim != 1) {
        throw std::invalid_argument("timestamps and values must be 1-D arrays");
    }
    if (ts_buf.size != val_buf.size) {
        throw std::invalid_argument("timestamps and values must have the same length");
    }

    const std::size_t n = static_cast<std::size_t>(ts_buf.size);
    py::array_t<T> out(n);

    const int64_t* ts_ptr  = static_cast<const int64_t*>(ts_buf.ptr);
    const T*       val_ptr = static_cast<const T*>(val_buf.ptr);
    T*             out_ptr = static_cast<T*>(out.request().ptr);

    {
        py::gil_scoped_release release;   // allow other Python threads to run
        for (std::size_t i = 0; i < n; ++i) {
            out_ptr[i] = filter.update(Sample<T>{ts_ptr[i], val_ptr[i]});
        }
    }
    return out;
}

// ── bind_moving_average<T> ─────────────────────────────────────────────────────
template <typename T>
void bind_moving_average(py::module_& m, const char* class_name) {
    auto cls = py::class_<MovingAverage<T>>(m, class_name)
        .def(py::init<std::size_t, int64_t>(),
             py::arg("capacity"), py::arg("window_ns") = 0,
             "capacity: max samples (count mode) or buffer ceiling (time mode).\n"
             "window_ns: 0 = COUNT mode.  >0 = TIME-WINDOW mode (nanoseconds).")

        .def("process",
             [](MovingAverage<T>& self,
                py::array_t<int64_t> ts,
                py::array_t<T> vals) {
                 return batch_process<MovingAverage<T>, T>(self, ts, vals);
             },
             py::arg("timestamps"), py::arg("values"),
             "Batch process; returns filtered numpy array. GIL released during loop.")

        .def("reset", &MovingAverage<T>::reset, "Reset internal state.")
        .def_property_readonly("size",     &MovingAverage<T>::size)
        .def_property_readonly("capacity", &MovingAverage<T>::capacity);

    // Per-sample update: integer types get noconvert() to block float→int
    if constexpr (std::is_integral_v<T>) {
        cls.def("update",
                [](MovingAverage<T>& self, int64_t ts, T val) {
                    return self.update(Sample<T>{ts, val});
                },
                py::arg("timestamp_ns"), py::arg("value").noconvert(),
                "Feed one sample; return filtered value.\n"
                "Integer type: value must be the exact integer type (e.g. np.int32), "
                "not a Python float.");
    } else {
        cls.def("update",
                [](MovingAverage<T>& self, int64_t ts, T val) {
                    return self.update(Sample<T>{ts, val});
                },
                py::arg("timestamp_ns"), py::arg("value"),
                "Feed one sample; return filtered value.");
    }
}

// ── bind_low_pass<T> ───────────────────────────────────────────────────────────
template <typename T>
void bind_low_pass(py::module_& m, const char* class_name) {
    auto cls = py::class_<LowPass<T>>(m, class_name)
        .def(py::init<double, T>(),
             py::arg("cutoff_hz"), py::arg("initial") = T{0},
             "cutoff_hz: filter cutoff in Hz.\n"
             "  From FINDINGS.md: 20.0 Hz retains 97.4% of accel signal power.\n"
             "initial: optional initial filter state (default 0).")

        .def("process",
             [](LowPass<T>& self,
                py::array_t<int64_t> ts,
                py::array_t<T> vals) {
                 return batch_process<LowPass<T>, T>(self, ts, vals);
             },
             py::arg("timestamps"), py::arg("values"),
             "Batch process; returns filtered numpy array. GIL released during loop.")

        .def("reset", &LowPass<T>::reset, "Reset internal state.");

    if constexpr (std::is_integral_v<T>) {
        cls.def("update",
                [](LowPass<T>& self, int64_t ts, T val) {
                    return self.update(Sample<T>{ts, val});
                },
                py::arg("timestamp_ns"), py::arg("value").noconvert(),
                "Feed one sample; return filtered value.\n"
                "Integer type: value must be np.int32, not a Python int or float.");
    } else {
        cls.def("update",
                [](LowPass<T>& self, int64_t ts, T val) {
                    return self.update(Sample<T>{ts, val});
                },
                py::arg("timestamp_ns"), py::arg("value"),
                "Feed one sample; return filtered value.");
    }
}

// ── Module ─────────────────────────────────────────────────────────────────────
PYBIND11_MODULE(siglib_py, m) {
    m.doc() = R"doc(
siglib_py — Python bindings for the siglib signal processing library.

Types
-----
MovingAverageI32(capacity, window_ns=0)
MovingAverageF32(capacity, window_ns=0)
LowPassI32(cutoff_hz, initial=0)
LowPassF32(cutoff_hz, initial=0.0)

Each type exposes:
  .update(timestamp_ns: int, value) -> value    per-sample
  .process(timestamps, values) -> np.ndarray    batch (fast)
  .reset()

Integer types enforce strict types (noconvert):
  import numpy as np, siglib_py
  lp = siglib_py.LowPassI32(20.0)
  lp.update(5_000_000, np.int32(100))   # OK
  lp.update(5_000_000, 100.0)           # TypeError: won't silently cast float

Filter parameters from FINDINGS.md:
  cutoff_hz = 20.0           # retains 97.4% of accel signal power
  capacity  = 5              # count-mode MA
  window_ns = 25_000_000     # time-window MA, T = 25 ms
)doc";

    bind_moving_average<int32_t>(m, "MovingAverageI32");
    bind_moving_average<float>  (m, "MovingAverageF32");
    bind_low_pass<int32_t>      (m, "LowPassI32");
    bind_low_pass<float>        (m, "LowPassF32");
}
