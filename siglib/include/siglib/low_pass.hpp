#pragma once
#include <cmath>
#include <cstdint>
#include <limits>
#include <stdexcept>
#include <type_traits>

#include "filter.hpp"
#include "sample.hpp"

namespace siglib {

// ── LowPass<T> — variable-dt first-order IIR low-pass filter ─────────────────
//
// Transfer function (continuous time): H(s) = 1 / (1 + s·τ)
// Discretized with Euler forward method:
//   α  = dt / (τ + dt)            ← computed per sample, handles jitter
//   y[n] = y[n-1] + α·(x[n] − y[n-1])
//
// Why α = dt/(τ+dt) not 1−exp(−dt/τ)?
//   Both are valid approximations; the rational form (dt/(τ+dt)) only requires
//   division, which maps cleanly to fixed-point without a Taylor expansion.
//   At small dt/τ ratios (our case: dt≈5ms, τ=7.96ms) the two differ by
//   only ~0.4% — well within acceptable tolerance.
//
// Why per-sample α?
//   Jitter std = 1.559 ms = 31% of period.  A fixed α computed at 200 Hz
//   would be wrong on every sample.  By computing α = dt/(τ+dt) each call,
//   the filter becomes time-continuous: the math already encodes variable dt,
//   including the 157ms dropout — during the dropout α→1 (filter output jumps
//   toward input, then re-converges).  No special-case dropout handling needed.
//
// Two specializations selected via if constexpr (one template, no duplication):
//
//   FLOAT path (T = float or double):
//     State stored as double.  α computed in double.  No fixed-point.
//
//   INTEGER path (T = int32_t or similar):
//     Fixed-point scheme.  NO float arithmetic in the update loop.
//
//     ┌─ Fixed-point design ────────────────────────────────────────────────┐
//     │                                                                      │
//     │  FRAC = 16  (extra sub-integer bits in the state variable)           │
//     │                                                                      │
//     │  State y_state_ stores  (true_output << FRAC), so the true output    │
//     │  is  y_state_ >> FRAC.                                               │
//     │                                                                      │
//     │  Why the extra bits?  "Deadband bug":                                │
//     │    If state ≈ input, then (x<<FRAC − y_state) is small.              │
//     │    Without FRAC, this rounds to 0 before multiply → filter freezes.   │
//     │    With FRAC=16, the difference is magnified 65536× before rounding,  │
//     │    so the filter keeps converging even on tiny inputs.                │
//     │                                                                      │
//     │  alpha_q15 = round(α × 2^15)  ∈ [0, 32768]                          │
//     │    = (dt_ns << 15 + den/2) / den,   den = tau_ns + dt_ns            │
//     │    Stored in int32_t (not int16_t) because it can reach 32768.       │
//     │                                                                      │
//     │  Update:                                                             │
//     │    diff     = (x << FRAC) − y_state_        [int64_t, no overflow]  │
//     │    y_state_ += (alpha_q15 × diff + (1<<14)) >> 15   [half-up round] │
//     │                                                                      │
//     │  Output (with saturation):                                           │
//     │    raw = (y_state_ + (1<<(FRAC-1))) >> FRAC   [half-up round]       │
//     │    clamp to [T_MIN, T_MAX] to guard against pathological inputs      │
//     │                                                                      │
//     └──────────────────────────────────────────────────────────────────────┘
//
template <typename T>
class LowPass : public Filter<T> {
public:
    /// @param cutoff_hz  Filter cutoff frequency in Hz.  From FINDINGS.md: 20 Hz
    ///                   (retains 97.4% of accel signal power).
    /// @param initial    Initial filter state value (default 0).
    explicit LowPass(double cutoff_hz, T initial = T{0})
        : tau_ns_(static_cast<int64_t>(1.0e9 / (2.0 * M_PI * cutoff_hz)))
    {
        if (cutoff_hz <= 0.0) {
            throw std::invalid_argument("LowPass: cutoff_hz must be > 0");
        }
        reset_to(initial);
    }

    T update(const Sample<T>& s) override {
        if (!initialized_) {
            // First sample: initialize state to input, no filtering yet.
            set_state(s.value);
            initialized_ = true;
            prev_ts_ns_  = s.timestamp_ns;
            return s.value;
        }

        const int64_t dt_ns = s.timestamp_ns - prev_ts_ns_;
        prev_ts_ns_ = s.timestamp_ns;

        if constexpr (std::is_floating_point_v<T>) {
            return update_float(s.value, dt_ns);
        } else {
            return update_fixed(s.value, dt_ns);
        }
    }

    void reset() override { reset_to(T{0}); }

private:
    // ── Float path ────────────────────────────────────────────────────────
    T update_float(T x, int64_t dt_ns) {
        const double dt  = static_cast<double>(dt_ns) * 1e-9;
        const double tau = static_cast<double>(tau_ns_) * 1e-9;
        const double alpha = dt / (tau + dt);
        state_double_ += alpha * (static_cast<double>(x) - state_double_);
        return static_cast<T>(state_double_);
    }

    // ── Integer fixed-point path ──────────────────────────────────────────
    // NO float operations below this line.
    T update_fixed(T x, int64_t dt_ns) {
        // Compute alpha_q15 = round(dt/(tau+dt) * 2^15) using integer math.
        // den = tau_ns + dt_ns. Both are nanoseconds (int64_t), no overflow.
        const int64_t den = tau_ns_ + dt_ns;
        // (dt_ns << 15 + den/2) / den — the "+den/2" gives half-up rounding.
        const int32_t alpha_q15 = static_cast<int32_t>(
            (dt_ns * (1 << 15) + den / 2) / den
        );
        // alpha_q15 ∈ [0, 32768]. When dt >> tau, dt/(tau+dt) → 1 → alpha_q15=32768.

        // Update state (held at FRAC=16 extra bits).
        // x << FRAC may overflow int32_t, so promote to int64_t first.
        constexpr int FRAC = 16;
        const int64_t x_shifted = static_cast<int64_t>(x) << FRAC;
        const int64_t diff      = x_shifted - state_fixed_;

        // Half-up rounding: add 2^14 before right-shifting by 15.
        state_fixed_ += (static_cast<int64_t>(alpha_q15) * diff + (1 << 14)) >> 15;

        // Extract output with rounding: add 2^(FRAC-1) then shift right.
        const int64_t raw = (state_fixed_ + (1 << (FRAC - 1))) >> FRAC;

        // Saturate to T's range (defensive: pathological long dropout could push
        // state out of int32_t range if input is at INT32_MAX for many samples).
        constexpr int64_t T_MIN = static_cast<int64_t>(std::numeric_limits<T>::min());
        constexpr int64_t T_MAX = static_cast<int64_t>(std::numeric_limits<T>::max());
        const int64_t clamped = raw < T_MIN ? T_MIN : (raw > T_MAX ? T_MAX : raw);

        return static_cast<T>(clamped);
    }

    // ── Helpers ───────────────────────────────────────────────────────────
    void set_state(T v) {
        if constexpr (std::is_floating_point_v<T>) {
            state_double_ = static_cast<double>(v);
        } else {
            constexpr int FRAC = 16;
            state_fixed_ = static_cast<int64_t>(v) << FRAC;
        }
    }

    void reset_to(T v) {
        initialized_ = false;
        prev_ts_ns_  = 0;
        set_state(v);
    }

    const int64_t tau_ns_;    // τ in nanoseconds = 1e9 / (2π · fc)

    bool    initialized_{false};
    int64_t prev_ts_ns_{0};

    // State storage — only one is used depending on T, but both are trivially
    // sized and the compiler will likely optimize away the unused member.
    double  state_double_{0.0};
    int64_t state_fixed_{0};
};

}  // namespace siglib
