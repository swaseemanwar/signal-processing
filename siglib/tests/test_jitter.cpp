// test_jitter.cpp
//
// Requirement: "handles non-uniform dt — alpha recomputed per sample"
//
// Strategy: construct a small sequence of samples with known non-uniform
// timestamps, compute the expected IIR output analytically, and verify
// the filter matches.
//
// Why this matters: a fixed-dt filter computes alpha once from the nominal
// 5ms period.  With 31% jitter, individual dt can be 3.6ms to 9ms+.
// Using the fixed alpha with wrong dt accumulates error every sample.
// The variable-dt filter recomputes alpha each call, so it tracks the
// correct continuous-time solution.
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include <vector>
#include "siglib/low_pass.hpp"
#include "siglib/moving_average.hpp"
#include "siglib/sample.hpp"

// Manually compute the expected IIR output for a known sequence.
static std::vector<float> reference_iir(
    const std::vector<std::pair<int64_t, float>>& samples,
    double cutoff_hz)
{
    const double tau = 1.0 / (2.0 * M_PI * cutoff_hz);
    std::vector<float> out;
    double y = 0.0;
    bool first = true;
    int64_t prev_ts = 0;
    for (auto& [ts, x] : samples) {
        if (first) { y = x; first = false; prev_ts = ts; out.push_back(static_cast<float>(y)); continue; }
        double dt = static_cast<double>(ts - prev_ts) * 1e-9;
        double alpha = dt / (tau + dt);
        y += alpha * (x - y);
        prev_ts = ts;
        out.push_back(static_cast<float>(y));
    }
    return out;
}

TEST(Jitter, VariableDtMatchesReference) {
    // Non-uniform timestamps: alternating short and long intervals
    // (simulates ±40% jitter — worse than our measured 31%)
    const double fc = 20.0;
    std::vector<std::pair<int64_t, float>> samples = {
        {        0LL, 0.0f},
        { 3'000'000LL, 1.0f},   // 3 ms
        { 8'000'000LL, 0.5f},   // 5 ms
        {11'000'000LL, 2.0f},   // 3 ms
        {18'000'000LL, 0.0f},   // 7 ms
        {21'500'000LL, 1.0f},   // 3.5 ms
        {28'500'000LL, 1.0f},   // 7 ms
    };

    auto ref = reference_iir(samples, fc);

    siglib::LowPass<float> lp(fc);
    std::vector<float> got;
    for (auto& [ts, x] : samples) {
        got.push_back(lp.update({ts, x}));
    }

    for (std::size_t i = 0; i < ref.size(); ++i) {
        EXPECT_NEAR(got[i], ref[i], ref[i] * 1e-5f + 1e-6f)
            << "Mismatch at sample " << i
            << ": expected " << ref[i] << " got " << got[i];
    }
}

TEST(Jitter, TimeWindowMAEvictsByTimestamp) {
    // TIME-WINDOW mode: window = 20ms.
    // Feed samples at 3ms and 7ms intervals alternately.
    // The MA must evict based on timestamps, not count.
    siglib::MovingAverage<float> ma(10, 20'000'000LL);  // 20ms window

    // t=0: one sample
    ma.update({0LL, 10.0f});
    // t=7ms: two samples in window
    ma.update({7'000'000LL, 20.0f});
    // t=14ms: three samples (0, 7, 14) all within 20ms of t=14
    ma.update({14'000'000LL, 30.0f});
    // t=21ms: sample at t=0 is now 21ms ago → evicted. Window: [7, 14, 21]
    float out = ma.update({21'000'000LL, 0.0f});
    // Expected: (20 + 30 + 0) / 3 = 16.666...
    EXPECT_NEAR(out, 50.0f / 3.0f, 0.01f)
        << "Time-window MA did not evict by timestamp";
}
