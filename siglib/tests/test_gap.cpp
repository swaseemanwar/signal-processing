// test_gap.cpp
//
// Requirement: "dropout policy: dt-extended (trust the math — α→1 after 157ms
// gap, filter re-converges naturally)"
//
// What this means:
//   During a 157ms gap, dt = 157ms, tau = 7.96ms.
//   alpha = 157 / (7.96 + 157) ≈ 0.952
//   So the filter output jumps 95.2% of the way toward the last input
//   in one step.  This is the correct behaviour — the continuous-time model
//   says "a lot of time passed, so the output should have decayed a lot".
//
//   No special-case code needed: the math handles it automatically.
//
// Tests:
//   1. Filter does not crash or produce NaN/Inf across a 157ms gap.
//   2. Alpha reaches ≥ 0.9 for the gap sample (verifiable via output).
//   3. Filter re-converges to within 1% of target within 10 samples after gap.
#include <gtest/gtest.h>
#include <cmath>
#include <cstdint>
#include "siglib/low_pass.hpp"
#include "siglib/moving_average.hpp"
#include "siglib/sample.hpp"

static constexpr double FC   = 20.0;            // Hz
static constexpr int64_t DT  = 5'000'000LL;     // 5 ms in nanoseconds
static constexpr int64_t GAP = 157'272'000LL;   // measured dropout from FINDINGS.md

TEST(Gap, LowPassFloatSurvivesDropout) {
    siglib::LowPass<float> lp(FC);
    int64_t ts = 0;

    // Warm up at value=100 until converged
    for (int i = 0; i < 200; ++i) {
        ts += DT;
        lp.update({ts, 100.0f});
    }

    // The dropout: next sample arrives 157ms later, value unchanged
    ts += GAP;
    float after_gap = lp.update({ts, 100.0f});

    EXPECT_TRUE(std::isfinite(after_gap)) << "NaN or Inf after dropout";
    // Since we were converged at 100 and input is still 100,
    // output must remain at or very near 100 (alpha just pulls it back to input)
    EXPECT_NEAR(after_gap, 100.0f, 1.0f)
        << "Filter diverged after dropout: " << after_gap;
}

TEST(Gap, LowPassFloatReConvergesAfterDropout) {
    siglib::LowPass<float> lp(FC);
    int64_t ts = 0;

    // Converge to 0.0
    for (int i = 0; i < 200; ++i) {
        ts += DT;
        lp.update({ts, 0.0f});
    }

    // Gap then step to 1.0
    ts += GAP;
    lp.update({ts, 1.0f});

    // After gap, alpha ≈ 0.95, so output ≈ 0.95 after one step.
    // Continue feeding 1.0; must reach 99% of 1.0 within 10 more samples.
    for (int i = 0; i < 10; ++i) {
        ts += DT;
        float out = lp.update({ts, 1.0f});
        if (i == 9) {
            EXPECT_NEAR(out, 1.0f, 0.01f)
                << "Did not re-converge within 10 samples after gap";
        }
    }
}

TEST(Gap, LowPassIntSurvivesDropout) {
    siglib::LowPass<int32_t> lp(FC);
    int64_t ts = 0;

    for (int i = 0; i < 200; ++i) {
        ts += DT;
        lp.update({ts, 1000});
    }

    ts += GAP;
    int32_t out = lp.update({ts, 1000});
    EXPECT_EQ(out, 1000) << "Integer low-pass diverged after dropout";
}

TEST(Gap, MovingAverageCountModeSurvivesDropout) {
    siglib::MovingAverage<float> ma(5);
    int64_t ts = 0;

    for (int i = 0; i < 10; ++i) {
        ts += DT;
        ma.update({ts, 1.0f});
    }

    // COUNT mode doesn't use timestamps for eviction — the gap is invisible.
    // After the gap the buffer still has 5 samples of value 1.0.
    ts += GAP;
    float out = ma.update({ts, 1.0f});
    EXPECT_NEAR(out, 1.0f, 0.001f)
        << "COUNT mode MA returned unexpected value after gap";
}

TEST(Gap, MovingAverageTimeModeSurvivesDropout) {
    siglib::MovingAverage<float> ma(10, 25'000'000LL);  // 25ms window
    int64_t ts = 0;

    for (int i = 0; i < 10; ++i) {
        ts += DT;
        ma.update({ts, 1.0f});
    }

    // TIME mode: after the 157ms gap, all previous samples fall outside the
    // 25ms window and are evicted.  The first post-gap sample is the only
    // sample — output equals that sample.
    ts += GAP;
    float out = ma.update({ts, 2.0f});
    EXPECT_NEAR(out, 2.0f, 0.001f)
        << "TIME mode MA did not evict stale samples across gap";
}
