// test_rounding.cpp
//
// Requirement: "negative values don't bias toward -∞"
//
// C++ integer division truncates toward zero (since C++11), so:
//   (-3) / 2 = -1   (not -2, which would be floor division)
//   (-5) / 3 = -1   (not -2)
//
// This is symmetric: +3/2 = 1, -3/2 = -1.  No systematic downward bias.
// The test confirms MA output matches this expectation.
//
// Low-pass integer path: "half-up rounding" via (x + (1<<(FRAC-1))) >> FRAC.
// For negative x: (x + 32768) >> 16.  We verify this doesn't introduce bias.
#include <gtest/gtest.h>
#include <cstdint>
#include "siglib/moving_average.hpp"
#include "siglib/low_pass.hpp"
#include "siglib/sample.hpp"

// ── Moving average rounding ───────────────────────────────────────────────────
TEST(Rounding, MANegativeSumTruncatesTowardZero) {
    // Window of 2, values [-3, 0]: sum=-3, mean=-3/2=-1 (truncate, not -2)
    siglib::MovingAverage<int32_t> ma(2);
    ma.update({1'000'000LL, -3});
    auto out = ma.update({2'000'000LL,  0});
    EXPECT_EQ(out, -1) << "Expected truncation toward zero, got floor";
}

TEST(Rounding, MAPositiveAndNegativeSymmetric) {
    // +3/2 = 1,  -3/2 = -1: same magnitude
    siglib::MovingAverage<int32_t> ma_pos(2);
    ma_pos.update({1'000'000LL,  3});
    auto pos_out = ma_pos.update({2'000'000LL, 0});

    siglib::MovingAverage<int32_t> ma_neg(2);
    ma_neg.update({1'000'000LL, -3});
    auto neg_out = ma_neg.update({2'000'000LL,  0});

    EXPECT_EQ(pos_out, -neg_out)
        << "Rounding is asymmetric: pos=" << pos_out << " neg=" << neg_out;
}

// ── Low-pass integer rounding ─────────────────────────────────────────────────
TEST(Rounding, LowPassFixedPointConvergesToInput) {
    // Feed a constant DC value; filter must eventually output that value
    // (not permanently stuck 1 LSB below due to floor rounding).
    siglib::LowPass<int32_t> lp(20.0);
    int64_t ts = 0;
    int32_t out = 0;
    for (int i = 0; i < 500; ++i) {
        ts += 5'000'000LL;
        out = lp.update({ts, 1000});
    }
    // After 500 × 5ms = 2.5 s at 20 Hz cutoff (τ ≈ 7.96ms), the filter
    // has had ~314 time constants to converge.  Must be exactly 1000.
    EXPECT_EQ(out, 1000) << "Fixed-point deadband: filter did not converge to DC input";
}

TEST(Rounding, LowPassFixedPointNegativeDC) {
    siglib::LowPass<int32_t> lp(20.0);
    int64_t ts = 0;
    int32_t out = 0;
    for (int i = 0; i < 500; ++i) {
        ts += 5'000'000LL;
        out = lp.update({ts, -1000});
    }
    EXPECT_EQ(out, -1000) << "Fixed-point floor bias: negative DC did not converge";
}
