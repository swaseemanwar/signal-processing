// test_overflow.cpp
//
// Requirement: int64_t accumulator — "prove it with bit-budget arithmetic"
//
// Strategy: feed N copies of INT32_MAX into MovingAverage<int32_t>.
//   - int32_t accumulator: INT32_MAX + INT32_MAX = 0x7fffffff + 0x7fffffff
//     = 0xfffffffe, which as a signed 32-bit is -2 → WRONG output.
//   - int64_t accumulator: 2 × 2,147,483,647 = 4,294,967,294 → no overflow.
//
// The output of MA(5 × INT32_MAX) should be exactly INT32_MAX.
#include <gtest/gtest.h>
#include <cstdint>
#include <limits>
#include "siglib/moving_average.hpp"
#include "siglib/sample.hpp"

TEST(Overflow, AccumulatorDoesNotWrapForInt32Max) {
    siglib::MovingAverage<int32_t> ma(5);

    const int32_t val = std::numeric_limits<int32_t>::max();
    int64_t ts = 0;
    int32_t out = 0;

    for (int i = 0; i < 10; ++i) {
        ts += 5'000'000;  // 5 ms in nanoseconds
        out = ma.update({ts, val});
    }

    // After 5+ samples of INT32_MAX, output must be exactly INT32_MAX.
    // If accumulator overflowed to int32_t, output would be negative or wrong.
    EXPECT_EQ(out, val) << "Overflow in accumulator: int64_t required";
}

TEST(Overflow, AccumulatorDoesNotWrapForInt32Min) {
    siglib::MovingAverage<int32_t> ma(5);

    const int32_t val = std::numeric_limits<int32_t>::min();
    int64_t ts = 0;
    int32_t out = 0;

    for (int i = 0; i < 10; ++i) {
        ts += 5'000'000;
        out = ma.update({ts, val});
    }

    EXPECT_EQ(out, val) << "Underflow in accumulator: int64_t required";
}

TEST(Overflow, MixedLargeValues) {
    // Prove int64_t accumulator is required for mixed large-value streams.
    //
    // Fill a window of 3 with [P, P, -1]:
    //   Correct sum (int64_t): 2×2147483647 − 1 = 4294967293
    //   Output = 4294967293 / 3 = 1431655764   ← fits in int32_t, positive
    //
    //   Broken sum (int32_t accumulator):
    //     P+P overflows → −2 as signed int32. Then −2−1=−3.
    //     Output = −3 / 3 = −1  ← wrong sign, proves overflow
    //
    // So EXPECT_GT(out, 0) catches the int32 overflow case.
    const int32_t P = std::numeric_limits<int32_t>::max();
    siglib::MovingAverage<int32_t> ma(3);

    int64_t ts = 0;
    ts += 5'000'000; ma.update({ts, P});
    ts += 5'000'000; ma.update({ts, P});
    ts += 5'000'000; int32_t out = ma.update({ts, -1});

    EXPECT_EQ(out, 1431655764)
        << "Wrong output: if negative, accumulator overflowed to int32";
    EXPECT_GT(out, 0) << "Sign error suggests accumulator overflow";
}
