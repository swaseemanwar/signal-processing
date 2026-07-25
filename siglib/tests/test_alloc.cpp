// test_alloc.cpp
//
// Requirement: "no dynamic memory allocation in steady-state (no new/malloc/
// push_back growth after init)"
//
// Method: override the global operator new to count calls.
// Reset the counter after construction (warmup), then run the filter in
// steady state and assert the counter stays at zero.
//
// Why this works:
//   - std::vector::push_back within reserved capacity does NOT call operator new.
//   - std::vector::erase within capacity does NOT call operator new (it shifts
//     elements in-place, never needing more memory).
//   - std::vector::reserve DOES call operator new — that's fine, it happens in
//     the constructor (warmup phase).
//
// Note: GoogleTest itself may allocate (for test bookkeeping). We reset the
// counter immediately before the steady-state loop and check only that.
#include <gtest/gtest.h>
#include <cstdlib>
#include <cstdint>
#include <new>
#include "siglib/moving_average.hpp"
#include "siglib/low_pass.hpp"
#include "siglib/sample.hpp"

// ── Global allocation counter ─────────────────────────────────────────────────
static int g_alloc_count = 0;
static bool g_tracking   = false;

void* operator new(std::size_t sz) {
    if (g_tracking) ++g_alloc_count;
    void* p = std::malloc(sz);
    if (!p) throw std::bad_alloc{};
    return p;
}

void* operator new[](std::size_t sz) {
    if (g_tracking) ++g_alloc_count;
    void* p = std::malloc(sz);
    if (!p) throw std::bad_alloc{};
    return p;
}

void operator delete(void* p) noexcept { std::free(p); }
void operator delete[](void* p) noexcept { std::free(p); }
void operator delete(void* p, std::size_t) noexcept { std::free(p); }
void operator delete[](void* p, std::size_t) noexcept { std::free(p); }

// ── Helper ────────────────────────────────────────────────────────────────────
struct AllocGuard {
    AllocGuard()  { g_alloc_count = 0; g_tracking = true; }
    ~AllocGuard() { g_tracking = false; }
    int count() const { return g_alloc_count; }
};

static constexpr int64_t DT = 5'000'000LL;  // 5 ms

// ── Tests ─────────────────────────────────────────────────────────────────────
TEST(Alloc, MovingAverageCountModeZeroAllocSteadyState) {
    // Construct (warmup — allocations allowed)
    siglib::MovingAverage<int32_t> ma(5);

    // Steady-state: push_back within reserved capacity, erase in-place
    AllocGuard g;
    int64_t ts = 0;
    for (int i = 0; i < 1000; ++i) {
        ts += DT;
        ma.update({ts, i});
    }
    EXPECT_EQ(g.count(), 0)
        << "MovingAverage (count mode) allocated " << g.count() << " times in steady state";
}

TEST(Alloc, MovingAverageTimeModeZeroAllocSteadyState) {
    siglib::MovingAverage<float> ma(10, 25'000'000LL);  // 25ms window

    AllocGuard g;
    int64_t ts = 0;
    for (int i = 0; i < 1000; ++i) {
        ts += DT;
        ma.update({ts, static_cast<float>(i)});
    }
    EXPECT_EQ(g.count(), 0)
        << "MovingAverage (time mode) allocated " << g.count() << " times in steady state";
}

TEST(Alloc, LowPassFloatZeroAllocSteadyState) {
    siglib::LowPass<float> lp(20.0);

    AllocGuard g;
    int64_t ts = 0;
    for (int i = 0; i < 1000; ++i) {
        ts += DT;
        lp.update({ts, 1.0f});
    }
    EXPECT_EQ(g.count(), 0)
        << "LowPass<float> allocated " << g.count() << " times in steady state";
}

TEST(Alloc, LowPassIntZeroAllocSteadyState) {
    siglib::LowPass<int32_t> lp(20.0);

    AllocGuard g;
    int64_t ts = 0;
    for (int i = 0; i < 1000; ++i) {
        ts += DT;
        lp.update({ts, 1000});
    }
    EXPECT_EQ(g.count(), 0)
        << "LowPass<int32_t> allocated " << g.count() << " times in steady state";
}
