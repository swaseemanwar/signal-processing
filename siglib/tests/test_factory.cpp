// test_factory.cpp
//
// Requirement: "third filter added with NEW files only, zero edits to existing
// files — use self-registering factory pattern"
//
// This test includes median.hpp (the "new file").  By doing so, the
// file-scope Registrar statics run and register "median_i32" / "median_f32"
// in the factory.  The test then creates instances via the factory and
// verifies they produce correct output.
//
// Key point: filter.hpp, moving_average.hpp, low_pass.hpp, and
// CMakeLists.txt are NOT modified by the addition of Median.
#include <gtest/gtest.h>
#include <cstdint>
#include <memory>
#include "siglib/filter.hpp"
#include "siglib/median.hpp"    // ← the only new include
#include "siglib/sample.hpp"

TEST(Factory, MedianI32RegistersAndWorks) {
    auto f = siglib::FilterFactory<int32_t>::create("median_i32");
    ASSERT_NE(f, nullptr);

    // Feed [5, 1, 3]: sorted = [1, 3, 5], median = 3
    f->update({1'000'000LL, 5});
    f->update({2'000'000LL, 1});
    int32_t out = f->update({3'000'000LL, 3});
    EXPECT_EQ(out, 3);
}

TEST(Factory, MedianF32RegistersAndWorks) {
    auto f = siglib::FilterFactory<float>::create("median_f32");
    ASSERT_NE(f, nullptr);

    f->update({1'000'000LL, 10.0f});
    f->update({2'000'000LL,  2.0f});
    float out = f->update({3'000'000LL,  6.0f});
    EXPECT_NEAR(out, 6.0f, 1e-5f);
}

TEST(Factory, UnknownFilterThrows) {
    EXPECT_THROW(
        siglib::FilterFactory<float>::create("nonexistent"),
        std::runtime_error
    );
}

TEST(Factory, MedianRejectsOutlier) {
    // Median property: single outlier has zero effect on output
    auto f = siglib::FilterFactory<float>::create("median_f32");
    // Window of 5: [1, 1, 1, 1, 1000] → median = 1
    f->update({1'000'000LL,    1.0f});
    f->update({2'000'000LL,    1.0f});
    f->update({3'000'000LL,    1.0f});
    f->update({4'000'000LL,    1.0f});
    float out = f->update({5'000'000LL, 1000.0f});
    EXPECT_NEAR(out, 1.0f, 0.01f)
        << "Median did not reject outlier: " << out;
}

TEST(Factory, MedianReset) {
    auto f = siglib::FilterFactory<int32_t>::create("median_i32");
    f->update({1'000'000LL, 100});
    f->update({2'000'000LL, 200});
    f->reset();
    // After reset, window is empty; first sample returns itself
    int32_t out = f->update({3'000'000LL, 50});
    EXPECT_EQ(out, 50);
}
