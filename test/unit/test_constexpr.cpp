#include <gtest/gtest.h>
#include "../test_registers.hpp"

using namespace test;
using namespace ros;
using namespace ros::literals;

// These tests verify that key operations are truly constexpr-evaluable.
// If any static_assert fails, the test won't compile.

TEST(Constexpr, FieldMask) {
    using low = decltype(simple_reg::low_nibble);
    static_assert(low::mask == 0x0F);

    using high = decltype(simple_reg::high_nibble);
    static_assert(high::mask == 0xF0);
}

TEST(Constexpr, FieldLength) {
    using low = decltype(simple_reg::low_nibble);
    static_assert(low::length == 4);

    using en = decltype(bit_reg::enable);
    static_assert(en::length == 1);
}

TEST(Constexpr, FieldToReg) {
    using low = decltype(simple_reg::low_nibble);
    static_assert(low::to_reg(0xF0, 0x05) == 0xF5);

    using high = decltype(simple_reg::high_nibble);
    static_assert(high::to_reg(0x03, 0x0A) == 0xA3);
}

TEST(Constexpr, FieldToField) {
    using low = decltype(simple_reg::low_nibble);
    static_assert(low::to_field(0xA5) == 0x05);

    using high = decltype(simple_reg::high_nibble);
    static_assert(high::to_field(0xA5) == 0x0A);
}

TEST(Constexpr, RegLayout) {
    static_assert(simple_reg::layout == 0xFF);
    static_assert(full_reg::layout == 0xFFFFFFFF);
}

TEST(Constexpr, RegProperties) {
    static_assert(!simple_reg::has_wo_field);
    static_assert(!simple_reg::has_ro_field);
    static_assert(mixed_reg::has_wo_field);
    static_assert(mixed_reg::has_ro_field);
}

TEST(Constexpr, LiteralValues) {
    constexpr auto fv = 42_f;
    static_assert(fv.value == 42);
    constexpr auto rv = 0xFF_r;
    static_assert(rv.value == 255);
    constexpr auto m = 7_msb;
    static_assert(m.value == 7);
    constexpr auto l = 0_lsb;
    static_assert(l.value == 0);
    constexpr auto a = 0x1000_addr;
    static_assert(a.value == 0x1000);
}
