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
    static_assert(simple_reg::writable_mask == 0xFF);
    static_assert(full_reg::writable_mask == 0xFFFFFFFF);
}

TEST(Constexpr, RegProperties) {
    static_assert(simple_reg::physically_readable);
    static_assert(!wo_reg::physically_readable);
    static_assert(simple_reg::ro_mask == 0);
    static_assert(ro_reg::ro_mask == 0xFFFF);
    static_assert(simple_reg::identity == 0);
}

TEST(Constexpr, FieldAccessHelpers) {
    using rw = decltype(simple_reg::low_nibble);
    static_assert(rw::writable());
    static_assert(rw::readable());
    static_assert(rw::readwritable());

    using ro = decltype(mixed_reg::status);
    static_assert(!ro::writable());
    static_assert(ro::readable());
    static_assert(ro::readonly());

    using wo = decltype(mixed_reg::command);
    static_assert(wo::writable());
    static_assert(!wo::readable());
    static_assert(wo::writeonly());
}

TEST(Constexpr, FieldSubMasks) {
    using rw = decltype(simple_reg::low_nibble);
    static_assert(rw::rmw_mask == rw::mask);
    static_assert(rw::write_mask == rw::mask);
    static_assert(rw::read_mask == rw::mask);

    using ro = decltype(mixed_reg::status);
    static_assert(ro::rmw_mask == 0);
    static_assert(ro::write_mask == 0);
    static_assert(ro::read_mask == ro::mask);
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
