#include <gtest/gtest.h>
#include "../test_registers.hpp"

using namespace test;
using namespace ros;
using namespace ros::literals;

TEST(FieldMask, LowNibble) {
    // bits [3:0] -> mask = 0x0F
    using low = decltype(simple_reg::low_nibble);
    EXPECT_EQ(low::mask, 0x0F);
}

TEST(FieldMask, HighNibble) {
    // bits [7:4] -> mask = 0xF0
    using high = decltype(simple_reg::high_nibble);
    EXPECT_EQ(high::mask, 0xF0);
}

TEST(FieldMask, SingleBit) {
    using en = decltype(bit_reg::enable);
    EXPECT_EQ(en::mask, 0x01);

    using rst = decltype(bit_reg::reset);
    EXPECT_EQ(rst::mask, 0x02);
}

TEST(FieldMask, FullWidth) {
    using val = decltype(full_reg::value);
    EXPECT_EQ(val::mask, 0xFFFFFFFF);
}

TEST(FieldMask, WideField) {
    // bits [7:2] -> mask = 0b1111'1100 = 0xFC
    using d = decltype(bit_reg::data);
    EXPECT_EQ(d::mask, 0xFC);
}

TEST(FieldLength, MultiBit) {
    using low = decltype(simple_reg::low_nibble);
    EXPECT_EQ(low::length, 4); // msb - lsb + 1 = 3 - 0 + 1
}

TEST(FieldLength, SingleBit) {
    using en = decltype(bit_reg::enable);
    EXPECT_EQ(en::length, 1);
}

TEST(FieldToReg, LowNibble) {
    using low = decltype(simple_reg::low_nibble);
    // value 5 placed into [3:0], existing reg = 0xF0
    EXPECT_EQ(low::to_reg(0xF0, 0x05), 0xF5);
}

TEST(FieldToReg, HighNibble) {
    using high = decltype(simple_reg::high_nibble);
    // value 0xA placed into [7:4], existing reg = 0x03
    EXPECT_EQ(high::to_reg(0x03, 0x0A), 0xA3);
}

TEST(FieldToReg, SingleBit_Set) {
    using en = decltype(bit_reg::enable);
    EXPECT_EQ(en::to_reg(0x00, 1), 0x01);
}

TEST(FieldToReg, SingleBit_Clear) {
    using en = decltype(bit_reg::enable);
    EXPECT_EQ(en::to_reg(0xFF, 0), 0xFE);
}

TEST(FieldToField, LowNibble) {
    using low = decltype(simple_reg::low_nibble);
    EXPECT_EQ(low::to_field(0xA5), 0x05);
}

TEST(FieldToField, HighNibble) {
    using high = decltype(simple_reg::high_nibble);
    EXPECT_EQ(high::to_field(0xA5), 0x0A);
}

TEST(FieldToField, SingleBit) {
    using en = decltype(bit_reg::enable);
    EXPECT_EQ(en::to_field(0b1111'1110), 0);
    EXPECT_EQ(en::to_field(0b0000'0001), 1);
}

TEST(FieldRuntimeCheck, ValidValue) {
    using low = decltype(simple_reg::low_nibble);
    EXPECT_EQ(low::runtime_check(0x07), 0x07);
}

TEST(FieldRuntimeCheck, OverflowClamps) {
    using low = decltype(simple_reg::low_nibble);
    // 0xFF exceeds mask >> lsb = 0x0F, clamp returns (1 << length) - 1
    auto clamped = low::runtime_check(0xFF);
    EXPECT_EQ(clamped, (1 << low::length) - 1);
}

TEST(FieldRuntimeCheck, BoundaryValue) {
    using low = decltype(simple_reg::low_nibble);
    // mask >> lsb = 0x0F, so 0x0F is the max valid
    EXPECT_EQ(low::runtime_check(0x0F), 0x0F);
}

TEST(FieldAccessType, ReadOnly) {
    using status = decltype(mixed_reg::status);
    EXPECT_EQ(static_cast<uint8_t>(status::access), static_cast<uint8_t>(access_type::RO));
}

TEST(FieldAccessType, WriteOnly) {
    using cmd = decltype(mixed_reg::command);
    EXPECT_EQ(static_cast<uint8_t>(cmd::access), static_cast<uint8_t>(access_type::WO));
}

TEST(FieldAccessType, ReadWrite) {
    using cfg = decltype(mixed_reg::config);
    EXPECT_EQ(static_cast<uint8_t>(cfg::access), static_cast<uint8_t>(access_type::RW));
}
