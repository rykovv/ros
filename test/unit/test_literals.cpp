#include <gtest/gtest.h>

#include <ros/literals.hpp>

using namespace ros::literals;

TEST(Literals, DecimalFieldValue) {
    constexpr auto v = 42_f;
    EXPECT_EQ(v.value, 42u);
}

TEST(Literals, DecimalRegisterValue) {
    constexpr auto v = 255_r;
    EXPECT_EQ(v.value, 255u);
}

TEST(Literals, HexFieldValue) {
    constexpr auto v = 0xFF_f;
    EXPECT_EQ(v.value, 0xFFu);
}

TEST(Literals, HexRegisterValue) {
    constexpr auto v = 0xDEAD'BEEF_r;
    EXPECT_EQ(v.value, 0xDEADBEEFu);
}

TEST(Literals, MsbLsb) {
    constexpr auto m = 7_msb;
    constexpr auto l = 0_lsb;
    EXPECT_EQ(m.value, 7u);
    EXPECT_EQ(l.value, 0u);
}

TEST(Literals, Address) {
    constexpr auto a = 0x4000'0000_addr;
    EXPECT_EQ(a.value, 0x40000000u);
}

TEST(Literals, DecimalWithDelimiter) {
    constexpr auto v = 1'000_f;
    EXPECT_EQ(v.value, 1000u);
}

TEST(Literals, ZeroValues) {
    constexpr auto fv = 0_f;
    constexpr auto rv = 0_r;
    EXPECT_EQ(fv.value, 0u);
    EXPECT_EQ(rv.value, 0u);
}
