#include <gtest/gtest.h>
#include "../test_registers.hpp"

using namespace test;
using namespace ros;
using namespace ros::literals;

TEST(RegLayout, AllRW) {
    // Both nibbles are RW: layout = 0x0F | 0xF0 = 0xFF
    EXPECT_EQ(simple_reg::layout, 0xFF);
}

TEST(RegLayout, MixedAccess) {
    // get_rmw_mask includes fields with R bit set:
    // status [3:0] RO (has R) = 0x000F
    // config [7:4] RW (has R) = 0x00F0
    // command [11:8] WO (no R) = excluded
    // flags [15:12] RW (has R) = 0xF000
    uint16_t expected = 0x000F | 0x00F0 | 0xF000;
    EXPECT_EQ(mixed_reg::layout, expected);
}

TEST(RegLayout, AllRO) {
    // byte0 [7:0] RO + byte1 [15:8] RO
    uint32_t expected = 0x0000FF | 0x00FF00;
    EXPECT_EQ(ro_reg::layout, expected);
}

TEST(RegLayout, AllWO) {
    // WO fields have no R bit, so layout = 0
    EXPECT_EQ(wo_reg::layout, 0u);
}

TEST(RegLayout, FullWidth) {
    EXPECT_EQ(full_reg::layout, 0xFFFFFFFF);
}

TEST(RegProperties, HasWoField) {
    EXPECT_FALSE(simple_reg::has_wo_field);
    EXPECT_TRUE(mixed_reg::has_wo_field);
    EXPECT_TRUE(wo_reg::has_wo_field);
    EXPECT_FALSE(ro_reg::has_wo_field);
}

TEST(RegProperties, HasRoField) {
    EXPECT_FALSE(simple_reg::has_ro_field);
    EXPECT_TRUE(mixed_reg::has_ro_field);
    EXPECT_TRUE(ro_reg::has_ro_field);
    EXPECT_FALSE(wo_reg::has_ro_field);
}

TEST(RegAddress, Values) {
    EXPECT_EQ(simple_reg::address::value, 0x10u);
    EXPECT_EQ(mixed_reg::address::value, 0x20u);
    EXPECT_EQ(ro_reg::address::value, 0x30u);
    EXPECT_EQ(wo_reg::address::value, 0x40u);
}
