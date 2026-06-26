#include "../test_registers.hpp"

#include <gtest/gtest.h>

using namespace test;
using namespace ros;
using namespace ros::literals;

// --- Layout (now: all field masks OR'd) ---

TEST(RegLayout, AllRW) { EXPECT_EQ(simple_reg::writable_mask, 0xFF); }

TEST(RegLayout, MixedAccess) {
    // All field masks OR'd: status|config|command|flags =
    // 0x000F|0x00F0|0x0F00|0xF000
    EXPECT_EQ(mixed_reg::writable_mask, 0xFFF0);
}

TEST(RegLayout, AllRO) { EXPECT_EQ(ro_reg::writable_mask, 0x0); }

TEST(RegLayout, AllWO) {
    // WO fields now contribute to layout: 0xFF | 0xFF00
    EXPECT_EQ(wo_reg::writable_mask, 0xFFFFu);
}

TEST(RegLayout, FullWidth) { EXPECT_EQ(full_reg::writable_mask, 0xFFFFFFFF); }

// --- ro_mask (only RO fields contribute) ---

TEST(RegRoMask, AllRW) {
    // No RO fields
    EXPECT_EQ(simple_reg::ro_mask, 0u);
}

TEST(RegRoMask, MixedAccess) {
    // Only status [3:0] is RO
    EXPECT_EQ(mixed_reg::ro_mask, 0x000Fu);
}

TEST(RegRoMask, AllRO) { EXPECT_EQ(ro_reg::ro_mask, 0xFFFFu); }

TEST(RegRoMask, AllWO) { EXPECT_EQ(wo_reg::ro_mask, 0u); }

// --- physically_readable ---

TEST(RegPhysicallyReadable, AllRW) {
    EXPECT_TRUE(simple_reg::physically_readable);
}

TEST(RegPhysicallyReadable, AllRO) { EXPECT_TRUE(ro_reg::physically_readable); }

TEST(RegPhysicallyReadable, AllWO) {
    EXPECT_FALSE(wo_reg::physically_readable);
}

TEST(RegPhysicallyReadable, Mixed) {
    // mixed_reg has a WO field → not physically readable
    EXPECT_FALSE(mixed_reg::physically_readable);
}

// --- identity mask ---

TEST(RegIdentity, AllRW) {
    // RW fields have identity = 0
    EXPECT_EQ(simple_reg::identity, 0u);
}

TEST(RegIdentity, SpecialAccessTypes) {
    // RW_0C and RW_0S have identity = mask (writing all-ones is identity)
    // Others (RW, RW_1C, RW_1S, RW_1T, RC, RS, RW_O) have identity = 0
    using clear_w0 = decltype(special_reg::clear_on_w0);
    using set_w0 = decltype(special_reg::set_on_w0);
    uint32_t expected = clear_w0::mask | set_w0::mask;
    EXPECT_EQ(special_reg::identity, expected);
}

// --- rmw_mask (only plain RW fields contribute) ---

TEST(RegRmwMask, AllRW) {
    // Both fields are RW → rmw_mask covers all bits
    EXPECT_EQ(simple_reg::rmw_mask, 0xFFu);
}

TEST(RegRmwMask, MixedAccess) {
    // Only config [7:4] and flags [15:12] are RW
    EXPECT_EQ(mixed_reg::rmw_mask, 0xF0F0u);
}

TEST(RegRmwMask, AllRO) { EXPECT_EQ(ro_reg::rmw_mask, 0u); }

TEST(RegRmwMask, AllWO) {
    // WO is not plain RW → rmw_mask = 0
    EXPECT_EQ(wo_reg::rmw_mask, 0u);
}

TEST(RegRmwMask, SpecialAccess) {
    // special_reg has no plain RW fields
    EXPECT_EQ(special_reg::rmw_mask, 0u);
}

TEST(RegRmwMask, RW_Plus_Special) {
    // rw_w1c_reg: data[3:0] is RW, status[7:4] is RW_1C
    EXPECT_EQ(rw_w1c_reg::rmw_mask, 0x0Fu);
}

// --- Address ---

TEST(RegAddress, Values) {
    EXPECT_EQ(simple_reg::address::value, 0x10u);
    EXPECT_EQ(mixed_reg::address::value, 0x20u);
    EXPECT_EQ(ro_reg::address::value, 0x30u);
    EXPECT_EQ(wo_reg::address::value, 0x40u);
    EXPECT_EQ(special_reg::address::value, 0x70u);
}
