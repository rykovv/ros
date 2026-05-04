#include <gtest/gtest.h>
#include <ros/apply.hpp>
#include "../test_registers.hpp"

using namespace test;
using namespace ros;
using namespace ros::literals;

class ApplyFieldTest : public ::testing::Test {
protected:
    void SetUp() override { reset_bus(); }
};

// --- Compile-time writes ---

TEST_F(ApplyFieldTest, SingleFieldWrite_CT) {
    constexpr simple_reg r{};
    bus_read_value = 0xF0; // existing value for RMW

    apply(r.low_nibble = 5_f);

    // partial write -> reads first, then writes
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(bus_log[0].address, 0x10u);
    EXPECT_EQ(bus_log[1].op, bus_event::type::write);
    EXPECT_EQ(bus_log[1].address, 0x10u);
    EXPECT_EQ(bus_log[1].value, 0xF5u); // 0xF0 & ~0x0F | (5 << 0) = 0xF5
}

TEST_F(ApplyFieldTest, MultipleFieldWrite_CT) {
    constexpr simple_reg r{};
    bus_read_value = 0x00;

    apply(r.low_nibble = 5_f, r.high_nibble = 0xA_f);

    // full register write (both fields cover all bits) -> no read needed
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].value, 0xA5u);
}

TEST_F(ApplyFieldTest, FullWidthWrite_CT_NoRMW) {
    constexpr full_reg r{};

    apply(r.value = 0xDEAD'BEEF_f);

    // full-width field covers all bits -> no read needed
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].value, 0xDEADBEEFu);
}

// --- Runtime writes ---

TEST_F(ApplyFieldTest, SingleFieldWrite_RT) {
    constexpr simple_reg r{};
    bus_read_value = 0xA0;
    uint8_t val = 3;

    apply(r.low_nibble = val);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xA3u);
}

TEST_F(ApplyFieldTest, RuntimeOverflow_ClampedWrite) {
    constexpr simple_reg r{};
    bus_read_value = 0x00;
    uint8_t val = 0xFF; // exceeds [3:0], gets clamped

    apply(r.low_nibble = val);

    ASSERT_EQ(bus_log.size(), 2u);
    using low = decltype(simple_reg::low_nibble);
    uint8_t clamped = (1 << low::length) - 1;
    EXPECT_EQ(bus_log[1].value, static_cast<uint32_t>(clamped));
}

// --- Reads ---

TEST_F(ApplyFieldTest, SingleFieldRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xA5;

    auto [low_val] = apply(r.low_nibble.read());

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(low_val, 0x05u);
}

TEST_F(ApplyFieldTest, MultipleFieldRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xA5;

    auto [low_val, high_val] = apply(r.low_nibble.read(), r.high_nibble.read());

    // single bus read, two field extractions
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(low_val, 0x05u);
    EXPECT_EQ(high_val, 0x0Au);
}

// --- Mixed read + write ---

TEST_F(ApplyFieldTest, WriteAndRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xA0;

    auto [high_val] = apply(r.low_nibble = 7_f, r.high_nibble.read());

    // partial write triggers RMW read, then write
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xA7u);
    EXPECT_EQ(high_val, 0x0Au);
}

// --- Invocable writes ---

TEST_F(ApplyFieldTest, InvocableWrite_SingleField) {
    constexpr simple_reg r{};
    bus_read_value = 0x03; // low_nibble currently = 3

    apply(r.low_nibble([](uint8_t current) -> uint8_t {
        return current + 1;
    }));

    ASSERT_EQ(bus_log.size(), 2u); // read + write
    // invocable reads current (3), returns 4, written back
    EXPECT_EQ(bus_log[1].value, 0x04u);
}

// --- Single bit operations ---

TEST_F(ApplyFieldTest, SingleBit_Set) {
    constexpr bit_reg r{};
    bus_read_value = 0x00;

    apply(r.enable = 1_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0x01u);
}

TEST_F(ApplyFieldTest, SingleBit_Clear) {
    constexpr bit_reg r{};
    bus_read_value = 0xFF;

    apply(r.enable = 0_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xFEu);
}
