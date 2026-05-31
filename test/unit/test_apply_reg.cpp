#include <gtest/gtest.h>
#include <ros/eval.hpp>
#include "../test_registers.hpp"

using namespace test;
using namespace ros;
using namespace ros::literals;

class ApplyRegTest : public ::testing::Test {
protected:
    void SetUp() override { reset_bus(); }
};

// --- Compile-time register writes ---

TEST_F(ApplyRegTest, SingleRegWrite_CT) {
    constexpr simple_reg r{};

    eval(r = 0xAB_r);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].address, 0x10u);
    EXPECT_EQ(bus_log[0].value, 0xABu);
}

TEST_F(ApplyRegTest, MultipleRegWrite_CT) {
    constexpr simple_reg r1{};
    constexpr full_reg r2{};

    eval(r1 = 0x55_r, r2 = 0x1234'5678_r);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[0].address, 0x10u);
    EXPECT_EQ(bus_log[0].value, 0x55u);
    EXPECT_EQ(bus_log[1].address, 0x60u);
    EXPECT_EQ(bus_log[1].value, 0x12345678u);
}

// --- Runtime register writes ---

TEST_F(ApplyRegTest, SingleRegWrite_RT) {
    constexpr simple_reg r{};
    uint8_t val = 0x77;

    eval(r = val);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].value, 0x77u);
}

TEST_F(ApplyRegTest, RuntimeWrite_MaskedByHandler) {
    constexpr mixed_reg r{};
    // ro_mask = status [3:0] = 0x000F
    // writing 0xFFFF sets RO bits -> triggers mask_handler -> val & layout
    uint16_t val = 0xFFFF;

    eval(r = val);

    ASSERT_EQ(bus_log.size(), 1u);
    // mask_handler returns static_cast<T>(v & layout), layout = 0xFFFF
    EXPECT_EQ(bus_log[0].value, static_cast<uint32_t>(0xFFFF & mixed_reg::layout));
}

TEST_F(ApplyRegTest, RuntimeWrite_NoRoBits_NoMasking) {
    constexpr simple_reg r{};
    // simple_reg has no RO fields (ro_mask = 0), so no masking occurs
    uint8_t val = 0xFF;

    eval(r = val);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].value, 0xFFu);
}

// --- Register reads ---

TEST_F(ApplyRegTest, SingleRegRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xCD;

    auto val = eval(r);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(bus_log[0].address, 0x10u);
    EXPECT_EQ(val, 0xCDu);
}

TEST_F(ApplyRegTest, MultipleRegRead) {
    constexpr simple_reg r1{};
    constexpr full_reg r2{};
    bus_read_value = 0x42;

    auto [v1, v2] = eval(r1, r2);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(v1, 0x42u);
    EXPECT_EQ(v2, 0x42u);
}

// --- Invocable register writes ---

TEST_F(ApplyRegTest, InvocableWrite_Self) {
    constexpr simple_reg r{};
    bus_read_value = 0x0F;

    eval(r([](uint8_t current) -> uint8_t {
        return current | 0xF0;
    }));

    // invocable reads current register, calls lambda, writes result
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(bus_log[1].op, bus_event::type::write);
    EXPECT_EQ(bus_log[1].value, 0xFFu);
}

// --- Mixed reads + writes ---

TEST_F(ApplyRegTest, ReadAndWrite) {
    constexpr simple_reg r1{};
    constexpr full_reg r2{};
    bus_read_value = 0xAA;

    auto v1 = eval(r1, r2 = 0xBEEF_r);

    // read r1 + write r2
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(v1, 0xAAu);
    EXPECT_EQ(bus_log[1].address, 0x60u);
    EXPECT_EQ(bus_log[1].value, 0xBEEFu);
}

// --- Unsafe register handler ---

TEST_F(ApplyRegTest, UnsafeRegWrite) {
    constexpr simple_reg r{};

    eval(r.unsafe = uint8_t{0xFF});

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].value, 0xFFu);
}

// --- Bus call ordering ---

TEST_F(ApplyRegTest, WritesBeforeReads_Ordering) {
    constexpr simple_reg r1{};
    constexpr full_reg r2{};
    bus_read_value = 0x42;

    auto v1 = eval(r1, r2 = 0xABCD_r);

    // reads first, then invocable writes, then CT writes, then RT writes evaluated last
    // r1 read should happen, then r2 write
    EXPECT_EQ(bus_log.back().op, bus_event::type::write);
    EXPECT_EQ(v1, 0x42u);
}
