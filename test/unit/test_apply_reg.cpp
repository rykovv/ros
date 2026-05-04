#include <gtest/gtest.h>
#include <ros/apply.hpp>
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

    apply(r = 0xAB_r);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].address, 0x10u);
    EXPECT_EQ(bus_log[0].value, 0xABu);
}

TEST_F(ApplyRegTest, MultipleRegWrite_CT) {
    constexpr simple_reg r1{};
    constexpr full_reg r2{};

    apply(r1 = 0x55_r, r2 = 0x1234'5678_r);

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

    apply(r = val);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].value, 0x77u);
}

TEST_F(ApplyRegTest, RuntimeWrite_MaskedByHandler) {
    constexpr mixed_reg r{};
    // layout excludes WO command bits [11:8]
    // writing 0xFFFF triggers mask handler -> val & layout
    uint16_t val = 0xFFFF;

    apply(r = val);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].value, static_cast<uint32_t>(0xFFFF & mixed_reg::layout));
}

// --- Register reads ---

TEST_F(ApplyRegTest, SingleRegRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xCD;

    auto [val] = apply(r.read());

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(bus_log[0].address, 0x10u);
    EXPECT_EQ(val, 0xCDu);
}

TEST_F(ApplyRegTest, MultipleRegRead) {
    constexpr simple_reg r1{};
    constexpr full_reg r2{};
    bus_read_value = 0x42;

    auto [v1, v2] = apply(r1.read(), r2.read());

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(v1, 0x42u);
    EXPECT_EQ(v2, 0x42u);
}

// --- Invocable register writes ---

TEST_F(ApplyRegTest, InvocableWrite_Self) {
    constexpr simple_reg r{};
    bus_read_value = 0x0F;

    apply(r([](uint8_t current) -> uint8_t {
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

    auto [v1] = apply(r1.read(), r2 = 0xBEEF_r);

    // read r1 + write r2
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(v1, 0xAAu);
    EXPECT_EQ(bus_log[1].address, 0x60u);
    EXPECT_EQ(bus_log[1].value, 0xBEEFu);
}
