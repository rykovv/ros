#include "../test_registers.hpp"

#include <gtest/gtest.h>

#include <ros/eval.hpp>

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

    eval(r.low_nibble = 5_f);

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

    eval(r.low_nibble = 5_f, r.high_nibble = 0xA_f);

    // full register write (both fields cover all bits) -> no read needed
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].value, 0xA5u);
}

TEST_F(ApplyFieldTest, FullWidthWrite_CT_NoRMW) {
    constexpr full_reg r{};

    eval(r.value = 0xDEAD'BEEF_f);

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

    eval(r.low_nibble = val);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xA3u);
}

TEST_F(ApplyFieldTest, RuntimeOverflow_ClampedWrite) {
    constexpr simple_reg r{};
    bus_read_value = 0x00;
    uint8_t val = 0xFF; // exceeds [3:0], gets clamped

    eval(r.low_nibble = val);

    ASSERT_EQ(bus_log.size(), 2u);
    using low = decltype(simple_reg::low_nibble);
    uint8_t clamped = (1u << low::length) - 1;
    EXPECT_EQ(bus_log[1].value, static_cast<uint32_t>(clamped));
}

TEST_F(ApplyFieldTest, RuntimeOverflow_ClampedWrite_NonZeroLsb) {
    constexpr simple_reg r{};
    bus_read_value = 0x00;
    uint8_t val = 0x1F; // exceeds high_nibble [7:4] max (0xF), gets clamped

    eval(r.high_nibble = val);

    ASSERT_EQ(bus_log.size(), 2u);
    using high = decltype(simple_reg::high_nibble);
    // a fully-saturated field equals its (register-positioned) mask: 0xF << 4.
    // the clamp must be computed in field space then positioned, so the low
    // bits of the field value must survive the shift back to [7:4].
    EXPECT_EQ(bus_log[1].value, static_cast<uint32_t>(high::mask));
}

// --- Reads ---

TEST_F(ApplyFieldTest, SingleFieldRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xA5;

    auto low_val = eval(r.low_nibble);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(low_val, 0x05u);
}

TEST_F(ApplyFieldTest, MultipleFieldRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xA5;

    auto [low_val, high_val] = eval(r.low_nibble, r.high_nibble);

    // single bus read, two field extractions
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(low_val, 0x05u);
    EXPECT_EQ(high_val, 0x0Au);
}

// --- Mixed read + write ---

TEST_F(ApplyFieldTest, WriteAndRead) {
    constexpr simple_reg r{};
    bus_read_value = 0xA0;

    auto high_val = eval(r.low_nibble = 7_f, r.high_nibble);

    // partial write triggers RMW read, then write
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xA7u);
    EXPECT_EQ(high_val, 0x0Au);
}

// --- Invocable writes ---

TEST_F(ApplyFieldTest, InvocableWrite_SingleField) {
    constexpr simple_reg r{};
    bus_read_value = 0x03; // low_nibble currently = 3

    eval(r.low_nibble([](uint8_t current) -> uint8_t { return current + 1; }));

    ASSERT_EQ(bus_log.size(), 2u); // read + write
    // invocable reads current (3), returns 4, written back
    EXPECT_EQ(bus_log[1].value, 0x04u);
}

TEST_F(ApplyFieldTest, InvocableWrite_PartialWrite_PreservesOtherFields) {
    constexpr simple_reg r{};
    bus_read_value = 0xA3; // low_nibble = 3, high_nibble = A

    eval(r.low_nibble([](uint8_t current) -> uint8_t { return current + 1; }));

    ASSERT_EQ(bus_log.size(), 2u);
    // partial write: read 0xA3, invocable returns 3+1=4, high_nibble preserved
    EXPECT_EQ(bus_log[1].value, 0xA4u);
}

TEST_F(ApplyFieldTest, InvocableWrite_PlusCTWrite_CoversAllRMW) {
    constexpr simple_reg r{};
    bus_read_value = 0xA3; // low_nibble = 3, high_nibble = A

    eval(r.low_nibble([](uint8_t current) -> uint8_t {
        return current + 1; // 3 + 1 = 4
    }),
         r.high_nibble = 0xB_f);

    ASSERT_EQ(bus_log.size(), 2u); // read + write
    // invocable should read low_nibble as 3, return 4
    // CT write sets high_nibble to B
    EXPECT_EQ(bus_log[1].value, 0xB4u);
}

TEST_F(ApplyFieldTest,
       InvocableWrite_MultipleInvocables_LaterSeesEarlierWrite) {
    constexpr simple_reg r{};
    bus_read_value = 0x21; // low_nibble = 1, high_nibble = 2

    // Invocables are applied as a sequential chain in argument order, NOT
    // against a single pre-read snapshot. The 2nd invocable (writing
    // low_nibble) reads high_nibble, which the 1st invocable has already
    // incremented.
    eval(r.high_nibble(
             [](uint8_t h) -> uint8_t { return h + 1; }), // high: 2 -> 3
         r.low_nibble(
             [](uint8_t h) -> uint8_t { return h; }, // low = current high
             r.high_nibble));

    ASSERT_EQ(bus_log.size(), 2u);
    // low reads high AFTER it became 3 -> 0x33 (a snapshot semantics would give
    // 0x32)
    EXPECT_EQ(bus_log[1].value, 0x33u);
}

TEST_F(ApplyFieldTest, InvocableWrite_MultipleInvocables_OrderDependent) {
    constexpr simple_reg r{};
    bus_read_value = 0x21; // low_nibble = 1, high_nibble = 2

    // Same two invocables, reversed order: low_nibble's invocable now runs
    // first, so it reads high_nibble's original value (2) before it is
    // incremented.
    eval(r.low_nibble([](uint8_t h) -> uint8_t { return h; }, r.high_nibble),
         r.high_nibble([](uint8_t h) -> uint8_t { return h + 1; }));

    ASSERT_EQ(bus_log.size(), 2u);
    // low reads high BEFORE the increment -> low=2; high then becomes 3 -> 0x32
    EXPECT_EQ(bus_log[1].value, 0x32u);
}

TEST_F(ApplyFieldTest, InvocableWrite_FullWidthField) {
    constexpr full_reg r{};
    bus_read_value = 0xDEADBEEF;

    eval(r.value([](uint32_t current) -> uint32_t {
        return current ^ 0xFF; // toggle low byte
    }));

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xDEADBE10u);
}

TEST_F(ApplyFieldTest, InvocableWrite_SpecialFieldsPreserveIdentity) {
    constexpr rw_w1c_reg r{};
    bus_read_value = 0xF3; // data = 3, status(RW_1C) = F

    eval(r.data([](uint8_t current) -> uint8_t {
        return current + 1; // 3 + 1 = 4
    }),
         r.status = 0_f);

    ASSERT_EQ(bus_log.size(), 2u);
    // data should be 4 (from invocable), status identity for RW_1C is 0
    EXPECT_EQ(bus_log[1].value, 0x04u);
}

TEST_F(ApplyFieldTest, InvocableWrite_UnwrittenSpecialFieldGetsIdentity) {
    constexpr rw_w1c_reg r{};
    bus_read_value = 0xF3; // data = 3, status(RW_1C) = F

    // only write data via invocable, status is NOT written
    eval(r.data([](uint8_t current) -> uint8_t {
        return current + 1; // 3 + 1 = 4
    }));

    ASSERT_EQ(bus_log.size(), 2u);
    // data = 4, status should get identity (0 for RW_1C), NOT read value (0xF)
    // writing 0xF to RW_1C would accidentally clear those bits in hardware
    EXPECT_EQ(bus_log[1].value, 0x04u);
}

// --- Single bit operations ---

TEST_F(ApplyFieldTest, SingleBit_Set) {
    constexpr bit_reg r{};
    bus_read_value = 0x00;

    eval(r.enable = 1_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0x01u);
}

TEST_F(ApplyFieldTest, SingleBit_Clear) {
    constexpr bit_reg r{};
    bus_read_value = 0xFF;

    eval(r.enable = 0_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value, 0xFEu);
}

// --- Enum assignment ---

TEST_F(ApplyFieldTest, EnumAssignment) {
    constexpr enum_reg r{};
    bus_read_value = 0x00;

    eval(r.mode_field = enum_reg::mode::HIGH);

    ASSERT_EQ(bus_log.size(), 2u); // partial write → RMW
    EXPECT_EQ(bus_log[1].value, static_cast<uint32_t>(enum_reg::mode::HIGH));
}

TEST_F(ApplyFieldTest, EnumAssignment_PreservesOtherFields) {
    constexpr enum_reg r{};
    bus_read_value = 0xFC; // data field = 0x3F

    eval(r.mode_field = enum_reg::mode::TURBO);

    ASSERT_EQ(bus_log.size(), 2u);
    // mode_field [1:0] = 3, data [7:2] preserved = 0xFC
    EXPECT_EQ(bus_log[1].value, 0xFFu);
}

// --- Unsafe field handler ---

TEST_F(ApplyFieldTest, UnsafeFieldWrite) {
    constexpr simple_reg r{};
    bus_read_value = 0x00;

    // unsafe bypasses range check
    eval(r.low_nibble.unsafe = uint8_t{0xFF});

    ASSERT_EQ(bus_log.size(), 2u);
    // unsafe directly casts, so 0xFF goes in as field value
    // to_reg(0x00, 0xFF) = (0 & ~0x0F) | (0xFF << 0) & 0x0F = 0x0F
    EXPECT_EQ(bus_log[1].value & 0x0Fu, 0x0Fu);
}

// --- Bus call ordering ---

TEST_F(ApplyFieldTest, PartialWrite_ReadsBeforeWrites) {
    constexpr simple_reg r{};
    bus_read_value = 0xA0;

    eval(r.low_nibble = 5_f);

    ASSERT_GE(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(bus_log[1].op, bus_event::type::write);
}

TEST_F(ApplyFieldTest, FullWrite_NoRead) {
    constexpr simple_reg r{};

    eval(r.low_nibble = 5_f, r.high_nibble = 0xA_f);

    // all bits covered → no RMW read
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
}
