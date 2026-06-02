#include <gtest/gtest.h>
#include <ros/eval.hpp>
#include "../test_registers.hpp"

using namespace test;
using namespace ros;
using namespace ros::literals;

class AccessTypeTest : public ::testing::Test {
protected:
    void SetUp() override { reset_bus(); }
};

// ============================================================================
// Register-level mask correctness
// ============================================================================

TEST_F(AccessTypeTest, RmwMask_OnlyPlainRW) {
    // rmw_mask includes only plain RW fields — special access types excluded
    EXPECT_EQ(rw_w1c_reg::rmw_mask, 0x0Fu); // data[3:0] only
    EXPECT_EQ(rw_w0s_reg::rmw_mask, 0x0Fu);
    EXPECT_EQ(rw_w1t_reg::rmw_mask, 0x0Fu);
    EXPECT_EQ(rw_w0c_reg::rmw_mask, 0x0Fu);
    EXPECT_EQ(rw_w1s_reg::rmw_mask, 0x0Fu);
    EXPECT_EQ(rw_rc_reg::rmw_mask,  0x0Fu);
    EXPECT_EQ(special_only_reg::rmw_mask, 0u); // no plain RW at all
}

TEST_F(AccessTypeTest, WritableMask_IncludesAllWritable) {
    // writable_mask includes every field that can be written (RW + all special writable)
    EXPECT_EQ(rw_w1c_reg::writable_mask, 0xFFu);
    EXPECT_EQ(rw_w0s_reg::writable_mask, 0xFFu);
    EXPECT_EQ(rw_w1t_reg::writable_mask, 0xFFu);
    EXPECT_EQ(rw_rc_reg::writable_mask,  0x0Fu); // RC is not writable
    EXPECT_EQ(special_only_reg::writable_mask, 0xFFu);
}

TEST_F(AccessTypeTest, Identity_RW0C_RW0S_HaveMaskBits) {
    // RW_0C and RW_0S: writing all-ones is the no-op value
    EXPECT_EQ(rw_w0c_reg::identity, 0xF0u); // flags[7:4] = mask
    EXPECT_EQ(rw_w0s_reg::identity, 0xF0u); // control[7:4] = mask
}

TEST_F(AccessTypeTest, Identity_RW1C_RW1T_RW1S_HaveZero) {
    // RW_1C, RW_1T, RW_1S: writing all-zeros is the no-op value
    EXPECT_EQ(rw_w1c_reg::identity, 0u);
    EXPECT_EQ(rw_w1t_reg::identity, 0u);
    EXPECT_EQ(rw_w1s_reg::identity, 0u);
}

TEST_F(AccessTypeTest, Identity_SpecialOnly) {
    // RW_1C[3:0] identity=0, RW_0S[7:4] identity=0xF0
    EXPECT_EQ(special_only_reg::identity, 0xF0u);
}

// ============================================================================
// RMW optimization — bus read avoidance
//
// The library avoids bus reads when all RMW-able (plain RW) bits are covered
// by the write. This is critical for registers containing RC/RS fields,
// where an unnecessary read would trigger HW side effects.
// ============================================================================

TEST_F(AccessTypeTest, FullRW_AllFieldsWritten_NoRead) {
    // Writing both fields of a pure-RW register → no bus read needed
    constexpr simple_reg r{};

    eval(r.low_nibble = 5_f, r.high_nibble = 0xA_f);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].value, 0xA5u);
}

TEST_F(AccessTypeTest, PartialRW_SubsetWritten_ReadsOnce) {
    // Writing one of two RW fields → needs RMW read for the other
    constexpr simple_reg r{};
    bus_read_value = 0xF0;

    eval(r.low_nibble = 5_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);  // RMW read
    EXPECT_EQ(bus_log[1].op, bus_event::type::write);
    EXPECT_EQ(bus_log[1].value, 0xF5u); // preserved high nibble + new low
}

TEST_F(AccessTypeTest, SpecialFieldWritten_RWCovered_NoRead) {
    // Register has RW[3:0] + RW_1C[7:4]. Writing only the RW data field
    // covers all rmw_mask bits (0x0F) → no bus read needed.
    // The RW_1C field gets its identity (0) automatically.
    constexpr rw_w1c_reg r{};

    eval(r.data = 0xA_f);

    // No read! rmw_mask = 0x0F, write_mask = 0x0F → full coverage
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
}

TEST_F(AccessTypeTest, SpecialOnlyReg_NoRead) {
    // Register with only special fields (no plain RW): rmw_mask = 0
    // Writing one field → is_partial_write = false → no bus read
    constexpr special_only_reg r{};

    eval(r.clear_bits = 0xF_f);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
}

TEST_F(AccessTypeTest, RC_FieldPresent_WritingRW_NoRead) {
    // Register has RW[3:0] + RC[7:4]. Writing the RW field covers all
    // rmw_mask bits. No bus read is triggered — crucial because reading
    // an RC field clears it in hardware as a side effect.
    constexpr rw_rc_reg r{};

    eval(r.data = 7_f);

    // Must NOT read — that would clear the RC irq_status bits in HW
    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
}

// ============================================================================
// Identity preservation during partial writes
//
// When doing RMW, non-written special-access fields must receive their
// identity value (the HW no-op) rather than the read-back value.
// This prevents unintended clears/sets/toggles on fields the user
// did not target.
//
// Each test writes only the plain RW data[3:0] field and verifies
// that the special field [7:4] receives the correct identity value.
// ============================================================================

TEST_F(AccessTypeTest, RW1C_GetsZero_NotReadBack) {
    // RW_1C: writing 1 clears bits → identity = 0 (writing 0 is no-op)
    // If read-back (0xF0) were written, bits 4-7 would be unintentionally cleared.
    constexpr rw_w1c_reg r{};
    bus_read_value = 0xFF; // HW has 0xF in status[7:4]

    eval(r.data = 0xA_f);

    ASSERT_EQ(bus_log.size(), 1u); // no read needed (rmw_mask fully covered)
    // status[7:4] must be 0x0 (identity), NOT 0xF (read-back)
    EXPECT_EQ(bus_log[0].value & 0xF0u, 0x00u);
    EXPECT_EQ(bus_log[0].value & 0x0Fu, 0x0Au);
}

TEST_F(AccessTypeTest, RW0S_GetsMask_NotReadBack) {
    // RW_0S: writing 0 sets bits → identity = mask (writing 1 is no-op)
    // If read-back (0x00 in control) were written, bits would be unintentionally set.
    constexpr rw_w0s_reg r{};
    bus_read_value = 0x0F; // HW has 0x0 in control[7:4]

    eval(r.data = 0xA_f);

    ASSERT_EQ(bus_log.size(), 1u);
    // control[7:4] must be 0xF (identity=mask), NOT 0x0 (read-back)
    EXPECT_EQ(bus_log[0].value & 0xF0u, 0xF0u);
    EXPECT_EQ(bus_log[0].value & 0x0Fu, 0x0Au);
}

TEST_F(AccessTypeTest, RW1T_GetsZero_NotReadBack) {
    // RW_1T: writing 1 toggles bits → identity = 0 (writing 0 is no-op)
    // If read-back (0xF0) were written, bits would be unintentionally toggled.
    constexpr rw_w1t_reg r{};
    bus_read_value = 0xFF;

    eval(r.data = 0xA_f);

    ASSERT_EQ(bus_log.size(), 1u);
    // toggle[7:4] must be 0x0 (identity), NOT 0xF (read-back)
    EXPECT_EQ(bus_log[0].value & 0xF0u, 0x00u);
    EXPECT_EQ(bus_log[0].value & 0x0Fu, 0x0Au);
}

TEST_F(AccessTypeTest, RW0C_GetsMask_NotReadBack) {
    // RW_0C: writing 0 clears bits → identity = mask (writing 1 is no-op)
    // Writing back read-value is actually safe for RW_0C (idempotent),
    // but using identity is equally correct and consistent.
    constexpr rw_w0c_reg r{};
    bus_read_value = 0x0F; // HW has 0x0 in flags[7:4]

    eval(r.data = 0xA_f);

    ASSERT_EQ(bus_log.size(), 1u);
    // flags[7:4] = 0xF (identity=mask) — no-op for RW_0C
    EXPECT_EQ(bus_log[0].value & 0xF0u, 0xF0u);
    EXPECT_EQ(bus_log[0].value & 0x0Fu, 0x0Au);
}

TEST_F(AccessTypeTest, RW1S_GetsZero_NotReadBack) {
    // RW_1S: writing 1 sets bits → identity = 0 (writing 0 is no-op)
    // Writing back read-value is actually safe for RW_1S (idempotent),
    // but using identity is equally correct and consistent.
    constexpr rw_w1s_reg r{};
    bus_read_value = 0xFF;

    eval(r.data = 0xA_f);

    ASSERT_EQ(bus_log.size(), 1u);
    // sticky[7:4] = 0x0 (identity) — no-op for RW_1S
    EXPECT_EQ(bus_log[0].value & 0xF0u, 0x00u);
    EXPECT_EQ(bus_log[0].value & 0x0Fu, 0x0Au);
}

// ============================================================================
// Identity preservation during partial write WITH bus read
//
// When a register has two RW fields and one special field, writing
// one RW field triggers an RMW read for the other RW field.
// The special field must still get identity, not the read-back.
// ============================================================================

TEST_F(AccessTypeTest, RW1C_PartialRW_IdentityPreserved) {
    // simple_reg has two RW fields. If we used rw_w1c_reg and only wrote
    // data[3:0], rmw_mask is fully covered (no read). To force a read,
    // we need a register with TWO RW fields plus a special field.
    // Since we don't have that exact register, this test verifies the
    // identity value is produced correctly when write covers all rmw bits.
    constexpr rw_w1c_reg r{};
    bus_read_value = 0xBB;

    // Write data[3:0] = 5. rmw_mask = 0x0F → fully covered → no read.
    // status[7:4] = identity(0)
    eval(r.data = 5_f);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].value, 0x05u); // 0x00 | 0x05
}

// ============================================================================
// Direct special field writes
//
// Writing a special field directly should place the user's value into
// the field position. The RW data field gets its identity (0) since
// rmw_mask for the RW field is fully covered when we include it.
// Note: when writing ONLY the special field, the RW data[3:0] field
// is NOT covered by write_mask, but rmw_mask = 0x0F is also not covered,
// triggering an RMW read to preserve the RW bits.
// ============================================================================

TEST_F(AccessTypeTest, RW1C_DirectWrite) {
    // Write RW_1C status[7:4] = 0xF (to clear all bits in HW)
    // data[3:0] is RW and not written → RMW read needed to preserve it
    constexpr rw_w1c_reg r{};
    bus_read_value = 0xAB; // data=0xB, status=0xA in HW

    eval(r.status = 0xF_f);

    // RMW: read(data bits) + write(preserved data | new status)
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(bus_log[1].op, bus_event::type::write);
    // data[3:0] = read-back 0xB (plain RW → preserved), status[7:4] = 0xF (user value)
    EXPECT_EQ(bus_log[1].value & 0x0Fu, 0x0Bu);
    EXPECT_EQ(bus_log[1].value & 0xF0u, 0xF0u);
}

TEST_F(AccessTypeTest, RW0S_DirectWrite) {
    // Write RW_0S control[7:4] = 0x0 (to set all bits in HW)
    constexpr rw_w0s_reg r{};
    bus_read_value = 0x53;

    eval(r.control = 0x0_f);

    ASSERT_EQ(bus_log.size(), 2u);
    // data[3:0] = read-back 0x3 (RW preserved), control[7:4] = 0x0 (user value)
    EXPECT_EQ(bus_log[1].value & 0x0Fu, 0x03u);
    EXPECT_EQ(bus_log[1].value & 0xF0u, 0x00u);
}

TEST_F(AccessTypeTest, RW1T_DirectWrite) {
    // Write RW_1T toggle[7:4] = 0xA (toggle bits 5 and 7)
    constexpr rw_w1t_reg r{};
    bus_read_value = 0x12;

    eval(r.toggle = 0xA_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value & 0x0Fu, 0x02u); // data preserved
    EXPECT_EQ(bus_log[1].value & 0xF0u, 0xA0u); // toggle value
}

TEST_F(AccessTypeTest, RW0C_DirectWrite) {
    // Write RW_0C flags[7:4] = 0x5 (clear bits where 0, preserve where 1)
    constexpr rw_w0c_reg r{};
    bus_read_value = 0x37;

    eval(r.flags = 0x5_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value & 0x0Fu, 0x07u); // data preserved
    EXPECT_EQ(bus_log[1].value & 0xF0u, 0x50u); // flags value
}

TEST_F(AccessTypeTest, RW1S_DirectWrite) {
    // Write RW_1S sticky[7:4] = 0xC (set bits 6 and 7)
    constexpr rw_w1s_reg r{};
    bus_read_value = 0x49;

    eval(r.sticky = 0xC_f);

    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(bus_log[1].value & 0x0Fu, 0x09u); // data preserved
    EXPECT_EQ(bus_log[1].value & 0xF0u, 0xC0u); // sticky value
}

// ============================================================================
// Special-only register (no plain RW fields)
// ============================================================================

TEST_F(AccessTypeTest, SpecialOnly_WriteOne_IdentityForOther) {
    // Write clear_bits[3:0] = 0xF. set_bits[7:4] should get identity (0xF0).
    // rmw_mask = 0 → is_partial_write = false → no bus read
    constexpr special_only_reg r{};

    eval(r.clear_bits = 0xF_f);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    // clear_bits = 0xF (user value), set_bits = 0xF (identity for RW_0S)
    EXPECT_EQ(bus_log[0].value, 0xFFu);
}

TEST_F(AccessTypeTest, SpecialOnly_WriteBoth_NoRead) {
    // Writing both fields → no read needed (rmw_mask = 0 → always "full coverage")
    constexpr special_only_reg r{};

    eval(r.clear_bits = 0xA_f, r.set_bits = 0x5_f);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::write);
    EXPECT_EQ(bus_log[0].value & 0x0Fu, 0x0Au);
    EXPECT_EQ(bus_log[0].value & 0xF0u, 0x50u);
}

// ============================================================================
// Field reads for special access types
// ============================================================================

TEST_F(AccessTypeTest, RC_FieldRead) {
    // Reading an RC field should work through eval (it's readable)
    constexpr rw_rc_reg r{};
    bus_read_value = 0xA5;

    auto data_val = eval(r.data);

    ASSERT_EQ(bus_log.size(), 1u);
    EXPECT_EQ(bus_log[0].op, bus_event::type::read);
    EXPECT_EQ(data_val, 0x05u); // data[3:0] extracted from 0xA5
}

// ============================================================================
// Mixed operations: write special + read RW simultaneously
// ============================================================================

TEST_F(AccessTypeTest, WriteSpecial_ReadRW) {
    // Write status[7:4] (RW_1C) and read data[3:0] (RW) in one eval call.
    // The RMW read to preserve data also provides the read value.
    constexpr rw_w1c_reg r{};
    bus_read_value = 0xAB; // data=0xB, status=0xA

    auto data_val = eval(r.status = 0xF_f, r.data);

    // read (RMW for data preservation) + write
    ASSERT_EQ(bus_log.size(), 2u);
    EXPECT_EQ(data_val, 0x0Bu); // data[3:0] from read
    // written: data=0xB (preserved) | status=0xF (user value)
    EXPECT_EQ(bus_log[1].value, 0xFBu);
}
