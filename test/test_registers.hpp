#pragma once

#include "mock_bus.hpp"

#include <ros/access.hpp>
#include <ros/field.hpp>
#include <ros/reg.hpp>

namespace test {

using namespace ros;
using namespace ros::literals;

// A simple 8-bit register at address 0x10 with two RW fields
struct simple_reg : reg<simple_reg, uint8_t, 0x10_addr, mock_bus> {
    using reg<simple_reg, uint8_t, 0x10_addr, mock_bus>::operator=;

    field<simple_reg, 3_msb, 0_lsb, access_type::RW> low_nibble;
    field<simple_reg, 7_msb, 4_lsb, access_type::RW> high_nibble;
};

// A 16-bit register with mixed access fields
struct mixed_reg : reg<mixed_reg, uint16_t, 0x20_addr, mock_bus> {
    using reg<mixed_reg, uint16_t, 0x20_addr, mock_bus>::operator=;

    field<mixed_reg, 3_msb, 0_lsb, access_type::RO> status;
    field<mixed_reg, 7_msb, 4_lsb, access_type::RW> config;
    field<mixed_reg, 11_msb, 8_lsb, access_type::WO> command;
    field<mixed_reg, 15_msb, 12_lsb, access_type::RW> flags;
};

// A read-only register
struct ro_reg : reg<ro_reg, uint32_t, 0x30_addr, mock_bus> {
    using reg<ro_reg, uint32_t, 0x30_addr, mock_bus>::operator=;

    field<ro_reg, 7_msb, 0_lsb, access_type::RO> byte0;
    field<ro_reg, 15_msb, 8_lsb, access_type::RO> byte1;
};

// A write-only register
struct wo_reg : reg<wo_reg, uint32_t, 0x40_addr, mock_bus> {
    using reg<wo_reg, uint32_t, 0x40_addr, mock_bus>::operator=;

    field<wo_reg, 7_msb, 0_lsb, access_type::WO> byte0;
    field<wo_reg, 15_msb, 8_lsb, access_type::WO> byte1;
};

// Single-bit field register
struct bit_reg : reg<bit_reg, uint8_t, 0x50_addr, mock_bus> {
    using reg<bit_reg, uint8_t, 0x50_addr, mock_bus>::operator=;

    field<bit_reg, 0_msb, 0_lsb, access_type::RW> enable;
    field<bit_reg, 1_msb, 1_lsb, access_type::RW> reset;
    field<bit_reg, 7_msb, 2_lsb, access_type::RW> data;
};

// Full-width register (all bits writable)
struct full_reg : reg<full_reg, uint32_t, 0x60_addr, mock_bus> {
    using reg<full_reg, uint32_t, 0x60_addr, mock_bus>::operator=;

    field<full_reg, 31_msb, 0_lsb, access_type::RW> value;
};

// Register with special access types (RC, RW_0C, RW_1C, RW_1S, RW_0S, RW_1T)
struct special_reg : reg<special_reg, uint32_t, 0x70_addr, mock_bus> {
    using reg<special_reg, uint32_t, 0x70_addr, mock_bus>::operator=;

    field<special_reg, 3_msb, 0_lsb, access_type::RC>
        clear_on_read; // read-clear
    field<special_reg, 7_msb, 4_lsb, access_type::RW_0C>
        clear_on_w0; // clear on write 0
    field<special_reg, 11_msb, 8_lsb, access_type::RW_1C>
        clear_on_w1; // clear on write 1
    field<special_reg, 15_msb, 12_lsb, access_type::RW_1S>
        set_on_w1; // set on write 1
    field<special_reg, 19_msb, 16_lsb, access_type::RW_0S>
        set_on_w0; // set on write 0
    field<special_reg, 23_msb, 20_lsb, access_type::RW_1T>
        toggle_on_w1; // toggle on write 1
    field<special_reg, 27_msb, 24_lsb, access_type::RS> set_on_read; // read-set
    field<special_reg, 31_msb, 28_lsb, access_type::RW_O>
        write_once; // read-write once
};

// Enum for field assignment testing
struct enum_reg : reg<enum_reg, uint8_t, 0x80_addr, mock_bus> {
    using reg<enum_reg, uint8_t, 0x80_addr, mock_bus>::operator=;

    enum class mode : uint8_t { OFF = 0, LOW = 1, HIGH = 2, TURBO = 3 };

    field<enum_reg, 1_msb, 0_lsb, access_type::RW, mode> mode_field;
    field<enum_reg, 7_msb, 2_lsb, access_type::RW> data;
};

// ============================================================================
// Focused registers for access_type-dependent behavior testing.
// Each pairs one RW field with one special-access field so that
// RMW identity preservation can be tested in isolation.
// ============================================================================

// RW + RW_1C — identity for RW_1C is 0 (writing 0 = no-op)
struct rw_w1c_reg : reg<rw_w1c_reg, uint8_t, 0xA0_addr, mock_bus> {
    using reg<rw_w1c_reg, uint8_t, 0xA0_addr, mock_bus>::operator=;

    field<rw_w1c_reg, 3_msb, 0_lsb, access_type::RW> data;
    field<rw_w1c_reg, 7_msb, 4_lsb, access_type::RW_1C> status;
};

// RW + RW_0S — identity for RW_0S is mask (writing 1 = no-op)
struct rw_w0s_reg : reg<rw_w0s_reg, uint8_t, 0xA1_addr, mock_bus> {
    using reg<rw_w0s_reg, uint8_t, 0xA1_addr, mock_bus>::operator=;

    field<rw_w0s_reg, 3_msb, 0_lsb, access_type::RW> data;
    field<rw_w0s_reg, 7_msb, 4_lsb, access_type::RW_0S> control;
};

// RW + RW_1T — identity for RW_1T is 0 (writing 0 = no-op)
struct rw_w1t_reg : reg<rw_w1t_reg, uint8_t, 0xA2_addr, mock_bus> {
    using reg<rw_w1t_reg, uint8_t, 0xA2_addr, mock_bus>::operator=;

    field<rw_w1t_reg, 3_msb, 0_lsb, access_type::RW> data;
    field<rw_w1t_reg, 7_msb, 4_lsb, access_type::RW_1T> toggle;
};

// RW + RW_0C — identity for RW_0C is mask (writing 1 = no-op)
struct rw_w0c_reg : reg<rw_w0c_reg, uint8_t, 0xA3_addr, mock_bus> {
    using reg<rw_w0c_reg, uint8_t, 0xA3_addr, mock_bus>::operator=;

    field<rw_w0c_reg, 3_msb, 0_lsb, access_type::RW> data;
    field<rw_w0c_reg, 7_msb, 4_lsb, access_type::RW_0C> flags;
};

// RW + RW_1S — identity for RW_1S is 0 (writing 0 = no-op)
struct rw_w1s_reg : reg<rw_w1s_reg, uint8_t, 0xA4_addr, mock_bus> {
    using reg<rw_w1s_reg, uint8_t, 0xA4_addr, mock_bus>::operator=;

    field<rw_w1s_reg, 3_msb, 0_lsb, access_type::RW> data;
    field<rw_w1s_reg, 7_msb, 4_lsb, access_type::RW_1S> sticky;
};

// RW + RC — tests that writing the RW field doesn't trigger a bus read
// that would inadvertently clear RC bits in hardware
struct rw_rc_reg : reg<rw_rc_reg, uint8_t, 0xA5_addr, mock_bus> {
    using reg<rw_rc_reg, uint8_t, 0xA5_addr, mock_bus>::operator=;

    field<rw_rc_reg, 3_msb, 0_lsb, access_type::RW> data;
    field<rw_rc_reg, 7_msb, 4_lsb, access_type::RC> irq_status;
};

// Only special fields (no plain RW) — tests that eval works
// without any RMW-able fields; rmw_mask = 0
struct special_only_reg : reg<special_only_reg, uint8_t, 0xA6_addr, mock_bus> {
    using reg<special_only_reg, uint8_t, 0xA6_addr, mock_bus>::operator=;

    field<special_only_reg, 3_msb, 0_lsb, access_type::RW_1C> clear_bits;
    field<special_only_reg, 7_msb, 4_lsb, access_type::RW_0S> set_bits;
};

} // namespace test
