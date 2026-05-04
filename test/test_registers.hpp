#pragma once

#include <ros/access.hpp>
#include <ros/field.hpp>
#include <ros/reg.hpp>
#include "mock_bus.hpp"

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

} // namespace test
