#pragma once

#include <ros/access.hpp>
#include <ros/field.hpp>
#include <ros/reg.hpp>

using namespace ros;
using namespace ros::literals;

struct dummy_bus {
    template <typename T, typename Addr>
    static T read(Addr) { return T{}; }

    template <typename T, typename Addr>
    static void write(T, Addr) {}
};

struct rw_reg : reg<rw_reg, uint8_t, 0x10_addr, dummy_bus> {
    using reg<rw_reg, uint8_t, 0x10_addr, dummy_bus>::operator=;

    field<rw_reg, 3_msb, 0_lsb, access_type::RW> low;
    field<rw_reg, 7_msb, 4_lsb, access_type::RW> high;
};

struct mixed_reg : reg<mixed_reg, uint16_t, 0x20_addr, dummy_bus> {
    using reg<mixed_reg, uint16_t, 0x20_addr, dummy_bus>::operator=;

    field<mixed_reg, 3_msb, 0_lsb, access_type::RO> status;
    field<mixed_reg, 7_msb, 4_lsb, access_type::RW> config;
    field<mixed_reg, 11_msb, 8_lsb, access_type::WO> command;
    field<mixed_reg, 15_msb, 12_lsb, access_type::RW> flags;
};

struct ro_reg : reg<ro_reg, uint32_t, 0x30_addr, dummy_bus> {
    using reg<ro_reg, uint32_t, 0x30_addr, dummy_bus>::operator=;

    field<ro_reg, 7_msb, 0_lsb, access_type::RO> byte0;
    field<ro_reg, 15_msb, 8_lsb, access_type::RO> byte1;
};

struct wo_reg : reg<wo_reg, uint32_t, 0x40_addr, dummy_bus> {
    using reg<wo_reg, uint32_t, 0x40_addr, dummy_bus>::operator=;
    
    field<wo_reg, 7_msb, 0_lsb, access_type::WO> byte0;
    field<wo_reg, 15_msb, 8_lsb, access_type::WO> byte1;
};
