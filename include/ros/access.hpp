#pragma once

#include <cstdint>
#include <tuple>

namespace ros {
// Based on IEEE 1685 (IP-XACT) and ARM CMSIS-SVD
// clang-format off
enum class access_type : std::uint8_t {    
    RSVD  = 0b0'0'00'00'00,
    RO    = 0b0'0'00'00'01,
    WO    = 0b0'0'00'00'10,
    RW    = 0b0'0'00'00'11,
    RW_0C = 0b0'0'00'01'11, // clear on write 0
    RW_1C = 0b0'0'00'10'11, // clear on write 1
    RC    = 0b0'0'00'11'01, // read-clear
    RW_0S = 0b0'0'01'00'11, // set on write 0
    RW_1S = 0b0'0'10'00'11, // set on write 1
    RS    = 0b0'0'11'00'01, // read-set
    RW_1T = 0b0'1'00'00'11, // toggle on write 1
    RW_O  = 0b1'0'00'00'11, // read-write once
    W1T   = RW_1T,
    R     = RO,
    W     = WO,
    RESERVED = RSVD
};
// clang-format on

struct bus {
    template <typename T, typename Addr> static auto read(Addr address) -> T;
    template <typename T, typename Addr> static void write(T val, Addr address);
    template <typename... AdjacentAddrs, typename... ValueTypes>
    static auto read(std::tuple<AdjacentAddrs...> addrs) -> std::tuple<ValueTypes...>;
    template <typename... AdjacentAddrs, typename... ValueTypes>
    static void write(std::tuple<AdjacentAddrs...> addrs,
                      std::tuple<ValueTypes...> values);
};
} // namespace ros