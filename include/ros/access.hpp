#pragma once

#include <cstdint>
#include <tuple>

namespace ros {
enum class access_type : std::uint8_t {
    NA = 0b000'00000,
    RO = 0b000'00001,
    WO = 0b000'00010,
    RW = 0b000'00011,
    RW_0C = 0b000'00111,
    RW_1C = 0b000'01011,
    RW_1S = 0b000'10011,
    R = RO,
    W = WO
};

struct bus {
    template <typename T, typename Addr> static T read(Addr address);
    template <typename T, typename Addr> static void write(T val, Addr address);
    template <typename... AdjacentAddrs, typename... ValueTypes>
    static std::tuple<ValueTypes...> read(std::tuple<AdjacentAddrs...> addrs);
    template <typename... AdjacentAddrs, typename... ValueTypes>
    static void write(std::tuple<AdjacentAddrs...> addrs,
                      std::tuple<ValueTypes...> values);
};
} // namespace ros