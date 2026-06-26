#pragma once

#include <cstddef>
#include <cstdint>
#include <tuple>
#include <vector>

namespace test {

struct bus_event {
    enum class type : std::uint8_t { read, write };
    type op;
    std::size_t address;
    std::uint32_t value;
};

inline std::vector<bus_event> bus_log;
inline std::uint32_t bus_read_value = 0;

inline void reset_bus() {
    bus_log.clear();
    bus_read_value = 0;
}

struct mock_bus {
    template <typename T, typename Addr> static auto read(Addr address) -> T {
        bus_log.push_back({bus_event::type::read,
                           static_cast<std::size_t>(address), bus_read_value});
        return static_cast<T>(bus_read_value);
    }

    template <typename T, typename Addr>
    static void write(T val, Addr address) {
        bus_log.push_back({bus_event::type::write,
                           static_cast<std::size_t>(address),
                           static_cast<std::uint32_t>(val)});
    }

    template <typename... AdjacentAddrs, typename... ValueTypes>
    static auto
    read(std::tuple<AdjacentAddrs...> addrs) -> std::tuple<ValueTypes...>;

    template <typename... AdjacentAddrs, typename... ValueTypes>
    static void write(std::tuple<AdjacentAddrs...> addrs,
                      std::tuple<ValueTypes...> values);
};

} // namespace test
