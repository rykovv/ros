#pragma once

#include <ros/constant.hpp>

#include <concepts>
// #include <ros/concepts.hpp>

#pragma once

namespace ros {
namespace detail {

template <typename T>
concept field_type = std::integral<T> || std::is_enum_v<T>;

template <field_type T, T val> struct field_value {
    constexpr static T value = val;
};

template <typename T>
concept register_type = std::integral<T>;

template <register_type T, T val> struct register_value {
    constexpr static T value = val;
};

template <std::unsigned_integral MsbT, MsbT val> struct msb {
    constexpr static MsbT value = val;
};

template <std::unsigned_integral LsbT, LsbT val> struct lsb {
    constexpr static LsbT value = val;
};

template <std::unsigned_integral AddrT, AddrT val> struct addr {
    constexpr static AddrT value = val;
};

} // namespace detail

namespace literals {

template <char... Chars> constexpr auto operator""_f() {
    constexpr std::size_t new_value =
        ros::detail::to_constant<std::size_t, Chars...>();
    return detail::field_value<std::size_t, new_value>{};
}

template <char... Chars> constexpr auto operator""_r() {
    constexpr std::size_t new_value =
        ros::detail::to_constant<std::size_t, Chars...>();
    return detail::register_value<std::size_t, new_value>{};
}

template <char... Chars> constexpr auto operator""_msb() {
    constexpr std::size_t new_value =
        ros::detail::to_constant<std::size_t, Chars...>();
    return detail::msb<std::size_t, new_value>{};
}

template <char... Chars> constexpr auto operator""_lsb() {
    constexpr std::size_t new_value =
        ros::detail::to_constant<std::size_t, Chars...>();
    return detail::lsb<std::size_t, new_value>{};
}

template <char... Chars> constexpr auto operator""_addr() {
    constexpr std::size_t new_value =
        ros::detail::to_constant<std::size_t, Chars...>();
    return detail::addr<std::size_t, new_value>{};
}

} // namespace literals
} // namespace ros