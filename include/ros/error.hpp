#pragma once

#include <concepts>
#include <type_traits>
#include <limits>
#include <ros/utils.hpp>


namespace ros {
namespace detail {
template <typename T>
concept enumeration = std::is_enum_v<T>;
} // namespace detail
namespace error {

template <typename Field, typename T = typename Field::value_type>
using field_error_handler = T (*)(T);

template <typename Field, typename T = typename Field::value_type>
constexpr field_error_handler<Field> ignore_handler = [](T v) -> T {
    return T{0};
};

template <typename Field, typename T = typename Field::value_type>
    requires std::unsigned_integral<T>
constexpr auto trancate_handler(T v) -> T {
    return v & (Field::mask >> Field::lsb());
}

template <typename Field, typename T = typename Field::value_type>
    requires detail::enumeration<T>
constexpr auto trancate_handler(T v) -> T {
    using U = std::underlying_type_t<T>;
    
    // caution: can be cast to a non-existent enum value
    return static_cast<T>(detail::to_underlying(v) & (Field::mask >> Field::lsb()));
}

template <typename Field>
constexpr field_error_handler<Field> handle_field_error = trancate_handler<Field>;

template <typename Register, typename T = typename Register::value_type>
using register_error_handler = T (*)(T);

template <typename Register, typename T = typename Register::value_type>
constexpr register_error_handler<Register> mask_handler = [](T v) -> T {
    return static_cast<T>(v & Register::writable_mask);
};

template <typename Register>
constexpr field_error_handler<Register> handle_register_error =
    mask_handler<Register>;

} // namespace error
} // namespace ros