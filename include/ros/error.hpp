#pragma once

// TODO: remove std::cout
#include <iostream>

namespace ros {
namespace error {

template <typename Field, typename T = typename Field::value_type>
using field_error_handler = T (*)(T);

template <typename Field, typename T = typename Field::value_type>
constexpr field_error_handler<Field> ignore_handler = [](T v) -> T {
    std::cout << "ignore handler with " << v << std::endl;
    return T{0};
};
template <typename Field, typename T = typename Field::value_type>
constexpr field_error_handler<Field> clamp_handler = [](T v) -> T {
    using value_type_r = typename Field::value_type_r;
    std::cout << "clamp handler with " << static_cast<value_type_r>(v)
              << std::endl;
    return T{((1u << Field::length) - 1)};
};

template <typename Field>
constexpr field_error_handler<Field> handle_field_error = clamp_handler<Field>;

template <typename Register, typename T = typename Register::value_type>
using register_error_handler = T (*)(T);

template <typename Register, typename T = typename Register::value_type>
constexpr register_error_handler<Register> mask_handler = [](T v) -> T {
    std::cout << "Attempt to assign read-only bits with " << v << std::endl;
    return static_cast<T>(v & Register::writable_mask);
};

template <typename Register>
constexpr field_error_handler<Register> handle_register_error =
    mask_handler<Register>;

} // namespace error
} // namespace ros