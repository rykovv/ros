#pragma once

#include <concepts>
#include <type_traits>
#include <limits>


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
    requires std::integral<T>
constexpr auto clamp_handler(T v) -> T {
    return T{Field::length == std::numeric_limits<T>::digits
              ? ~T{0}
              : ((T{1} << Field::length) - 1)};
}

template <typename Field, typename T = typename Field::value_type>
    requires detail::enumeration<T>
constexpr auto clamp_handler(T v) -> T {
    using U = std::underlying_type_t<T>;

    U mask = Field::length == std::numeric_limits<U>::digits
             ? ~U{0}
             : ((U{1} << Field::length) - 1);
    
    // caution: can be cast to a non-existent enum value
    return static_cast<T>(mask);
}

template <typename Field>
constexpr field_error_handler<Field> handle_field_error = clamp_handler<Field>;

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