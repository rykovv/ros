#pragma once

#include <ros/type_traits.hpp>

namespace ros {
namespace detail {

template <typename T, typename enable = void> struct unwrap_enum {
    using type = T;
};

template <typename T>
struct unwrap_enum<T, typename std::enable_if_t<std::is_enum_v<T>>> {
    using type = std::underlying_type_t<T>;
};

template <typename T> using unwrap_enum_t = typename unwrap_enum<T>::type;

template <typename T>
concept derivable_unsigned_integral = std::unsigned_integral<unwrap_enum_t<T>>;

template <typename T, detail::msb msb, detail::lsb lsb>
concept field_selectable = (derivable_unsigned_integral<T>) &&
                           (msb.value <= std::numeric_limits<T>::digits - 1 &&
                            lsb.value <= std::numeric_limits<T>::digits - 1) &&
                           (msb.value >= lsb.value);

template <typename... Ops>
concept one_assignment_per_field = one_field_assignment_per_apply_v<Ops...>;

template <typename Op, typename... Ops>
concept same_register =
    (std::is_same_v<typename Op::type::reg, typename Ops::type::reg> && ...);

template <typename... Ops>
concept field_operations = (is_field_v<typename Ops::type> && ...);

template <typename Op, typename... Ops>
concept field_constraints =
    field_operations<Op, Ops...> and same_register<Op, Ops...> and
    one_assignment_per_field<Op, Ops...>;

template <typename... Ops>
concept one_assignment_per_register =
    one_register_assignment_per_apply_v<Ops...>;

template <typename... Ops>
concept register_operations =
    ((is_derived_reg_v<Ops> || is_register_assignment_v<Ops>) && ...);

template <typename Op, typename... Ops>
concept register_constraints =
    register_operations<Op, Ops...> and one_assignment_per_register<Op, Ops...>;

} // namespace detail
} // namespace ros