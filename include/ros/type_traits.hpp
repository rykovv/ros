#pragma once

#include <type_traits>

#include <ros/literals.hpp>
#include <ros/operations.hpp>

namespace ros {
// generic type traits
namespace detail {
template <typename T, typename enable = void> struct unwrap_enum {
    using type = T;
};

template <typename T>
struct unwrap_enum<T, typename std::enable_if_t<std::is_enum_v<T>>> {
    using type = std::underlying_type_t<T>;
};

template <typename T> using unwrap_enum_t = typename unwrap_enum<T>::type;
} // namespace detail

// field type traits
namespace detail {

// base case forward declaration for specialization in field.hpp
template <typename T> struct is_field : std::false_type {};
template <typename T> constexpr bool is_field_v = is_field<T>::value;

template <typename T> struct is_field_read : is_field<T> {};

template <typename> struct is_field_assignment_ct {
    constexpr static bool value = false;
};

template <typename Field, typename Field::value_type val>
struct is_field_assignment_ct<ros::detail::field_assignment_ct<Field, val>> {
    constexpr static bool value = true;
};

template <typename> struct is_field_assignment_rt {
    constexpr static bool value = false;
};

template <typename Field>
struct is_field_assignment_rt<ros::detail::field_assignment_rt<Field>> {
    constexpr static bool value = true;
};

template <typename...>
struct is_field_assignment_invocable : std::false_type {};

template <typename F, typename... Fields>
struct is_field_assignment_invocable<
    ros::detail::field_assignment_invocable<F, Fields...>> : std::true_type {};

template <typename... Ops> struct one_field_assignment_per_apply;

template <> struct one_field_assignment_per_apply<> : std::true_type {};

template <typename Op>
struct one_field_assignment_per_apply<Op> : std::true_type {};

template <typename Op, typename... Ops>
struct one_field_assignment_per_apply<Op, Ops...> {
    using OpField = typename Op::type;
    constexpr static bool more_than_one =
        (std::is_base_of_v<detail::field_assignment<OpField>, Ops> || ...);
    constexpr static bool value =
        not more_than_one and one_field_assignment_per_apply<Ops...>::value;
};

template <typename... Ops>
constexpr bool one_field_assignment_per_apply_v =
    one_field_assignment_per_apply<Ops...>::value;

} // namespace detail

// reg type traits
namespace detail {
template <typename...> struct is_register_assignment_ct : std::false_type {};

template <typename Register, typename Register::value_type val>
struct is_register_assignment_ct<
    ros::detail::register_assignment_ct<Register, val>> : std::true_type {};

template <typename> struct is_register_assignment_rt : std::false_type {};

template <typename Register>
struct is_register_assignment_rt<ros::detail::register_assignment_rt<Register>>
    : std::true_type {};

template <typename...>
struct is_register_assignment_invocable : std::false_type {};

template <typename F, typename... Registers>
struct is_register_assignment_invocable<
    ros::detail::register_assignment_invocable<F, Registers...>>
    : std::true_type {};

// base case forward declaration for specialization in reg.hpp
template <typename T>
struct is_register_assignment
    : std::is_base_of<register_assignment<typename T::type>, T> {};
template <typename T>
constexpr bool is_register_assignment_v = is_register_assignment<T>::value;

template <typename T> struct is_derived_reg {
    constexpr static bool value = std::is_base_of_v<typename T::type, T> and
                                  not std::is_same_v<typename T::type, T>;
};
template <typename T>
constexpr bool is_derived_reg_v = is_derived_reg<T>::value;

template <typename T> struct is_register_read : is_derived_reg<T> {};

template <typename... Ops> struct one_register_assignment_per_apply;

template <> struct one_register_assignment_per_apply<> : std::true_type {};

template <typename Op>
struct one_register_assignment_per_apply<Op> : std::true_type {};

template <typename Op, typename... Ops>
struct one_register_assignment_per_apply<Op, Ops...> {
    using OpRegister = typename Op::type;
    constexpr static bool more_than_one =
        (std::is_base_of_v<detail::register_assignment<OpRegister>, Ops> ||
         ...);
    constexpr static bool value =
        not more_than_one and one_register_assignment_per_apply<Ops...>::value;
};

template <typename... Ops>
constexpr bool one_register_assignment_per_apply_v =
    one_register_assignment_per_apply<Ops...>::value;
} // namespace detail
} // namespace ros