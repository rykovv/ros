#pragma once

#include <ros/literals.hpp>

namespace ros {
// field type traits
namespace detail {

template <typename> struct is_field_read {
    constexpr static bool value = false;
};

template <typename Field> struct is_field_read<ros::detail::field_read<Field>> {
    constexpr static bool value = true;
};

template <typename Field>
constexpr bool is_field_read_v = is_field_read<Field>::value;

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

template <typename> struct is_register_read {
    constexpr static bool value = false;
};

template <typename Register>
struct is_register_read<ros::detail::register_read<Register>> {
    constexpr static bool value = true;
};

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