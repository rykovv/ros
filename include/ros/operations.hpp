#pragma once

#include <tuple>
#include <ros/reflection.hpp>


namespace ros {
// field operations
namespace detail {

template <typename Field> struct field_assignment {
    using type = Field;
};

template <typename Field, typename Field::value_type val>
struct field_assignment_ct : field_assignment<Field> {
    constexpr static typename Field::value_type value = val;
};

template <typename Field> struct field_assignment_rt : field_assignment<Field> {
    using value_type = typename Field::value_type;

    constexpr explicit field_assignment_rt(value_type v) : value{v} {}

    value_type value;
};

template <typename F, typename FieldOp, typename Field0, typename... Fields>
struct field_assignment_invocable : field_assignment<FieldOp> {
    using fields = std::tuple<Field0, Fields...>;

    explicit field_assignment_invocable(F f) : lambda_{f} {}

    constexpr auto operator()(typename Field0::value_type f0,
                              typename Fields::value_type... fs) ->
        typename FieldOp::value_type {
        return lambda_(f0, fs...);
    }

    F lambda_;
};

template <typename Field> struct unsafe_field_operations_handler {
    using value_type = typename Field::value_type;

    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(auto const &rhs) const
        -> field_assignment_rt<Field> {
        static_assert(Field::access != access_type::RO,
                      "cannot write read-only field");

        return field_assignment_rt<Field>{static_cast<value_type>(rhs)};
    }

    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(auto &&rhs) const -> field_assignment_rt<Field> {
        static_assert(Field::access != access_type::RO,
                      "cannot write read-only field");

        return field_assignment_rt<Field>{static_cast<value_type>(rhs)};
    }

    // TODO: add compile time unsafe operations
};

} // namespace detail

// reg operations
namespace detail {
template <typename Register> struct register_assignment {
    using type = Register;
};

template <typename Register, typename Register::value_type val>
struct register_assignment_ct : register_assignment<Register> {
    constexpr static typename Register::value_type value = val;
};

template <typename Register>
struct register_assignment_rt : register_assignment<Register> {
    using value_type = typename Register::value_type;

    constexpr explicit register_assignment_rt(value_type v) : value{v} {}

    value_type value;
};

template <typename F, typename RegisterOp, typename Register0,
          typename... Registers>
struct register_assignment_invocable : register_assignment<RegisterOp> {
    using registerOp = RegisterOp;
    using registers = std::tuple<Register0, Registers...>;

    explicit register_assignment_invocable(F f) : lambda_{f} {}

    constexpr auto operator()(typename Register0::value_type r0,
                              typename Registers::value_type... rs) ->
        typename RegisterOp::value_type {
        return lambda_(r0, rs...);
    }

    F lambda_;
};

template <typename Register> struct unsafe_register_operations_handler {
    using value_type = typename Register::value_type;

    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(auto const &rhs) const
        -> register_assignment_rt<Register> {
        // safe static_case because assignment overload checked type and width
        // validity
        return register_assignment_rt<Register>{static_cast<value_type>(rhs)};
    }

    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(auto &&rhs) const
        -> register_assignment_rt<Register> {
        // safe static_case because assignment overload checked type and width
        // validity
        return register_assignment_rt<Register>{static_cast<value_type>(rhs)};
    }

    // TODO: Add compile time unsafe operations support
};

template <typename T, typename... Ts, std::size_t... Idx>
constexpr auto get_rmw_mask_helper(std::tuple<T, Ts...> const &t,
                                   std::index_sequence<Idx...>) ->
    typename T::value_type_r {
    return (std::get<Idx>(t).rmw_mask | ...);
};

template <typename reg>
constexpr auto get_rmw_mask(reg const &r) -> typename reg::value_type {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return get_rmw_mask_helper(tup, std::make_index_sequence<tup_size>{});
}

template <typename T, typename... Ts, std::size_t... Idx>
constexpr auto get_writable_mask_helper(std::tuple<T, Ts...> const &t,
                                        std::index_sequence<Idx...>) ->
    typename T::value_type_r {
    return ((std::get<Idx>(t).writable() ? std::get<Idx>(t).mask : 0u) | ...);
};

template <typename reg>
constexpr auto get_writable_mask(reg const &r) -> typename reg::value_type {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return get_writable_mask_helper(tup, std::make_index_sequence<tup_size>{});
}

// register operations safety
template <typename Tup, std::size_t... Idx>
constexpr auto has_wo_helper(access_type at, Tup const &t,
                             std::index_sequence<Idx...>) -> bool {
    return (std::tuple_element_t<Idx, Tup>::writeonly() or ...);
};

template <typename reg> constexpr auto has_wo_access(reg const &r) -> bool {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return has_wo_helper(access_type::W, tup,
                         std::make_index_sequence<tup_size>{});
}

template <typename T, typename... Ts, std::size_t... Idx>
constexpr auto get_identity_mask_helper(std::tuple<T, Ts...> const &t,
                                        std::index_sequence<Idx...>) ->
    typename T::value_type_r {
    return (std::tuple_element_t<Idx, std::tuple<T, Ts...>>::identity() | ...);
};

template <typename reg>
constexpr auto get_identity_mask(reg const &r) -> typename reg::value_type {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return get_identity_mask_helper(tup, std::make_index_sequence<tup_size>{});
}

template <typename Tup, std::size_t... Idx>
constexpr auto get_ro_mask_helper(Tup const &t, std::index_sequence<Idx...>) {
    return ((std::tuple_element_t<Idx, Tup>::readonly() ? std::get<Idx>(t).mask
                                                        : 0) |
            ...);
};

template <typename reg, typename T = reg::value_type>
constexpr auto get_ro_mask(reg const &r) -> T {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return get_ro_mask_helper(tup, std::make_index_sequence<tup_size>{});
}
} // namespace detail
} // namespace ros