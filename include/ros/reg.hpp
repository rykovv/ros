#pragma once

#include <ros/concepts.hpp>
#include <ros/error.hpp>
#include <ros/literals.hpp>
#include <ros/operations.hpp>
#include <ros/reflection.hpp>

namespace ros {
namespace detail {

template <typename Register> struct safe_register_operations_handler {
    using reg = Register;
    using value_type = typename Register::value_type;

    constexpr auto read() const -> register_read<Register> {
        return register_read<Register>{};
    }

    template <typename U, U val>
        requires(std::is_convertible_v<U, value_type>)
    constexpr auto operator=(register_value<U, val>) const
        -> register_assignment_ct<Register, val> const {
        static_assert(static_cast<value_type>(val & ~Register::layout) == 0,
                      "Attempt to assign read-only bits");
        return register_assignment_ct<Register, val>{};
    }

    template <typename U>
        requires std::integral<U> && std::is_convertible_v<U, value_type>
    constexpr auto
    operator=(U const &rhs) const -> register_assignment_rt<Register> {
        static_assert(std::numeric_limits<value_type>::digits >=
                          std::numeric_limits<U>::digits,
                      "Unsafe assignment. Assigned value type is too wide.");

        value_type value;
        if (rhs & ~Register::layout) {
            value = error::handle_register_error<Register>(rhs);
        } else {
            value = rhs;
        }

        return register_assignment_rt<Register>{value};
    }

    template <typename U>
        requires std::integral<U> && std::is_convertible_v<U, value_type>
    constexpr auto
    operator=(U &&rhs) const -> register_assignment_rt<Register> {
        static_assert(std::numeric_limits<value_type>::digits >=
                          std::numeric_limits<U>::digits,
                      "Unsafe assignment. Assigned value type is too wide.");

        value_type value;
        if (rhs & ~Register::layout) {
            value = error::handle_register_error<Register>(rhs);
        } else {
            value = rhs;
        }

        return register_assignment_rt<Register>{value};
    }

    constexpr auto operator()(std::invocable<value_type> auto f) const
        -> register_assignment_invocable<decltype(f), Register, Register> {
        return register_assignment_invocable<decltype(f), Register, Register>{
            f};
    }

    template <typename F, typename RegOpsHandlerT0, typename... RegOpsHandlerTs>
        requires std::invocable<F, typename RegOpsHandlerT0::reg::value_type,
                                typename RegOpsHandlerTs::reg::value_type...>
    constexpr auto operator()(F f, RegOpsHandlerT0 rh0,
                              RegOpsHandlerTs... rhs) const
        -> register_assignment_invocable<F, Register,
                                         typename RegOpsHandlerT0::reg,
                                         typename RegOpsHandlerTs::reg...> {
        return register_assignment_invocable<F, Register,
                                             typename RegOpsHandlerT0::reg,
                                             typename RegOpsHandlerTs::reg...>{
            f};
    }
};

template <typename Register> struct unsafe_register_operations_handler {
    using value_type = typename Register::value_type;

    constexpr auto
    operator=(auto const &rhs) const -> register_assignment_rt<Register> {
        // safe static_case because assignment overload checked type and width
        // validity
        return register_assignment_rt<Register>{static_cast<value_type>(rhs)};
    }

    constexpr auto
    operator=(auto &&rhs) const -> register_assignment_rt<Register> {
        // safe static_case because assignment overload checked type and width
        // validity
        return register_assignment_rt<Register>{static_cast<value_type>(rhs)};
    }

    // TODO: Add compile time unsafe operations support
};

template <typename T, typename... Ts, std::size_t... Idx>
constexpr auto get_rwm_mask_helper(std::tuple<T, Ts...> const &t,
                                   std::index_sequence<Idx...>) ->
    typename T::value_type_r {
    return (
        ((static_cast<
              std::remove_cvref_t<decltype(std::get<Idx>(t))>::value_type_r>(
              std::get<Idx>(t).access) &
          static_cast<
              std::remove_cvref_t<decltype(std::get<Idx>(t))>::value_type_r>(
              access_type::R))
             ? std::get<Idx>(t).mask
             : 0) |
        ...);
};

template <typename reg>
constexpr typename reg::value_type get_rmw_mask(reg const &r) {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return get_rwm_mask_helper(tup, std::make_index_sequence<tup_size>{});
}

template <typename Tuple, std::size_t... Idx>
constexpr auto get_write_mask_helper(Tuple const &tup,
                                     std::index_sequence<Idx...>) {
    return (std::tuple_element_t<Idx, Tuple>::type::mask | ...);
};

template <typename T> constexpr T get_write_mask(std::tuple<> const &tup) {
    return 0;
}

template <typename T, typename... Ts>
constexpr T get_write_mask(std::tuple<Ts...> const &tup) {
    return get_write_mask_helper(tup,
                                 std::make_index_sequence<sizeof...(Ts)>{});
}

// register operations safety
template <typename Tup, std::size_t... Idx>
constexpr bool check_rw_fields_helper(access_type at, Tup const &t,
                                      std::index_sequence<Idx...>) {
    return (
        ((static_cast<uint8_t>(std::tuple_element_t<Idx, Tup>::access) &
          static_cast<uint8_t>(access_type::RW)) == static_cast<uint8_t>(at)) |
        ...);
};

template <typename reg> constexpr bool check_wo_fields(reg const &r) {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return check_rw_fields_helper(access_type::WO, tup,
                                  std::make_index_sequence<tup_size>{});
}

template <typename reg> constexpr bool check_ro_fields(reg const &r) {
    auto tup = reflect::to_tuple(r);
    constexpr std::size_t tup_size = std::tuple_size_v<decltype(tup)>;
    return check_rw_fields_helper(access_type::RO, tup,
                                  std::make_index_sequence<tup_size>{});
}

} // namespace detail

template <typename reg_derived, detail::register_type T, detail::addr addr,
          typename bus_t>
struct reg {
    using reg_der = reg_derived;
    using value_type = T;
    using bus = bus_t;
    using address = std::integral_constant<std::size_t, addr.value>;

    constexpr static value_type layout = detail::get_rmw_mask(reg_der{});
    constexpr static bool has_wo_field = detail::check_wo_fields(reg_der{});
    constexpr static bool has_ro_field = detail::check_ro_fields(reg_der{});

    constexpr static ros::detail::unsafe_register_operations_handler<reg>
        unsafe{};
    constexpr static ros::detail::safe_register_operations_handler<reg> self{};
};

namespace detail {
// specialization for is_reg_v from concepts.hpp
template <typename reg_derived, detail::register_type T, detail::addr addr,
          typename bus_t>
constexpr bool is_reg_v<reg<reg_derived, T, addr, bus_t>> = true;
} // namespace detail

} // namespace ros