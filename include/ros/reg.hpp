#pragma once

#include <ros/access.hpp>
#include <ros/concepts.hpp>
#include <ros/error.hpp>
#include <ros/literals.hpp>
#include <ros/operations.hpp>

namespace ros {

template <typename reg_derived, detail::register_type T, detail::addr addr,
          typename bus_t>
struct reg {
    using reg_der = reg_derived;
    using value_type = T;
    using bus = bus_t;
    using address = std::integral_constant<std::size_t, addr.value>;
    using type = reg<reg_derived, T, addr, bus_t>;

    /* identity mask is relevant to RW_0C, RW_1C, RW_0S, RW_1S, RW_1T because
       they have predefined value that won't modify HW state */
    constexpr static value_type identity = detail::get_identity_mask(reg_der{});
    constexpr static value_type rmw_mask = detail::get_rmw_mask(reg_der{});
    constexpr static value_type writable_mask =
        detail::get_writable_mask(reg_der{});
    constexpr static value_type ro_mask = detail::get_ro_mask(reg_der{});
    constexpr static bool physically_readable =
        not detail::has_wo_access(reg_der{});

    template <typename U, U val>
        requires(std::is_convertible_v<U, value_type>)
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature, readability-const-return-type)
    constexpr auto operator=(detail::register_value<U, val>) const
        -> detail::register_assignment_ct<reg, val> const {
        static_assert(static_cast<value_type>(val & reg::ro_mask) == 0,
                      "Attempt to assign read-only bits");
        return detail::register_assignment_ct<reg, val>{};
    }

    template <typename U>
        requires std::integral<U> && std::is_convertible_v<U, value_type>
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(U const &rhs) const
        -> detail::register_assignment_rt<reg> {
        static_assert(std::numeric_limits<value_type>::digits >=
                          std::numeric_limits<U>::digits,
                      "Unsafe assignment. Assigned value type is too wide.");

        value_type value;
        if (rhs & reg::ro_mask) {
            value = error::handle_register_error<reg>(rhs);
        } else {
            value = rhs;
        }

        return detail::register_assignment_rt<reg>{value};
    }

    template <typename U>
        requires std::integral<U> && std::is_convertible_v<U, value_type>
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(U &&rhs) const
        -> detail::register_assignment_rt<reg> {
        static_assert(std::numeric_limits<value_type>::digits >=
                          std::numeric_limits<U>::digits,
                      "Unsafe assignment. Assigned value type is too wide.");

        value_type value;
        if (rhs & reg::ro_mask) {
            value = error::handle_register_error<reg>(rhs);
        } else {
            value = rhs;
        }

        return detail::register_assignment_rt<reg>{value};
    }

    constexpr auto operator()(std::invocable<value_type> auto f) const
        -> detail::register_assignment_invocable<decltype(f), reg, reg> {
        return detail::register_assignment_invocable<decltype(f), reg, reg>{f};
    }

    template <typename F, typename RegOpsHandlerT0, typename... RegOpsHandlerTs>
        requires std::invocable<F, typename RegOpsHandlerT0::reg::value_type,
                                typename RegOpsHandlerTs::reg::value_type...>
    constexpr auto operator()(F f, RegOpsHandlerT0 rh0,
                              RegOpsHandlerTs... rhs) const
        -> detail::register_assignment_invocable<
            F, reg, typename RegOpsHandlerT0::reg,
            typename RegOpsHandlerTs::reg...> {
        return detail::register_assignment_invocable<
            F, reg, typename RegOpsHandlerT0::reg,
            typename RegOpsHandlerTs::reg...>{f};
    }

    constexpr static ros::detail::unsafe_register_operations_handler<reg>
        unsafe{};
};

namespace detail {
// specialization for is_reg_v from concepts.hpp
// template <typename reg_derived, detail::register_type T, detail::addr addr,
//           typename bus_t>
// struct is_reg_op<reg<reg_derived, T, addr, bus_t>> : std::true_type {};
// struct is_reg<reg<reg_derived, T, addr, bus_t>> :
// std::is_base_of<reg<reg_derived, T, addr, bus_t>, reg_derived> {};
} // namespace detail

} // namespace ros