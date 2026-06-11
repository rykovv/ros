#pragma once

#include <type_traits>

#include <ros/access.hpp>
#include <ros/concepts.hpp>
#include <ros/error.hpp>
#include <ros/literals.hpp>
#include <ros/operations.hpp>

namespace ros {

namespace utils {
template <typename Enum>
[[nodiscard]] constexpr auto
to_underlying(Enum e) noexcept -> std::underlying_type_t<Enum> {
    return static_cast<std::underlying_type_t<Enum>>(e);
}
} // namespace utils

namespace detail {

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

template <typename reg_derived, detail::msb msb, detail::lsb lsb,
          access_type at,
          detail::field_type value_type_f = typename reg_derived::value_type>
    requires detail::field_selectable<value_type_f, msb, lsb>
struct field {
    using value_type_r = typename reg_derived::value_type;
    using value_type = value_type_f;
    using reg = reg_derived;
    using type = field<reg_derived, msb, lsb, at, value_type_f>;

    constexpr static access_type access = at;

    constexpr static uint8_t length = []() {
        return msb.value == lsb.value ? 1 : msb.value - lsb.value + 1;
    }();

    constexpr static value_type_r mask = []() -> value_type_r {
        if (msb.value != lsb.value) {
            if constexpr (msb.value ==
                          std::numeric_limits<value_type_r>::digits - 1) {
                return static_cast<value_type_r>(~((1u << lsb.value) - 1));
            } else {
                return static_cast<value_type_r>(((1u << (msb.value + 1)) - 1) &
                                                 ~((1u << lsb.value) - 1));
            }
        } else {
            return static_cast<value_type_r>(1u << msb.value);
        }
    }();

    constexpr static value_type_r rmw_mask = []() {
        if constexpr (at == access_type::RW) {
            return mask;
        } else {
            return 0;
        };
    }();

    constexpr static value_type_r write_mask = []() {
        if constexpr (at == access_type::WO || at == access_type::RW ||
                      at == access_type::RW_0C || at == access_type::RW_1C ||
                      at == access_type::RW_0S || at == access_type::RW_1S ||
                      at == access_type::RW_1T || at == access_type::RW_O) {
            return mask;
        } else {
            return 0;
        }
    }();

    constexpr static value_type_r read_mask = []() {
        if constexpr (at == access_type::RO || at == access_type::RW ||
                      at == access_type::RC || at == access_type::RS) {
            return mask;
        } else {
            return 0;
        }
    }();

    constexpr field() = default;

    template <typename U, U val>
        requires(std::is_convertible_v<U, value_type>)
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(detail::field_value<U, val>) const
        -> detail::field_assignment_ct<field, val> {
        static_assert(writable(), "Cannot write non-writable field");
        static_assert(val <= (mask >> lsb.value),
                      "Assigned value greater than the field length");

        return ros::detail::field_assignment_ct<field, val>{};
    }

    // constexpr auto operator= (field_type auto val) const ->
    // detail::field_assignment_rt<field> {
    //     static_assert(access_type != access_type::RO, "Cannot write
    //     non-writable field"); static_assert(val <= (mask >> lsb.value),
    //     "Assigned value greater than the field length"); return
    //     detail::field_assignment_rt<field>{val};
    // }

    // field-to-field assignment needs elaboration
    // constexpr auto operator= (auto const& rhs) ->
    // detail::field_assignment_ct<field, decltype(rhs)::value> {
    //     static_assert(access_type != access_type::RO, "Cannot write
    //     non-writable field"); using rhs_type =
    //     std::remove_reference_t<decltype(rhs)>;
    //     static_assert(rhs_type::length <= msb - lsb, "larger field cannot be
    //     safely assigned to a narrower one"); return
    //     detail::field_assignment_ct<field, rhs.value>{};
    // }

    // constexpr auto operator= (auto && rhs) ->
    // detail::field_assignment_ct<field, decltype(rhs)::value> {
    //     static_assert(access_type != access_type::RO, "Cannot write
    //     non-writable field"); using rhs_type =
    //     std::remove_reference_t<decltype(rhs)>;
    //     static_assert(rhs_type::length <= msb - lsb, "larger field cannot be
    //     safely assigned to a narrower one"); return
    //     detail::field_assignment_ct<field, rhs.value>{};
    // }

    // [TODO] create concept
    template <typename T>
        requires(std::unsigned_integral<T> &&
                 std::is_convertible_v<T, value_type> &&
                 std::numeric_limits<T>::digits >= msb.value - lsb.value)
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(T const &rhs) const
        -> detail::field_assignment_rt<field> {
        static_assert(writable(), "Cannot write non-writable field");
        static_assert(std::numeric_limits<value_type>::digits >=
                          std::numeric_limits<T>::digits,
                      "Assigned value type is wider than the base field type");

        return detail::field_assignment_rt<field>{runtime_check(rhs)};
    }

    template <typename T>
        requires(std::unsigned_integral<T> &&
                 std::is_convertible_v<T, value_type> &&
                 std::numeric_limits<T>::digits >= msb.value - lsb.value)
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(T &&rhs) const
        -> detail::field_assignment_rt<field> {
        static_assert(writable(), "Cannot write non-writable field");
        static_assert(std::numeric_limits<value_type>::digits >=
                          std::numeric_limits<T>::digits,
                      "Assigned value type is wider than the base field type");

        return detail::field_assignment_rt<field>{runtime_check(rhs)};
    }

    template <typename EnumT>
        requires(std::is_enum_v<EnumT>)
    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(EnumT val) const
        -> detail::field_assignment_rt<field> {
        static_assert(writable(), "Cannot write non-writable field");
        static_assert(
            std::numeric_limits<value_type_r>::digits >=
                std::numeric_limits<std::underlying_type_t<EnumT>>::digits,
            "Underling enum type is wider than the base field type");

        return detail::field_assignment_rt<field>{runtime_check(val)};
    }

    constexpr auto operator()(std::invocable<value_type> auto f) const
        -> detail::field_assignment_invocable<decltype(f), field, field> {
        static_assert(readwritable(),
                      "Invocable write requires RW field access_type");

        return detail::field_assignment_invocable<decltype(f), field, field>{f};
    }

    template <typename F, typename Field0, typename... Fields>
        requires std::invocable<F, typename Field0::value_type,
                                typename Fields::value_type...>
    constexpr auto operator()(F f, Field0 f0, Fields... fs) const
        -> detail::field_assignment_invocable<F, field, Field0, Fields...> {
        static_assert(readwritable(),
                      "Invocable write requires RW field access_type");

        return detail::field_assignment_invocable<F, field, Field0, Fields...>{
            f};
    }

    constexpr static auto to_reg(value_type_r reg_value, value_type value) 
        -> value_type_r {
        return (reg_value & ~mask) |
               (static_cast<value_type_r>(value) << lsb.value) & mask;
    }

    constexpr static auto to_field(value_type_r value) -> value_type {
        // this can be safely ignored because the value cannot be a signed int, 
        // see field_selectable concept.
        // NOLINTNEXTLINE(hicpp-signed-bitwise)
        return (value & mask) >> lsb.value;
    }

    constexpr static auto runtime_check(value_type value) -> value_type {
        static_assert(writable(), "Cannot write non-writable field");

        value_type safe_val;
        if (static_cast<value_type_r>(value) <= mask >> lsb.value) {
            safe_val = value;
        } else {
            safe_val = error::handle_field_error<field>(value);
        }

        return safe_val;
    }

    constexpr static auto writable() -> bool {
        return (utils::to_underlying(access) &
                utils::to_underlying(access_type::W)) != 0;
    }

    constexpr static auto readable() -> bool {
        return (utils::to_underlying(access) &
                utils::to_underlying(access_type::R)) != 0;
    }

    constexpr static auto readwritable() -> bool { return writable() && readable(); }

    constexpr static auto writeonly() -> bool { return writable() && not readable(); }

    constexpr static auto readonly() -> bool { return not writable() && readable(); }

    constexpr static auto identity() -> value_type_r {
        if constexpr (access == access_type::RW_0C ||
                      access == access_type::RW_0S) {
            return mask;
        } else {
            /* access == access_type::RW    ||
               access == access_type::RW_1C ||
               access == access_type::RW_1S ||
               access == access_type::RW_1T */
            return 0;
        }
    }

    constexpr static detail::unsafe_field_operations_handler<field> unsafe{};
};

namespace detail {
// specialization for is_reg_v from concepts.hpp
template <typename Reg, detail::msb msb, detail::lsb lsb, access_type AT,
          field_type field_t>
struct is_field<field<Reg, msb, lsb, AT, field_t>> : std::true_type {};
} // namespace detail

} // namespace ros