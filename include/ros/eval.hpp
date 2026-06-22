#pragma once

#include <ros/field.hpp>
#include <ros/filter.hpp>
#include <ros/reg.hpp>
#include <ros/type_traits.hpp>

namespace ros {

template <typename Op, typename... Ops>
    requires detail::field_constraints<Op, Ops...>
auto eval(Op op, Ops... ops) -> detail::return_reads_t<
    decltype(filter::tuple_filter<detail::is_field_read>(
        std::make_tuple(op, ops...)))> {
    using value_type = typename Op::type::value_type_r;
    using field_type = typename Op::type;
    using reg = typename Op::type::reg;
    using bus = typename reg::bus;

    constexpr value_type rmw_mask = reg::rmw_mask;

    auto operations = std::make_tuple(op, ops...);

    value_type value = reg::identity;

    // compile-time writes
    auto writes_ct =
        filter::tuple_filter<detail::is_field_assignment_ct>(operations);
    // runtime writes
    auto writes_rt =
        filter::tuple_filter<detail::is_field_assignment_rt>(operations);
    auto writes_inv =
        filter::tuple_filter<detail::is_field_assignment_invocable>(operations);
    auto reads = filter::tuple_filter<detail::is_field_read>(operations);

    []<typename... Rs>(std::tuple<Rs...>) {
        static_assert((Rs::type::readable() && ...),
                      "Cannot read non-readable field");
    }(reads);

    constexpr auto write_mask_ct =
        detail::get_write_mask<value_type>(writes_ct);
    constexpr auto write_mask_rt =
        detail::get_write_mask<value_type>(writes_rt);
    // currently captures only the assigned field, but it can be
    // extended to capture all fields that are used in the invocable
    constexpr auto write_mask_inv =
        detail::get_write_mask<value_type>(writes_inv);
    constexpr value_type write_mask =
        write_mask_ct | write_mask_rt | write_mask_inv;

    if constexpr (write_mask != 0) {
        constexpr bool is_partial_write = ((rmw_mask & write_mask) != rmw_mask);
        constexpr bool has_invocable_writes =
            std::tuple_size_v<decltype(writes_inv)> > 0;

        if constexpr (is_partial_write or has_invocable_writes) {
            value = bus::template read<value_type>(reg::address::value);
        }

        // compose monadic chain
        const auto chain
            = detail::chain(writes_inv)
            | detail::chain(writes_ct)
            | detail::chain(writes_rt)
            ;

        value = chain(value);

        // registers are not guaranteed to be idempotent, (the stored
        // value can hold value that may trigger clear/set/toggle.
        // to avoid unintended side effect, preserve the identity for
        // special fields so no side effect is triggered
        constexpr auto total_write_mask = rmw_mask | write_mask;
        value = (reg::identity & ~total_write_mask) 
              | (value & total_write_mask)
              ;

        bus::write(value, reg::address::value);
    } else /* if (return_reads) */ {
        // implicit because if there're no writes, the only possible op is read
        value = bus::template read<value_type>(reg::address::value);
    }

    auto get_read_fields =
        [&value]<typename... Ts>(std::tuple<Ts...> reads) /* -> ... */ {
            return std::make_tuple(Ts::type::to_field(value)...);
        };

    if constexpr (std::tuple_size_v<decltype(reads)> == 1) {
        // if there's only one read, just return the field value without tuple
        using read = std::tuple_element_t<0, decltype(reads)>;
        return read::type::to_field(value);
    } else {
        return get_read_fields(reads);
    }
}

template <typename Op, typename... Ops>
    requires detail::register_constraints<Op, Ops...>
auto eval(Op op, Ops... ops) -> detail::return_reads_t<
    decltype(filter::tuple_filter<detail::is_register_read>(
        std::make_tuple(op, ops...)))> {
    // 1. evaluate reads if any (may make sense to sort)
    // 2. evaluate invocable writes (doesn't make sense to sort. the point of
    // sorting
    //    is to potentially optimize bus utilization. read operations
    //    interleaved with writes will not make it possible. on the other hand,
    //    there's not enough weight on the side of keeping temporal values of
    //    the arguments until issuing all of the writes at once. That also will
    //    make writes handling more complex.)
    // 3. evaluate writes (may make sense to sort)

    // write whole reg
    // checks attempts to write RO fields

    auto operations = std::make_tuple(op, ops...);

    // compile-time writes
    auto writes_ct =
        filter::tuple_filter<detail::is_register_assignment_ct>(operations);
    // runtime writes
    auto writes_rt =
        filter::tuple_filter<detail::is_register_assignment_rt>(operations);
    auto writes_inv =
        filter::tuple_filter<detail::is_register_assignment_invocable>(
            operations);
    auto reads = filter::tuple_filter<detail::is_register_read>(operations);

    constexpr bool has_writes_ct = std::tuple_size_v<decltype(writes_ct)> > 0;
    constexpr bool has_writes_rt = std::tuple_size_v<decltype(writes_rt)> > 0;
    constexpr bool has_writes_inv = std::tuple_size_v<decltype(writes_inv)> > 0;

    // first, make all reads for old values
    //   cluster adjucent reads into separate tuples
    //   call read_bundle to each tuple
    // if there's a write and read for the same register old read
    //   value will be returned

    []<typename... Rs>(std::tuple<Rs...>) {
        static_assert((Rs::physically_readable && ...),
                      "Cannot read write-only register");
    }(reads);

    auto evaluated_reads = []<typename... Rs>(std::tuple<Rs...>) /* -> ... */ {
        return std::make_tuple(
            Rs::type::bus::template read<typename Rs::type::value_type>(
                Rs::type::address::value)...);
    }(reads);

    detail::evaluate(writes_inv);
    detail::evaluate(writes_ct);
    detail::evaluate(writes_rt);

    // if there's only one read, just return the reg value without tuple
    if constexpr (std::tuple_size_v<decltype(reads)> == 1) {
        using read = std::tuple_element_t<0, decltype(reads)>;
        return std::get<0>(evaluated_reads);
    } else {
        return evaluated_reads;
    }
}

} // namespace ros