#pragma once

#include <ros/field.hpp>
#include <ros/filter.hpp>
#include <ros/reg.hpp>
#include <ros/type_traits.hpp>

namespace ros {
namespace detail {

template <typename T> struct return_reads {
    using type = void;
};

template <typename... Ops> struct return_reads<std::tuple<Ops...>> {
    using type = std::tuple<typename Ops::type::value_type...>;
};

template <typename... Ops>
using return_reads_t = typename return_reads<Ops...>::type;

template <typename T, typename Tuple, std::size_t... Idx>
constexpr T get_write_value_helper(Tuple tup, std::index_sequence<Idx...>) {
    return (std::tuple_element_t<Idx, Tuple>::type::to_reg(
                T{0}, std::get<Idx>(tup).value) |
            ...);
}

template <typename T, typename... Ts>
constexpr T get_write_value(T value, T mask, std::tuple<Ts...> const &tup) {
    return (value & ~mask) |
           get_write_value_helper<T>(tup,
                                     std::make_index_sequence<sizeof...(Ts)>{});
}

template <typename T>
constexpr T get_write_value(T value, T mask, std::tuple<> const &tup) {
    return value;
}

template <typename T, typename InvocableWrite, typename TupleFields,
          std::size_t... Idx>
constexpr T get_invocable_write_fields_helper(T value, InvocableWrite iw,
                                              TupleFields tup,
                                              std::index_sequence<Idx...>) {
    // get each field
    return iw(std::tuple_element_t<Idx, TupleFields>::to_field(value)...);
}

template <typename T, typename TupleInvocableWrites, std::size_t... Idx>
constexpr T get_invocable_write_value_helper(T value, TupleInvocableWrites tup,
                                             std::index_sequence<Idx...>) {
    return (
        std::tuple_element_t<
            Idx, TupleInvocableWrites>::type::to_reg( // wrap back everything to
                                                      // reg value
            T{0}, // pass in zero, final value will assigned with a compound
                  // mask
            std::tuple_element_t<
                Idx, TupleInvocableWrites>::type::runtime_check( // safety check
                get_invocable_write_fields_helper( // make invocable call with
                                                   // each field value
                    value,                         // original reg value
                    std::get<Idx>(tup),            // invocable lambda wrapper
                    typename std::tuple_element_t<
                        Idx, TupleInvocableWrites>::fields{}, // tuple of fields
                    std::make_index_sequence<
                        std::tuple_size_v<typename std::tuple_element_t<
                            Idx, TupleInvocableWrites>::fields>>{}))) |
        ...);
}

template <typename T, typename... InvocableWrites>
constexpr T
get_invocable_write_value(T value, T mask,
                          std::tuple<InvocableWrites...> const &tup) {
    return (value & ~mask) |
           get_invocable_write_value_helper(
               value, tup,
               std::make_index_sequence<sizeof...(InvocableWrites)>{});
}

template <typename T>
constexpr T get_invocable_write_value(T value, T mask,
                                      std::tuple<> const &tup) {
    return value;
}

template <typename InvocableWrite, std::size_t... Is>
constexpr void
evaluate_invocable_assignment_helper(InvocableWrite iw,
                                     std::index_sequence<Is...>) {
    using registerOp = typename InvocableWrite::registerOp;
    using busOp = registerOp::bus;
    using registers = typename InvocableWrite::registers;

    busOp::write(
        iw(
            // call bus::read that corresponds to each register
            std::tuple_element_t<Is, registers>::bus::template read<
                typename std::tuple_element_t<Is, registers>::value_type>(
                std::tuple_element_t<Is, registers>::address::value)...),
        registerOp::address::value);
}

template <typename InvocableWrite>
constexpr void evaluate_invocable_assignment(InvocableWrite iw) {
    using registers = typename InvocableWrite::registers;
    evaluate_invocable_assignment_helper(
        iw, std::make_index_sequence<std::tuple_size_v<registers>>{});
}

template <typename... InvocableWrites, std::size_t... Is>
constexpr void
evaluate_invocable_assignments_helper(std::tuple<InvocableWrites...> writes,
                                      std::index_sequence<Is...>) {
    (evaluate_invocable_assignment(std::get<Is>(writes)), ...);
}

template <typename... InvocableWrites>
constexpr void
evaluate_invocable_assignments(std::tuple<InvocableWrites...> writes) {
    evaluate_invocable_assignments_helper(
        writes, std::make_index_sequence<sizeof...(InvocableWrites)>{});
}

} // namespace detail

template <typename Op, typename... Ops>
    requires detail::field_constraints<Op, Ops...>
auto apply(Op op, Ops... ops)
    -> detail::return_reads_t<
        decltype(filter::tuple_filter<detail::is_field_read>(
            std::make_tuple(op, ops...)))> {
    using value_type = typename Op::type::value_type_r;
    using field_type = typename Op::type;
    using reg = typename Op::type::reg;
    using bus = typename reg::bus;

    constexpr value_type rmw_mask = reg::layout;

    auto operations = std::make_tuple(op, ops...);

    value_type value{};

    // compile-time writes
    auto writes_ct =
        filter::tuple_filter<detail::is_field_assignment_ct>(operations);
    // runtime writes
    auto writes_rt =
        filter::tuple_filter<detail::is_field_assignment_rt>(operations);
    auto writes_inv =
        filter::tuple_filter<detail::is_field_assignment_invocable>(operations);

    constexpr value_type write_mask_ct =
        detail::get_write_mask<value_type>(writes_ct);
    constexpr value_type write_mask_rt =
        detail::get_write_mask<value_type>(writes_rt);
    constexpr value_type write_mask_inv =
        detail::get_write_mask<value_type>(writes_inv);
    constexpr value_type write_mask =
        write_mask_ct | write_mask_rt | write_mask_inv;

    if constexpr (write_mask != 0) {
        constexpr bool is_partial_write = ((rmw_mask & write_mask) != rmw_mask);
        constexpr bool has_invocable_writes =
            std::tuple_size_v<decltype(writes_inv)> > 0;

        if constexpr (is_partial_write || has_invocable_writes) {
            value = bus::template read<value_type>(reg::address::value);
        }

        // evaluate invocables at the beginning
        // it doesn't make much sense to evaluate it at the end because it will
        // have newly assigned values. this way just literals could be provided
        // in the lambda
        value = detail::get_invocable_write_value(value, write_mask_inv,
                                                  writes_inv);

        // [TODO] study efficiency of bundling together all writes
        // compile time
        value = detail::get_write_value(value, write_mask_ct, writes_ct);
        // runtime
        value = detail::get_write_value(value, write_mask_rt, writes_rt);

        bus::write(value, reg::address::value);
    } else /* if (return_reads) */ {
        // implicit because if there're no writes, the only possible op is read
        value = bus::template read<value_type>(reg::address::value);
    }

    auto get_read_fields =
        [&value]<typename... Ts>(std::tuple<Ts...> reads) /* -> ... */ {
            return std::make_tuple(Ts::type::to_field(value)...);
        };

    return get_read_fields(
        filter::tuple_filter<detail::is_field_read>(operations));
}

template <typename Op, typename... Ops>
    requires detail::register_constraints<Op, Ops...>
auto apply(Op op, Ops... ops)
    -> detail::return_reads_t<
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
    // reads
    auto reads = filter::tuple_filter<detail::is_register_read>(operations);

    constexpr bool has_writes_ct = std::tuple_size_v<decltype(writes_ct)> > 0;
    constexpr bool has_writes_rt = std::tuple_size_v<decltype(writes_rt)> > 0;
    constexpr bool has_writes_inv = std::tuple_size_v<decltype(writes_inv)> > 0;

    // first, make all reads for old values
    //   cluster adjucent reads into separate tuples
    //   call read_bundle to each tuple
    // if there's a write and read for the same register old read
    //   value will be returned

    auto evaluate_reads = []<typename... Rs>(std::tuple<Rs...>) /* -> ... */ {
        return std::make_tuple(
            Rs::type::bus::template read<typename Rs::type::value_type>(
                Rs::type::address::value)...);
    };

    if constexpr (has_writes_inv) {
        detail::evaluate_invocable_assignments(writes_inv);
    }

    // third, cluster adjacent writes into separate tuples
    //   call write bundled for each tuple

    // writes are peformed separately. the intent is to get jem
    //   out of compile-time accessible writes, and then let
    //   the compiler to optimize the runtime writes

    if constexpr (has_writes_ct) {
        []<typename... Ws>(std::tuple<Ws...>) -> void {
            (Ws::type::bus::write(Ws::value, Ws::type::address::value), ...);
        }(writes_ct);
    }

    if constexpr (has_writes_rt) {
        []<typename... Ws>(std::tuple<Ws...> ws) -> void {
            (Ws::type::bus::write(std::get<Ws>(ws).value,
                                  Ws::type::address::value),
             ...);
        }(writes_rt);
    }

    return evaluate_reads(reads);
}

} // namespace ros