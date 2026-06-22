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
    using value_type = typename Field::value_type;
    using value_type_r = typename Field::value_type_r;

    constexpr auto operator() (const value_type_r register_value) const
    -> value_type_r {
        return Field::to_reg(register_value, value);
    }

private:
    constexpr static value_type value = val;
};

template <typename Field> struct field_assignment_rt : field_assignment<Field> {
    using value_type = typename Field::value_type;
    using value_type_r = typename Field::value_type_r;

    constexpr explicit field_assignment_rt(const value_type v) : value{v} {}
    
    constexpr auto operator() (const value_type_r register_value) const
    -> value_type_r {
        return Field::to_reg(register_value, value);
    }

private:
    value_type value;
};

template <typename F, typename FieldOp, typename Field0, typename... Fields>
struct field_assignment_invocable : field_assignment<FieldOp> {
    using value_type = typename FieldOp::value_type;
    using value_type_r = typename FieldOp::value_type_r;
    using fields = std::tuple<Field0, Fields...>;
    
    constexpr explicit field_assignment_invocable(F f) : lambda_{f} {}
    
    constexpr auto operator() (const value_type_r register_value) const
    -> value_type_r {
        return FieldOp::to_reg(register_value, 
            invoke(register_value, fields{}, 
                std::make_index_sequence<1 + sizeof...(Fields)>{}));
    }
            
private:
    template <typename FieldsTuple, std::size_t... Idx>
    [[nodiscard]] constexpr auto invoke(const value_type_r value,
        const FieldsTuple tup,
        const std::index_sequence<Idx...>) const
    -> value_type {
        return lambda_(
            std::tuple_element_t<Idx, FieldsTuple>::to_field(value)...);
    }

    F lambda_;
};

template <typename Field> struct unsafe_field_operations_handler {
    using value_type = typename Field::value_type;

    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(auto const &rhs) const
        -> field_assignment_rt<Field> {
        static_assert(Field::writable(), "cannot write read-only field");

        return field_assignment_rt<Field>{static_cast<value_type>(rhs)};
    }

    // NOLINTNEXTLINE(cppcoreguidelines-c-copy-assignment-signature)
    constexpr auto operator=(auto &&rhs) const -> field_assignment_rt<Field> {
        static_assert(Field::writable(), "cannot write read-only field");

        return field_assignment_rt<Field>{static_cast<value_type>(rhs)};
    }

    // TODO: add compile time unsafe operations
};

template <typename L, typename R>
struct pipe {
public:
    constexpr pipe(const L lhs, const R rhs)
      : lhs_{lhs}, rhs_{rhs}
    {}
    
    constexpr auto operator() (const auto ...vs) const {
        return call_rhs(lhs_(vs...));
    }

private:
    constexpr auto call_rhs(auto && ...vs) const {
        return rhs_(std::forward<decltype(vs)>(vs)...);
    }

    L lhs_;
    R rhs_;
};

template <typename L, typename R>
constexpr auto operator| (L l, R r) {
    return pipe<L, R>{l, r};
}
} // namespace detail

// reg operations
namespace detail {
template <typename Register> struct register_assignment {
    using type = Register;
};

template <typename Register, typename Register::value_type val>
struct register_assignment_ct : register_assignment<Register> {
    using value_type = typename Register::value_type;
    using bus = typename Register::bus;
    
    constexpr auto operator() () const -> void {
        return bus::template write<value_type>(val,
            Register::address::value);
    };
};

template <typename Register>
struct register_assignment_rt : register_assignment<Register> {
    using value_type = typename Register::value_type;
    using bus = typename Register::bus;

    constexpr explicit register_assignment_rt(const value_type v) : value_{v} {}

    constexpr auto operator() () const -> void {
        return bus::template write<value_type>(value_,
            Register::address::value);
    };

private:
    value_type value_;
};

template <typename F, typename RegisterOp, typename Register0,
          typename... Registers>
struct register_assignment_invocable : register_assignment<RegisterOp> {
    using value_type = typename RegisterOp::value_type;
    using bus = typename RegisterOp::bus;
    using registerOp = RegisterOp;
    using registers = std::tuple<Register0, Registers...>;

    explicit register_assignment_invocable(F f) : lambda_{f} {}

    constexpr auto operator() () const -> void {
        bus::template write<value_type>(
            invoke(registers{}, 
                std::make_index_sequence<sizeof...(Registers) + 1>{}),
            RegisterOp::address::value);
    }

private:
    template <typename RegistersTuple, std::size_t... Idx>
    [[nodiscard]] constexpr auto invoke(const RegistersTuple tup,
                                        std::index_sequence<Idx...>) const
        -> value_type {
        return lambda_(
            bus::template read<typename std::tuple_element_t<Idx, RegistersTuple>::value_type>(
                std::tuple_element_t<Idx, RegistersTuple>::address::value)...);
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

template <typename T> struct return_reads {
    using type = void;
};

template <typename Op> struct return_reads<std::tuple<Op>> {
    using type = typename Op::type::value_type;
};

template <typename... Ops> struct return_reads<std::tuple<Ops...>> {
    using type = std::tuple<typename Ops::type::value_type...>;
};

template <typename... Ops>
using return_reads_t = typename return_reads<Ops...>::type;

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

template <typename Tuple, std::size_t... Idx>
constexpr auto get_write_mask_helper(Tuple const &tup,
                                     std::index_sequence<Idx...>) {
    return (std::tuple_element_t<Idx, Tuple>::type::mask | ...);
};

template <typename T> constexpr auto get_write_mask(std::tuple<> const &tup) -> T {
    return 0;
}

template <typename T, typename... Ts>
constexpr auto get_write_mask(std::tuple<Ts...> const &tup) -> T {
    return get_write_mask_helper(tup,
                                 std::make_index_sequence<sizeof...(Ts)>{});
}

template <typename... Ops, std::size_t... Idx>
constexpr auto chain_helper(std::tuple<Ops...> const &ops,
                            std::index_sequence<Idx...>) {
    return (std::get<Idx>(ops) | ...);
}

template <typename... Ops>
constexpr auto chain(std::tuple<Ops...> const &ops) {
    return chain_helper(ops, std::make_index_sequence<sizeof...(Ops)>{});
}

constexpr auto chain(std::tuple<> const &tup) {
    return [] (auto value) { return value; };
}

template <typename... Ops, std::size_t... Idx>
constexpr auto evaluate_helper(std::tuple<Ops...> const &ops,
                            std::index_sequence<Idx...>) {
    return (std::get<Idx>(ops)(), ...);
}

template <typename... Ops>
constexpr auto evaluate(std::tuple<Ops...> const &ops) {
    return evaluate_helper(ops, std::make_index_sequence<sizeof...(Ops)>{});
}

constexpr auto evaluate(std::tuple<> const &tup) {
    // no-op
}
} // namespace detail
} // namespace ros