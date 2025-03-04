#pragma once

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

    constexpr field_assignment_rt(value_type v) : value{v} {}

    value_type value;
};

template <typename F, typename FieldOp, typename Field0, typename... Fields>
struct field_assignment_invocable : field_assignment<FieldOp> {
    using fields = std::tuple<Field0, Fields...>;

    field_assignment_invocable(F f) : lambda_{f} {}

    constexpr auto operator()(typename Field0::value_type f0,
                              typename Fields::value_type... fs) ->
        typename FieldOp::value_type {
        return lambda_(f0, fs...);
    }

    F lambda_;
};

template <typename Field> struct field_read {
    using type = Field;
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

    constexpr register_assignment_rt(value_type v) : value{v} {}

    value_type value;
};

template <typename F, typename RegisterOp, typename Register0,
          typename... Registers>
struct register_assignment_invocable : register_assignment<RegisterOp> {
    using registerOp = RegisterOp;
    using registers = std::tuple<Register0, Registers...>;

    register_assignment_invocable(F f) : lambda_{f} {}

    constexpr auto operator()(typename Register0::value_type r0,
                              typename Registers::value_type... rs) ->
        typename RegisterOp::value_type {
        return lambda_(r0, rs...);
    }

    F lambda_;
};

template <typename Register> struct register_read {
    using type = Register;
};
} // namespace detail
} // namespace ros