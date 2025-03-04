#pragma once

namespace ros {
namespace reflect {

struct universal_type {
    template <typename T> operator T() {}
};

template <typename T> consteval auto get_struct_size(auto... members) {
    if constexpr (requires { T{members...}; } == false) {
        return sizeof...(members) - 2; // self and unsafe members
    } else {
        return get_struct_size<T>(members..., universal_type{});
    }
}

template <typename T> constexpr auto forward(T &&t) {
    return std::forward<T>(t);
}

template <typename T, unsigned S> struct to_tuple_helper;

template <typename T> struct to_tuple_helper<T, 0> {
    constexpr auto operator()(T const &t) const { return std::make_tuple(); }
};

template <typename T> struct to_tuple_helper<T, 1> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0] = forward(t);
        return std::make_tuple(f0);
    }
};

template <typename T> struct to_tuple_helper<T, 2> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1] = forward(t);
        return std::make_tuple(f0, f1);
    }
};

template <typename T> struct to_tuple_helper<T, 3> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2] = forward(t);
        return std::make_tuple(f0, f1, f2);
    }
};

template <typename T> struct to_tuple_helper<T, 4> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3] = forward(t);
        return std::make_tuple(f0, f1, f2, f3);
    }
};

template <typename T> struct to_tuple_helper<T, 5> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4);
    }
};

template <typename T> struct to_tuple_helper<T, 6> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5);
    }
};

template <typename T> struct to_tuple_helper<T, 7> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6);
    }
};

template <typename T> struct to_tuple_helper<T, 8> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7);
    }
};

template <typename T> struct to_tuple_helper<T, 9> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8);
    }
};

template <typename T> struct to_tuple_helper<T, 10> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9);
    }
};

template <typename T> struct to_tuple_helper<T, 11> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10);
    }
};

template <typename T> struct to_tuple_helper<T, 12> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10,
                               f11);
    }
};

template <typename T> struct to_tuple_helper<T, 13> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12] =
            forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12);
    }
};

template <typename T> struct to_tuple_helper<T, 14> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13] =
            forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13);
    }
};

template <typename T> struct to_tuple_helper<T, 15> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13,
                f14] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14);
    }
};

template <typename T> struct to_tuple_helper<T, 16> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15);
    }
};

template <typename T> struct to_tuple_helper<T, 17> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16);
    }
};

template <typename T> struct to_tuple_helper<T, 18> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17);
    }
};

template <typename T> struct to_tuple_helper<T, 19> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18);
    }
};

template <typename T> struct to_tuple_helper<T, 20> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19);
    }
};

template <typename T> struct to_tuple_helper<T, 21> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20);
    }
};

template <typename T> struct to_tuple_helper<T, 22> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20,
                               f21);
    }
};

template <typename T> struct to_tuple_helper<T, 23> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22);
    }
};

template <typename T> struct to_tuple_helper<T, 24> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23);
    }
};

template <typename T> struct to_tuple_helper<T, 25> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24);
    }
};

template <typename T> struct to_tuple_helper<T, 26> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25] =
            forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25);
    }
};

template <typename T> struct to_tuple_helper<T, 27> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26] =
            forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26);
    }
};

template <typename T> struct to_tuple_helper<T, 28> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26,
                f27] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27);
    }
};

template <typename T> struct to_tuple_helper<T, 29> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28);
    }
};

template <typename T> struct to_tuple_helper<T, 30> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29);
    }
};

template <typename T> struct to_tuple_helper<T, 31> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30);
    }
};

template <typename T> struct to_tuple_helper<T, 32> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30,
                               f31);
    }
};

template <typename T> struct to_tuple_helper<T, 33> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32);
    }
};

template <typename T> struct to_tuple_helper<T, 34> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33);
    }
};

template <typename T> struct to_tuple_helper<T, 35> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34);
    }
};

template <typename T> struct to_tuple_helper<T, 36> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35);
    }
};

template <typename T> struct to_tuple_helper<T, 37> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36);
    }
};

template <typename T> struct to_tuple_helper<T, 38> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37);
    }
};

template <typename T> struct to_tuple_helper<T, 39> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38] =
            forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38);
    }
};

template <typename T> struct to_tuple_helper<T, 40> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39] =
            forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39);
    }
};

template <typename T> struct to_tuple_helper<T, 41> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39,
                f40] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40);
    }
};

template <typename T> struct to_tuple_helper<T, 42> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40,
                               f41);
    }
};

template <typename T> struct to_tuple_helper<T, 43> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42);
    }
};

template <typename T> struct to_tuple_helper<T, 44> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43);
    }
};

template <typename T> struct to_tuple_helper<T, 45> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44);
    }
};

template <typename T> struct to_tuple_helper<T, 46> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45);
    }
};

template <typename T> struct to_tuple_helper<T, 47> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46);
    }
};

template <typename T> struct to_tuple_helper<T, 48> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47);
    }
};

template <typename T> struct to_tuple_helper<T, 49> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48);
    }
};

template <typename T> struct to_tuple_helper<T, 50> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49);
    }
};

template <typename T> struct to_tuple_helper<T, 51> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50);
    }
};

template <typename T> struct to_tuple_helper<T, 52> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51] =
            forward(t);
        return std::make_tuple(
            f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
            f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
            f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
            f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51);
    }
};

template <typename T> struct to_tuple_helper<T, 53> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52] =
            forward(t);
        return std::make_tuple(
            f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
            f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
            f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
            f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52);
    }
};

template <typename T> struct to_tuple_helper<T, 54> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52,
                f53] = forward(t);
        return std::make_tuple(
            f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
            f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
            f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
            f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53);
    }
};

template <typename T> struct to_tuple_helper<T, 55> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54);
    }
};

template <typename T> struct to_tuple_helper<T, 56> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54, f55);
    }
};

template <typename T> struct to_tuple_helper<T, 57> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54, f55, f56);
    }
};

template <typename T> struct to_tuple_helper<T, 58> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54, f55, f56, f57);
    }
};

template <typename T> struct to_tuple_helper<T, 59> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57, f58] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54, f55, f56, f57, f58);
    }
};

template <typename T> struct to_tuple_helper<T, 60> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57, f58, f59] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54, f55, f56, f57, f58, f59);
    }
};

template <typename T> struct to_tuple_helper<T, 61> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57, f58, f59, f60] = forward(t);
        return std::make_tuple(f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11,
                               f12, f13, f14, f15, f16, f17, f18, f19, f20, f21,
                               f22, f23, f24, f25, f26, f27, f28, f29, f30, f31,
                               f32, f33, f34, f35, f36, f37, f38, f39, f40, f41,
                               f42, f43, f44, f45, f46, f47, f48, f49, f50, f51,
                               f52, f53, f54, f55, f56, f57, f58, f59, f60);
    }
};

template <typename T> struct to_tuple_helper<T, 62> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57, f58, f59, f60, f61] = forward(t);
        return std::make_tuple(
            f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
            f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
            f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
            f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
            f54, f55, f56, f57, f58, f59, f60, f61);
    }
};

template <typename T> struct to_tuple_helper<T, 63> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57, f58, f59, f60, f61, f62] = forward(t);
        return std::make_tuple(
            f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
            f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
            f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
            f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
            f54, f55, f56, f57, f58, f59, f60, f61, f62);
    }
};

template <typename T> struct to_tuple_helper<T, 64> {
    constexpr auto operator()(T const &t) const {
        auto &&[f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
                f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
                f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
                f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
                f54, f55, f56, f57, f58, f59, f60, f61, f62, f63] = forward(t);
        return std::make_tuple(
            f0, f1, f2, f3, f4, f5, f6, f7, f8, f9, f10, f11, f12, f13, f14,
            f15, f16, f17, f18, f19, f20, f21, f22, f23, f24, f25, f26, f27,
            f28, f29, f30, f31, f32, f33, f34, f35, f36, f37, f38, f39, f40,
            f41, f42, f43, f44, f45, f46, f47, f48, f49, f50, f51, f52, f53,
            f54, f55, f56, f57, f58, f59, f60, f61, f62, f63);
    }
};

template <typename T> constexpr auto to_tuple(T const &t) {
    constexpr std::size_t ss = get_struct_size<T>();
    return to_tuple_helper<T, ss>{}(t);
}

} // namespace reflect
} // namespace ros