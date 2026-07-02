#pragma once

#include <type_traits>

#include <boost/mp11.hpp>

namespace ros::detail {
// available as std::to_underlying in C++23
template <typename Enum>
[[nodiscard]] constexpr auto
to_underlying(Enum e) noexcept -> std::underlying_type_t<Enum> {
    return static_cast<std::underlying_type_t<Enum>>(e);
}

template <template <typename> typename Predicate, typename... Ts>
auto filter(std::tuple<Ts...> const &t) 
    -> boost::mp11::mp_filter<Predicate, std::tuple<Ts...>> 
{
    return std::apply(
        []<typename... Rs>(Rs const&... args) {
            return std::tuple_cat(
                [&]<typename R>(R const& arg) {
                    if constexpr (Predicate<R>::value) {
                        return std::make_tuple(arg);
                    } else {
                        return std::tuple<>{};
                    }
                }(args)...);
            },
        t);
}
} // namespace ros::detail