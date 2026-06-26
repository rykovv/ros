#pragma once

#include <type_traits>

namespace ros::detail {
// available as std::to_underlying in C++23
template <typename Enum>
[[nodiscard]] constexpr auto
to_underlying(Enum e) noexcept -> std::underlying_type_t<Enum> {
    return static_cast<std::underlying_type_t<Enum>>(e);
}
} // namespace ros::detail