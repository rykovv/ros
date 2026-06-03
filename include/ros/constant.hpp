#pragma once

#include <array>

namespace ros {
namespace detail {

template <char Char>
constexpr bool is_decimal_digit_v = Char >= '0' && Char <= '9';

template <char Char> constexpr bool is_delimiter_v = Char == '\'';

template <char... Chars>
concept decimal_number =
    ((is_decimal_digit_v<Chars> or is_delimiter_v<Chars>) and ...);

constexpr static auto is_delimiter = [](char const c) -> bool {
    return c == '\'';
};

template <typename T, char... Chars>
    requires decimal_number<Chars...>
[[nodiscard]] static constexpr auto to_constant() -> T {
    constexpr T value = []() {
        constexpr std::array<char, sizeof...(Chars)> chars{Chars...};
        T sum = 0;

        for (char c : chars) {
            if (not is_delimiter(c)) {
                T const digit = c - '0';
                sum = (sum * 10) + digit;
            }
        }

        return sum;
    }();

    return value;
}

template <char Char> constexpr bool is_x_v = Char == 'x';

template <char Char> constexpr bool is_0_v = Char == '0';

template <char Char>
constexpr bool is_hex_char_v =
    (Char >= 'A' && Char <= 'F' || Char >= 'a' && Char <= 'f');

template <char Char>
constexpr bool is_hex_digit_v =
    is_decimal_digit_v<Char> || is_hex_char_v<Char> || is_delimiter_v<Char>;

template <char Char0, char Char1, char... Chars>
concept hex_number =
    is_0_v<Char0> && is_x_v<Char1> && (is_hex_digit_v<Chars> && ...);

template <typename T, char Char0, char Char1, char... Chars>
    requires hex_number<Char0, Char1, Chars...>
[[nodiscard]] static constexpr auto to_constant() -> T {
    constexpr T value = []() {
        constexpr std::array<char, sizeof...(Chars)> chars{Chars...};
        T sum = 0;

        for (char c : chars) {
            if (not is_delimiter(c)) {
                /* The difference between upper and lower case in ascii table is
                 * the presence of the 6th bit */
                T const digit =
                    c <= '9' ? c - '0' : (c & 0b1101'1111) - 'A' + 10;
                sum = (sum * 16) + digit;
            }
        }

        return sum;
    }();

    return value;
}

} // namespace detail
} // namespace ros