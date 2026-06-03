#pragma once

#include <cstddef>
#include <tuple>
#include <utility>

namespace ros {
namespace filter {

template <typename... Is> struct index_sequence_concat;

template <std::size_t... Ls>
struct index_sequence_concat<std::index_sequence<Ls...>> {
    using type = std::index_sequence<Ls...>;
};

template <std::size_t... Ls, std::size_t... Rs>
struct index_sequence_concat<std::index_sequence<Ls...>,
                             std::index_sequence<Rs...>> {
    using type = std::index_sequence<Ls..., Rs...>;
};

template <typename Is0, typename Is1, typename... Is>
struct index_sequence_concat<Is0, Is1, Is...> {
    using type = typename index_sequence_concat<
        typename index_sequence_concat<Is0, Is1>::type, Is...>::type;
};

template <typename... Is>
using index_sequence_concat_t = typename index_sequence_concat<Is...>::type;

template <bool B, std::size_t I> struct conditional_index_sequence {
    using type = std::index_sequence<>;
};

template <std::size_t I> struct conditional_index_sequence<true, I> {
    using type = std::index_sequence<I>;
};

template <bool B, std::size_t I>
using conditional_index_sequence_t =
    typename conditional_index_sequence<B, I>::type;

template <template <typename...> typename Predicate, typename Tuple,
          std::size_t... Is>
struct filtered_index_sequence {
    using type = index_sequence_concat_t<conditional_index_sequence_t<
        Predicate<std::tuple_element_t<Is, Tuple>>::value, Is>...>;
};

template <template <typename...> typename Predicate, typename Tuple,
          std::size_t... Is>
using filtered_index_sequence_t =
    filtered_index_sequence<Predicate, Tuple, Is...>::type;

template <typename Tuple, std::size_t... Is>
constexpr auto tuple_filter_apply(Tuple const &tuple,
                                  std::index_sequence<Is...>) {
    return std::make_tuple(std::get<Is>(tuple)...);
}

template <template <typename...> typename Predicate, typename Tuple,
          std::size_t... Is>
constexpr auto tuple_filter_helper(Tuple const &tuple,
                                   std::index_sequence<Is...>) {
    return tuple_filter_apply(
        tuple, filtered_index_sequence_t<Predicate, Tuple, Is...>{});
}

template <template <typename...> typename Predicate, typename Tuple>
constexpr auto tuple_filter(Tuple const &tuple) {
    return tuple_filter_helper<Predicate>(
        tuple, std::make_index_sequence<std::tuple_size_v<Tuple>>{});
}

} // namespace filter
} // namespace ros