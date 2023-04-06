/**
 * @file NamedTuple.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

// the original version of the code can be found https://github.com/GiulioRomualdi/named_tuple

#ifndef BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_NAMED_TUPLE_H
#define BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_NAMED_TUPLE_H

#include <cstdint>
#include <tuple>
#include <type_traits>
#include <utility>

namespace BipedalLocomotion
{

namespace GenericContainer
{

using hash_type = std::uint64_t;

namespace detail
{

// The following code has been taken from https://gist.github.com/Manu343726/081512c43814d098fe4b
// it allows to generate a compile time string hash
constexpr hash_type fnv_basis = 14695981039346656037ull;
constexpr hash_type fnv_prime = 109951162821ull;

// FNV-1a 64 bit hash
constexpr hash_type sid_hash(const char* str, hash_type hash = fnv_basis) noexcept
{
    return *str ? sid_hash(str + 1, (hash ^ *str) * fnv_prime) : hash;
}

} // namespace detail

/**
 * named_param is a struct that associate a compile time hash to a value.
 * @note Type Ts must be default constructible.
 */
template <hash_type hash_, typename Type_> struct named_param
{
    static constexpr hash_type hash = hash_;
    using Type = Type_;

    Type param; /**< Parameter associated to the given hash */

    named_param() = default;

    template <typename OtherType>
    named_param(const OtherType& param_)
        : param(param_){};

    template <typename OtherType> named_param& operator=(const OtherType& param_)
    {
        param = param_;
        return *this;
    }
};

/**
 * named_tuple is a class that inherits from tuple. Its main purpose is to make more readable and
 * less error prone code that uses `std::tuple`.
 * \code{.cpp}
 * using namespace BipedalLocomotion::GenericContainer::literals;
 *
 * auto foo =
 * BipedalLocomotion::GenericContainer::make_named_tuple(BipedalLocomotion::GenericContainer::named_param<"name"_h, std::string>(),
 *                                                       BipedalLocomotion::GenericContainer::named_param<"number"_h, int>());
 *
 * // it is still possible to access the elements of the named_tuple with structured binding
 * // declaration
 * auto& [a, b] = foo;
 * b = 150;
 *
 * // moreover you can call std::get<> as usual
 * int n = std::get<1>(foo);
 *
 * // Differently from a normal tuple it is possible to access to the element via the hash as
 * // follows
 * foo.get_from_hash<"name"_h>() = "Smith";
 * const std::string& temp_string = foo.get_from_hash<"name"_h>();
 * \endcode
 */
template <typename... Params> struct named_tuple : public std::tuple<Params...>
{

    /** Underlying tuple that can be generate from this named_tuple */
    using underlying_tuple = std::tuple<typename Params::Type...>;

    /**
     * Constructor
     */
    constexpr named_tuple()
        : std::tuple<Params...>()
    {
    }

    /**
     * Constructor
     */
    named_tuple(Params&&... args)
        : std::tuple<Params...>(std::forward<Params>(args)...)
    {
    }

    /**
     * Construct from a tuple.
     * @param t a tuple having the same number of elements of of named_tuple.
     * @note This constructor requires the type of the elements of the tuple to be convertible into
     * Params.
     */
    template <typename... Args>
    named_tuple(const std::tuple<Args...>& t)
        : std::tuple<Params...>(t)
    {
        static_assert(sizeof...(Params) == sizeof...(Args),
                      "Invalid number of parameters in the std::tuple");
    }

    /**
     * Construct from a tuple.
     * @param t a tuple having the same number of elements of named_tuple.
     * @note This constructor requires the type of the elements of the tuple to be convertible into
     * Params.
     */
    template <typename... Args>
    named_tuple(std::tuple<Args...>&& t)
        : std::tuple<Params...>(std::forward<std::tuple<Args...>>(t))
    {
        static_assert(sizeof...(Params) == sizeof...(Args),
                      "Invalid number of parameters in the std::tuple");
    }

    /**
     * Copy from a tuple.
     * @param t tuple
     */
    template <typename... Args> named_tuple& operator=(const std::tuple<Args...>& t)
    {
        static_assert(sizeof...(Params) == sizeof...(Args),
                      "Invalid number of parameters in the std::tuple");
        this->copy_from_tuple(t);
        return *this;
    }

    /**
     * Extracts the Ith element from the named_tuple. It must be an integer value in `[0,
     * sizeof...(Types))`.
     * @param t the named_tuple
     * @return A reference to the selected element of t.
     */
    template <std::size_t I>
    const typename std::tuple_element<I, underlying_tuple>::type& get() const noexcept
    {
        const auto& param = (std::get<I>(static_cast<const std::tuple<Params...>&>(*this)));
        return param.param;
    }

    /**
     * Extracts the Ith element from the named_tuple. I must be an integer value in `[0,
     * sizeof...(Types))`.
     * @param t the named_tuple
     * @return A reference to the selected element of t.
     */
    template <std::size_t I> typename std::tuple_element<I, underlying_tuple>::type& get() noexcept
    {
        auto& param = (std::get<I>(static_cast<std::tuple<Params...>&>(*this)));
        return param.param;
    }

    /**
     * Extracts the  element from the named_tuple associated to the hash `Hash`. `Hash` must be
     * among one of the hash values associated to the tuple when created.
     * @@return A reference to the selected element of t.
     */
    template <hash_type Hash> const auto& get_from_hash() const noexcept
    {
        constexpr std::size_t index = get_element_index<Hash>();
        static_assert((index != error), "Wrong named tuple key.");
        return this->get<index>();
    }

    /**
     * Extracts the  element from the named_tuple associated to the hash `Hash`. `Hash` must be
     * among one of the hash values associated to the tuple when created.
     * @@return A reference to the selected element of t.
     */
    template <hash_type Hash> auto& get_from_hash() noexcept
    {
        constexpr std::size_t index = get_element_index<Hash>();
        static_assert((index != error), "Wrong named tuple key.");
        return this->get<index>();
    }

    /**
     * Return the associated std::tuple
     * @return the tuple associated to the named_tuple
     */
    underlying_tuple to_tuple() const noexcept
    {
        underlying_tuple temp;
        this->copy_to_tuple(temp);
        return temp;
    }

private:
    /**
     * Copy an element of a named_tuple in tuple.
     * @param t a tuple
     * @note This ends the compile time recursive function
     */
    template <std::size_t I = 0, typename... Args>
    inline typename std::enable_if<I == sizeof...(Params), void>::type
    copy_to_tuple(underlying_tuple& t) const
    {
    }

    /**
     * Copy an element of a named_tuple in tuple.
     * @param t a tuple
     * @note This is a compile time recursive function
     */
    template <std::size_t I = 0, typename... Args>
    inline typename std::enable_if<(I < sizeof...(Params)), void>::type
    copy_to_tuple(underlying_tuple& t) const
    {
        // copy each element
        std::get<I>(t) = this->get<I>();
        this->copy_to_tuple<I + 1>(t);
    }

    /**
     * Copy an element of a tuple in named_tuple.
     * @param t a tuple
     * @note This ends the compile time recursive function
     */
    template <std::size_t I = 0, typename... Args>
    inline typename std::enable_if<I == sizeof...(Params), void>::type
    copy_from_tuple(const std::tuple<Args...>& t)
    {
    }

    /**
     * Copy an element of a tuple in named_tuple.
     * @param t a tuple
     * @note This is a compile time recursive function
     */
    template <std::size_t I = 0, typename... Args>
    inline typename std::enable_if<(I < sizeof...(Params)), void>::type
    copy_from_tuple(const std::tuple<Args...>& t)
    {
        // copy each element
        this->get<I>() = std::get<I>(t);
        this->copy_from_tuple<I + 1>(t);
    }

    /**
     * Get the index of the element given the hash.
     * @return an error flag.
     * @note In this function is returned means that the hash has not be found.
     */
    template <hash_type Hash, std::size_t I = 0>
    constexpr typename std::enable_if<I == sizeof...(Params),
                                      const std::size_t>::type static get_element_index() noexcept
    {
        return error;
    }

    /**
     * Get the index of the element given the hash.
     * @return The index of  the element associated to the given hash.
     * @note This function is resolved at compile time. So if the hash is not found an error flag
     * will be returned.
     */
    template <hash_type Hash, std::size_t I = 0>
    constexpr typename std::enable_if<(I < sizeof...(Params)),
                                      const std::size_t>::type static get_element_index() noexcept
    {
        using element_type = typename std::tuple_element<I, std::tuple<Params...>>::type;
        if constexpr (Hash == element_type::hash)
        {
            return I;
        }
        return get_element_index<Hash, I + 1>();
    }

    static constexpr std::size_t error = -1;
};

/**
 * Creates a named_tuple object, deducing the target type from the types of arguments.
 * @params args zero or more arguments to construct the named_tuple from
 * @return A named_tuple object containing the given values, created as if by
 * `named_tuple<Args...>(std::forward<Args>(args)...).`
 */
template <typename... Args> constexpr auto make_named_tuple(Args&&... args)
{
    return named_tuple<Args...>(std::forward<Args>(args)...);
}

namespace literals
{

/**
 * Forms a hash from a char*
 * @note To use the literal operator please refer to the following code
 * \code{.cpp}
 * using namespace BipedalLocomotion::GenericContainer::literals;
 *
 * constexpr BipedalLocomotion::GenericContainer::hash_type hash = "foo"_h;
 * \endcode
 */
constexpr hash_type operator"" _h(const char* tag, std::size_t s)
{
    return ::BipedalLocomotion::GenericContainer::detail::sid_hash(tag);
}

} // namespace literals
} // namespace GenericContainer
} // namespace BipedalLocomotion

namespace std
{

/**
 * Extracts the Ith element from the named_tuple. It must be an integer value in `[0,
 * sizeof...(Types))`.
 * @param t the named_tuple
 * @return A reference to the selected element of t.
 */
template <std::size_t I, typename... Params>
typename std::tuple_element<
    I,
    typename ::BipedalLocomotion::GenericContainer::named_tuple<Params...>::underlying_tuple>::type&
get(::BipedalLocomotion::GenericContainer::named_tuple<Params...>& t) noexcept
{
    return t.template get<I>();
}

/**
 * Extracts the Ith element from the named_tuple. It must be an integer value in `[0,
 * sizeof...(Types))`.
 * @param t the named_tuple
 * @return A reference to the selected element of t.
 */
template <std::size_t I, typename... Params>
const typename tuple_element<
    I,
    typename ::BipedalLocomotion::GenericContainer::named_tuple<Params...>::underlying_tuple>::type&
get(const ::BipedalLocomotion::GenericContainer::named_tuple<Params...>& t) noexcept
{
    return t.template get<I>();
}

/**
 * Template specialization to make the elements of the named_tuple accessible with the Structured
 * binding declaration
 */
template <typename... Params>
struct tuple_size<::BipedalLocomotion::GenericContainer::named_tuple<Params...>>
    : integral_constant<size_t, sizeof...(Params)>
{
};

/**
 * Template specialization to make the elements of the named_tuple accessible with the Structured
 * binding declaration
 */
template <size_t Index, typename... Params>
struct tuple_element<Index, ::BipedalLocomotion::GenericContainer::named_tuple<Params...>>
    : std::tuple_element<
          Index,
          typename ::BipedalLocomotion::GenericContainer::named_tuple<Params...>::underlying_tuple>
{
};

} // namespace std

/**
 * Create the hash from a string
 * @param x_ key
 */
#define BLF_STRING_TO_HASH(x_) ::BipedalLocomotion::GenericContainer::detail::sid_hash(#x_)

/**
 * Create a named parameter given type and key
 * @param x_ key
 * @param type_ type of the parameter
 */
#define BLF_NAMED_PARAM(x_, type_) \
    ::BipedalLocomotion::GenericContainer::named_param<BLF_STRING_TO_HASH(x_), type_>

#endif // BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_NAMED_TUPLE_H
