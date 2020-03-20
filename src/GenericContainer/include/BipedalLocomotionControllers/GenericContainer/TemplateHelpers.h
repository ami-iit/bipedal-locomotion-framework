/**
 * @file GenericContainer.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_TEMPLATEHELPERS_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_TEMPLATEHELPERS_H

#include <type_traits>
#include <string_view>

namespace BipedalLocomotionControllers {

template<typename... Ts> struct make_void { typedef void type;};
template<typename... Ts> using void_t = typename make_void<Ts...>::type;

template <typename... Args> inline void unused(Args&&...) {}

/**
 * dependent_false is a type-dependent expression that is always false. Please check
 * https://en.cppreference.com/w/cpp/language/if for further details.
 */
template <class T> struct dependent_false : std::false_type
{
};

/**
 * is_iterable is used to build a type-dependent expression that check if an element is \a iterable
 * (i.e. the element has the methods <code>T::begin()<\code> and <code>T::end()<\code>). This
 * specific implementation is used when the the object is not iterable.
 */
template <typename T, typename = void> struct is_iterable : std::false_type
{
};

/**
 * is_iterable is used to build a type-dependent expression that check if an element is \a iterable
 * (i.e. the element has the methods <code>T::begin()<\code> and <code>T::end()<\code>). This
 * specific implementation is used when the the object is iterable, indeed
 * <code>std::void_t<\endcode> is used to detect ill-formed types in SFINAE context.
 */
template <typename T>
struct is_iterable<T, void_t<decltype(std::declval<T>().begin()), decltype(std::declval<T>().end())>>
    : std::true_type
{
};

/**
 * has_square_bracket_operator is used to build a type-dependent expression that check if an element
 * has <em>square bracket operator</em> (i.e. operator[]()). This specific implementation is used
 * when the the object does not have the square bracket operator
 */
template <typename T, typename = void> struct has_square_bracket_operator : std::false_type
{
};

/**
 * has_square_bracket_operator is used to build a type-dependent expression that check if an element
 * has <em>square bracket operator</em> (i.e. operator[]()). This specific implementation is used
 * when the the object has the square bracket operator, indeed <code>std::void_t<\endcode> is used
 * to detect ill-formed types in SFINAE context.
 */
template <typename T>
struct has_square_bracket_operator<T, void_t<decltype(std::declval<T>()[std::declval<int>()])>>
    : std::true_type
{
};

/**
 * is_resizable is used to build a type-dependent expression that check if an element is \a
 * resizable (i.e. the element has the methods <code>T::resize()<\code>). This specific
 * implementation is used when the the object is not \a resizable.
 */
template <typename T, typename = void> struct is_resizable : std::false_type
{
};

/**
 * is_resizable is used to build a type-dependent expression that check if an element is \a
 * resizable (i.e. the element has the methods <code>T::resize()<\code>). This specific
 * implementation is used when the the object is not \a resizable. Indeed
 * <code>std::void_t<\endcode> is used to detect ill-formed types in SFINAE context.
 */
template <typename T>
struct is_resizable<T, void_t<decltype(std::declval<T>().resize(std::declval<int>()))>> : std::true_type
{
};

template <typename T, typename = void> struct is_data_available : std::false_type
{
};

template <typename T>
struct is_data_available<T, void_t<decltype(std::declval<T>().data())>> : std::true_type
{
};

template< class, typename = void >
struct has_type_member : std::false_type { };

template< class T >
struct has_type_member<T, void_t<typename T::value_type>> : std::true_type { };

template <typename T, typename = void>
struct vector_data
{
    static_assert(dependent_false<T>::value, "Unable to detect type of data in the vector.");
};

template <typename T>
struct vector_data<T, typename std::enable_if<has_type_member<T>::value>::type>
{
    using type = typename T::value_type;
};

template <typename T>
struct vector_data<T, typename std::enable_if<!has_type_member<T>::value && is_data_available<T>::value>::type>
{
    using type = typename std::remove_pointer<decltype(std::declval<T>().data())>::type;
};

template <typename T>
struct vector_data<T, typename std::enable_if<std::is_array<T>::value>::type>
{
    using type = typename std::remove_all_extents_t<T>;
};

template <typename T, typename = void> struct is_size_available : std::false_type
{
};

template <typename T>
struct is_size_available<T, void_t<decltype(std::declval<T>().size())>> : std::true_type
{
};

template <typename T, typename = void>
struct size_type
{
    using type = std::ptrdiff_t;
};

template <typename T>
struct size_type<T, typename std::enable_if<is_size_available<T>::value>::type>
{
    using type = decltype(std::declval<T>().size());
};

}

//Taken from https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c
template <typename T>
constexpr std::string_view
type_name()
{
    std::string_view name, prefix, suffix;
#ifdef __clang__
    name = __PRETTY_FUNCTION__;
    prefix = "std::string_view type_name() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
    name = __PRETTY_FUNCTION__;
    prefix = "constexpr std::string_view type_name() [with T = ";
    suffix = "; std::string_view = std::basic_string_view<char>]";
#elif defined(_MSC_VER)
    name = __FUNCSIG__;
    prefix = "class std::basic_string_view<char,struct std::char_traits<char> > __cdecl type_name<";
    suffix = ">(void)";
#endif
    name.remove_prefix(prefix.size());
    name.remove_suffix(suffix.size());
    return name;
}

#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_TEMPLATEHELPERS_H
