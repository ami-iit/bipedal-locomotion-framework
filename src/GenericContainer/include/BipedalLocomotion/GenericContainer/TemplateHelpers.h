/**
 * @file GenericContainer.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_TEMPLATEHELPERS_H
#define BIPEDAL_LOCOMOTION_TEMPLATEHELPERS_H

#include <type_traits>
#include <string_view>

//Eigen (for toEigen() methods)
#include <Eigen/Core>

namespace BipedalLocomotion {

/**
 * Implementation of the c++17 <code>void_t<\code> metafunction to avoid some static analyzer warnings.
 */
template<typename... Ts> struct make_void { typedef void type;};
template<typename... Ts> using void_t = typename make_void<Ts...>::type;

/**
 * Utility metafunction to avoid compiler warnings about unused variables.
 */
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
 * <code>void_t<\code> is used to detect ill-formed types in SFINAE context.
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
 * when the the object has the square bracket operator, indeed <code>void_t<\code> is used
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
 * <code>void_t<\code> is used to detect ill-formed types in SFINAE context.
 */
template <typename T>
struct is_resizable<T, void_t<decltype(std::declval<T>().resize(std::declval<int>()))>> : std::true_type
{
};

/**
 * is_size_available is a utility metafunction to detect if typename T contains the <code>size()<\code> method.
 */
template <typename T, typename = void> struct is_size_available : std::false_type
{
};

/**
 * is_size_available is a utility metafunction to detect if typename T contains the <code>size()<\code> method.
 * This specialization uses <code>void_t<\code> to detect ill-formed types in SFINAE context.
 */
template <typename T>
struct is_size_available<T, void_t<decltype(std::declval<T>().size())>> : std::true_type
{
};

/**
 * is_data_available is a utility metafunction to detect if typename T contains the <code>data()<\code> method.
 */
template <typename T, typename = void> struct is_data_available : std::false_type
{
};

/**
 * is_data_available is a utility metafunction to detect if typename T contains the <code>data()<\code> method.
 * This specialization uses <code>void_t<\code> to detect ill-formed types in SFINAE context.
 */
template <typename T>
struct is_data_available<T, void_t<decltype(std::declval<T>().data())>> : std::true_type
{
};

/**
 * has_type_member is a utility metafunction to detect if typename T defines <code>value_type<\code>,
 * i.e. <code>T::value_type<\code> is available.
 */
template< class, typename = void >
struct has_type_member : std::false_type { };

/**
 * has_type_member is a utility metafunction to detect if typename T defines <code>value_type<\code>,
 * i.e. <code>T::value_type<\code> is available.
 * This specialization uses <code>void_t<\code> to detect ill-formed types in SFINAE context.
 */
template< class T >
struct has_type_member<T, void_t<typename T::value_type>> : std::true_type { };

/**
 * container_data is a utility metafunction to detect the type of container. If T is not a supported container, it throws
 * an assertion at compile time.
 */
template <typename T, typename = void>
struct container_data
{
    static_assert(dependent_false<T>::value, "Unable to detect type of data in the container.");
};

/**
 * container_data is a utility metafunction to detect the type of container.
 * This specialization is enabled if <code>T::value_type<\code> is available.
 */
template <typename T>
struct container_data<T, typename std::enable_if<has_type_member<T>::value>::type>
{
    using type = typename T::value_type;
};

/**
 * container_data is a utility metafunction to detect the type of container.
 * This specialization is enabled if <code>T::value_type<\code> is not available, but the method <code>data()<\code> is.
 */
template <typename T>
struct container_data<T, typename std::enable_if<!has_type_member<T>::value && is_data_available<T>::value>::type>
{
    using type = typename std::remove_pointer<decltype(std::declval<T>().data())>::type;
};

/**
 * container_data is a utility metafunction to detect the type of container.
 * This specialization is enabled if T is an array.
 */
template <typename T>
struct container_data<T, typename std::enable_if<std::is_array<T>::value>::type>
{
    using type = typename std::remove_all_extents_t<T>;
};

/**
 * size_type is a utility metafunction to detect the type used for the indices in the container.
 * By default if std::ptrdiff_t.
 */
template <typename T, typename = void>
struct size_type
{
    using type = std::ptrdiff_t;
};

/**
 * size_type is a utility metafunction to detect the type used for the indices in the container.
 * In this specialization it returns the return type of the <code>size()<\code> method, provided it exists.
 */
template <typename T>
struct size_type<T, typename std::enable_if<is_size_available<T>::value>::type>
{
    using type = decltype(std::declval<T>().size());
};

/**
 * is_string is a utility metafunction to detect the if a given type is a std::string or it can be
 * trivially converted in a std::string.
 */
template <typename T>
struct is_string : public std::disjunction<std::is_same<char*, typename std::decay<T>::type>,
                                           std::is_same<const char*, typename std::decay<T>::type>,
                                           std::is_same<std::string, typename std::decay<T>::type>>
{
};

/**
 * Utility function used to print the typename T.
 * It has been taken from
 * <a href="https://stackoverflow.com/questions/81870/is-it-possible-to-print-a-variables-type-in-standard-c">here</a>.
 */
template <typename T>
constexpr std::string_view
type_name()
{
    std::string_view name, prefix, suffix;
#ifdef __clang__
    name = __PRETTY_FUNCTION__;
    prefix = "std::string_view BipedalLocomotion::type_name() [T = ";
    suffix = "]";
#elif defined(__GNUC__)
    name = __PRETTY_FUNCTION__;
    prefix = "constexpr std::string_view BipedalLocomotion::type_name() [with T = ";
    suffix = "; std::string_view = std::basic_string_view<char>]";
#elif defined(_MSC_VER)
    name = __FUNCSIG__;
    prefix = "class std::basic_string_view<char,struct std::char_traits<char> > __cdecl BipedalLocomotion::type_name<";
    suffix = ">(void)";
#endif
    name.remove_prefix(prefix.size());
    name.remove_suffix(suffix.size());
    return name;
}

/**
 * is_specialization is used to check if the type Test is a specialization of the class Ref.
 * This specific implementation is called if Test is not specialized from Ref.
 */
template <typename Test, template <typename...> class Ref>
struct is_specialization : std::false_type
{
};

/**
 * is_specialization is used to check if the type Test is a specialization of the class Ref.
 * This specific implementation is called if Test is specialized from Ref.
 */
template <template <typename...> class Ref, typename... Args>
struct is_specialization<Ref<Args...>, Ref> : std::true_type
{
};

/**
 * is_data_const is a template metafunction to detect if the output of T.data() is const.
 */
template<typename T, typename = void >
struct is_data_const : std::false_type
{
};

/**
 * is_data_const is a template metafunction to detect if the output of T.data() is const.
 */
template <typename T>
    struct is_data_const<T,
                     std::enable_if_t<is_data_available<T>::value,
                                      std::enable_if_t<std::is_const_v<decltype(std::declval<T>().data())>>>> : std::true_type
{
};

/**
 * is_container_const is a struct containing a boolean value.
 * If the container is either const itself, or if the output of Container.data() is const, then value is true.
 */
template <typename Container>
struct is_container_const
{
    static constexpr bool value = std::is_const_v<Container> ||
                                  is_data_const<Container>::value;
};

/**
 * is_eigen_matrix is a template metafunction to check if T is an Eigen matrix.
 */
template <typename T, typename = void, typename = void>
struct is_eigen_matrix : std::false_type
{
};

/**
 * is_eigen_matrix is a template metafunction to check if T is an Eigen matrix. In this specialization, we first check if
 * the template parameter inherits from Eigen::MatrixBase<Derived> (Eigen exploits CRTP). If this is the case,
 * we check that neither the rows, nor the columns at compile time are identically equal to 1. If that is the case,
 * is_eigen_matrix<T>::value is true.
 */
template <typename Derived>
struct is_eigen_matrix<Derived,
                       typename std::enable_if_t<std::is_base_of_v<Eigen::MatrixBase<Derived>, Derived>>,
                       typename std::enable_if_t<Eigen::MatrixBase<Derived>::RowsAtCompileTime != 1 &&
                                                 Eigen::MatrixBase<Derived>::ColsAtCompileTime != 1>>: std::true_type
{
};

/**
 * Template metafunction to check if the input type is a pair
 */
template <typename, typename = void, typename = void>
struct is_pair : std::false_type
{ };

/**
 * Template metafunction to check if the input type is a pair
 */
template <typename T>
struct is_pair<T, void_t<decltype(std::declval<T>().first)>, void_t<decltype(std::declval<T>().second)>> : std::true_type
{ };

/**
 * Template metafunction to check if the input type is a pair iterator with a string as first element.
 */
template <typename, typename = void, typename = void>
struct is_pair_iterator_string : std::false_type
{ };

/**
 * Template metafunction to check if the input type is a pair iterator with a string as first element.
 */
template <typename T>
struct is_pair_iterator_string<T,
                               typename std::enable_if_t<is_pair<decltype(*std::declval<T>())>::value>,
                               typename std::enable_if_t<std::is_convertible<decltype(std::declval<T>()->first), std::string>::value>> : std::true_type
{ };

/**
 * Template metafunction implementing std::is_base_of for template classes.
 * https://stackoverflow.com/questions/34672441/stdis-base-of-for-template-classes/34672753
 */
template <template <typename...> class base, typename derived> struct is_base_of_template_impl
{
    template <typename... Ts> static constexpr std::true_type test(const base<Ts...>*);
    static constexpr std::false_type test(...);
    using type = decltype(test(std::declval<derived*>()));
};

template <template <typename...> class base, typename derived>
using is_base_of_template = typename is_base_of_template_impl<base, derived>::type;

} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_TEMPLATEHELPERS_H
