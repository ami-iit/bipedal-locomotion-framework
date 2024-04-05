/**
 * @file GenericContainer.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_VECTOR_H
#define BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_VECTOR_H

// iDynTree
#include <iDynTree/Span.h>

// Template helpers
#include <BipedalLocomotion/GenericContainer/TemplateHelpers.h>

//std
#include <functional>
#include <cassert>
#include <iostream>
#include <memory>

namespace BipedalLocomotion {
namespace GenericContainer {

/**
 * Forward declaration of Vector class. T is the type of vector (double, float, int, string,...)
 */
template <typename T>
class Vector;

/**
 * Utility alias to a std::shared_ptr of a Vector.
 */
template <typename T>
using Vector_ptr = std::shared_ptr<Vector<T>>;

}
}
/**
 * Vector is a utility class which maps another existing contiguous container.
 * It does not contain any data, but only a pointer to an existing contiguous area of memory containing a sequence of objects.
 * It also stores its size, i.e. the number of objects. It does not own this portion of memory, hence it needs to be properly
 * initialized from an existing container, such as an iDynTree::Vector, std::vector, array, yarp::sig::Vector and similar.
 * Even if it does not own the memory, it is possible to resize it. This is done through an user specified lambda, which
 * calls the corresponding "resize" method on the original object from which Vector has been initialized.
 *
 * @warning The original object from which Vector has been initialized should not be deallocated before Vector.
 * This would invalidate the pointer inside it.
 */
template <typename T>
class BipedalLocomotion::GenericContainer::Vector
{
public:

    /**
     * Utility aliases depending on the type T.
     *
     * Some notes:
     * - vector_element_type has a different name than the corresponding one in iDynTree::Span to avoid some compilation issues.
     * - value_type is the same as vector_element_type, but without an eventual const attribute
     */
    using vector_element_type = typename iDynTree::Span<T>::element_type;
    using value_type = typename iDynTree::Span<T>::value_type;
    using index_type = typename iDynTree::Span<T>::index_type;
    using pointer = typename iDynTree::Span<T>::pointer;
    using reference = typename iDynTree::Span<T>::reference;
    using const_reference = const value_type&;
    using size_type = typename iDynTree::Span<T>::size_type;


    /**
     * Utility aliases to define iterators
     */
    using iterator = typename iDynTree::Span<T>::iterator;
    using const_iterator = typename iDynTree::Span<T>::const_iterator;
    using reverse_iterator = typename iDynTree::Span<T>::reverse_iterator;
    using const_reverse_iterator = typename iDynTree::Span<T>::const_reverse_iterator;

    /**
     * Alias for the type of lambda used to resize the original vector.
     * In particular, it takes as input the new size (of type index size)
     */
    using resize_function_type = std::function<iDynTree::Span<T>(index_type)>;

    /**
     * Alias to determine the output type of toEigen()
     */
    using eigen_map_type = typename Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, 1>>;
    using eigen_map_const_type = typename Eigen::Map<const Eigen::Matrix<T, Eigen::Dynamic, 1>>;


protected:
    /**
     * @brief Span of the pointed vector. This allows to point to an existing container without owning it.
     */
    iDynTree::Span<T> m_span;
    /**
     * @brief User specified lambda to resize the existing container.
     */
    resize_function_type m_resizeLambda;

    /**
     * The default constructor is private. In fact, once the Vector is built, it is assumed to point to an existing container.
     * To be used only if m_span and m_resizeLamba are set manually.
     */
    Vector() = default;

public:

    /**
     * @brief Constructor
     * @param span Span of the existing container
     * @param resizeLambda User defined lambda to resize the original container
     */
    Vector(iDynTree::Span<T> span, resize_function_type resizeLambda)
    {
        m_span = span;
        m_resizeLambda = resizeLambda;
    }

    /**
     * @brief Constructor
     * @param span Span of the existing container
     *
     * Since no resizeLambda is provided, it is assumed that the original container cannot be resized.
     */
    Vector(iDynTree::Span<T> span)
    {
        m_span = span;
        m_resizeLambda = [span](index_type size){unused(size); return span;};
    }

    /**
     * @brief Destructor
     */
    ~Vector() = default;

    /**
     * @brief Copy constructor
     *
     * @warning It has been deleted since it would not be clear if the pointer or the pointed data would be copied.
     */
    Vector(const Vector<T>& other) = delete;

    /**
     * @brief Move constructor
     * @param other Another Vector
     *
     * @warning Here the pointers are copied, the content is not duplicated.
     */
    Vector(Vector<T>&& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    /**
     * @brief Copies the content of the vector
     * @param other Vector from which to copy
     * @return true in case of success. False if the two have different size and this is not resizable.
     *
     * @warning It performs memory allocation if this is resizable and the sizes are different.
     */
    bool clone(const Vector<T>& other)
    {
        if (size() != other.size())
        {
            if (!resizeVector(other.size()))
            {
                std::cerr << "[GenericContainer::Vector] Failed to resize. Copy aborted" << std::endl;
                return false;
            }
        }

        for (index_type i = 0; i < size(); ++i)
        {
            this->operator[](i) = other[i];
        }

        return true;
    }

    /**
     * @brief Copies the content of the vector
     * @param other Span from which to copy
     * @return true in case of success. False if the two have different size and this is not resizable.
     *
     * @warning It performs memory allocation if this is resizable and the sizes are different.
     */
    bool clone(iDynTree::Span<T> other)
    {
        if (size() != other.size())
        {
            if (!resizeVector(other.size()))
            {
                std::cerr << "[GenericContainer::Vector] Failed to resize. Copy aborted" << std::endl;
                return false;
            }
        }

        for (index_type i = 0; i < size(); ++i)
        {
            this->operator[](i) = other[i];
        }

        return true;
    }

    /**
     * @brief operator = Copies the content
     * @param other Vector from which to copy
     * @returns A reference to the vector.
     *
     * It calls clone(). There is an assert on its return value.
     *
     * @warning It performs memory allocation if this is resizable and the sizes are different.
     */
    Vector<T>& operator=(const Vector<T>& other)
    {
        bool ok = clone(other);
        assert(ok);
        unused(ok);
        return *this;
    }

    /**
     * @brief operator = Copies the content
     * @param other Vector from which to copy
     * @returns A reference to the vector.
     *
     * It calls clone(). There is an assert on its return value.
     *
     * @warning It performs memory allocation if this is resizable and the sizes are different.
     */
    Vector<T>& operator=(iDynTree::Span<T> other)
    {
        bool ok = clone(other);
        assert(ok);
        unused(ok);
        return *this;
    }

    /**
     * @brief Move operator = Copies the content
     * @param other Vector from which to copy
     * @returns A reference to the vector.
     *
     * It calls clone(). There is an assert on its return value.
     *
     * @warning It performs memory allocation if this is resizable and the sizes are different.
     */
    Vector<T>& operator=(Vector<T>&& other)
    {
        bool ok = clone(other);
        assert(ok);
        unused(ok);
        return *this;
    }

    /**
     * @brief resizeVector It resize the original vector (if possible)
     * @param newSize New size of the vector
     * @return true if the new size matches the desired. False otherwise.
     *
     * @warning It may perform memory allocation if the new size is different from the current and the resizeLambda has been specified.
     */
    bool resizeVector(index_type newSize)
    {
        m_span = m_resizeLambda(newSize);
        return m_span.size() == newSize;
    }

    /**
     * @brief resize It resize the original vector (if possible)
     * @param newSize New size of the vector
     * It calls resizeVector(). There is an assert on its return value.
     *
     * @warning It may perform memory allocation if the new size is different from the current and the resizeLambda has been specified.
     */
    void resize(index_type newSize)
    {
        bool ok = resizeVector(newSize);
        assert(ok);
        unused(ok);
    }

    /**
     * @brief size Get the size of the Vector
     * @return The size of the Vector
     */
    index_type size() const
    {
        return m_span.size();
    }

    /**
     * @brief empty Checks if the container is empty (zero size)
     * @return True if empty.
     */
    bool empty() const {
        return m_span.empty();
    }

    /**
     * @brief operator [] Accessor
     * @param idx The index to be accessed
     * @return A const reference to the requested element.
     */
    const_reference operator[](index_type idx) const
    {
        return m_span[idx];
    }

    /**
     * @brief operator [] Accessor
     * @param idx The index to be accessed
     * @return A reference to the requested element.
     */
    reference operator[](index_type idx)
    {
        return m_span[idx];
    }

    /**
     * @brief getVal Accessor
     * @param idx The index to be accessed
     * @return A const reference to the requested element.
     */
    vector_element_type getVal(index_type idx) const
    {
        return this->operator[](idx);
    }

    /**
     * @brief setVal Setter
     * @param idx Index of the values to be set
     * @param val The new value
     * @return False if idx is not within [0, size()).
     */
    bool setVal(index_type idx, vector_element_type val)
    {
        if (idx >= 0 && idx < size())
        {
            this->operator[](idx) = val;
            return true;
        }
        return false;
    }

    /**
     * @brief at Accessor
     * @param idx The index to be accessed
     * @return A const reference to the requested element.
     */
    const_reference at(index_type idx) const
    {
        return this->operator[](idx);
    }

    /**
     * @brief at Accessor
     * @param idx The index to be accessed
     * @return A reference to the requested element.
     */
    reference at(index_type idx)
    {
        return this->operator[](idx);
    }

    /**
     * @brief operator () Accessor
     * @param idx The index to be accessed
     * @return A const reference to the requested element.
     */
    const_reference operator()(index_type idx) const
    {
        return this->operator[](idx);
    }

    /**
     * @brief operator () Accessor
     * @param idx The index to be accessed
     * @return A reference to the requested element.
     */
    reference operator()(index_type idx)
    {
        return this->operator[](idx);
    }

    /**
     * @brief data Raw pointer to the data
     * @return THe raw pointer to the data
     */
    pointer data() const
    {
        return m_span.data();
    }

    /**
     * @brief begin Iterator
     * @return An iterator to the end of the sequence
     */
    iterator begin()
    {
        return m_span.begin();
    }

    /**
     * @brief end Iterator
     * @return An iterator to the end of the sequence
     * @warning This element acts as a placeholder; attempting to access it results in undefined behavior.
     */
    iterator end()
    {
        return m_span.end();
    }

    /**
     * @brief begin Iterator
     * @return A const iterator to the end of the sequence
     */
    const_iterator begin() const
    {
        return m_span.cbegin();
    }

    /**
     * @brief end Iterator
     * @return A const iterator to the end of the sequence
     * @warning This element acts as a placeholder; attempting to access it results in undefined behavior.
     */
    const_iterator end() const
    {
        return m_span.cend();
    }

    /**
     * @brief cbegin Iterator
     * @return A const iterator to the end of the sequence
     */
    const_iterator cbegin() const
    {
        return m_span.cbegin();
    }

    /**
     * @brief cend Iterator
     * @return A const iterator to the end of the sequence
     * @warning This element acts as a placeholder; attempting to access it results in undefined behavior.
     */
    const_iterator cend() const
    {
        return m_span.cend();
    }

    /**
     * @brief rbegin Iterator to the first element of the reversed vector
     * @return Returns a reverse iterator to the first element of the reversed vector.
     * It corresponds to the last element of the non-reversed vector
     */
    reverse_iterator rbegin()
    {
        return m_span.rbegin();
    }

    /**
     * @brief rend Iterator to the element following the last element of the reversed vector.
     * @return Returns a reverse iterator to the element following the last element of the reversed vector.
     *  It corresponds to the element preceding the first element of the non-reversed vector.
     * @warning This element acts as a placeholder; attempting to access it results in undefined behavior.
     */
    reverse_iterator rend()
    {
        return m_span.rend();
    }

    /**
     * @brief rbegin Iterator to the first element of the reversed vector
     * @return Returns a const reverse iterator to the first element of the reversed vector.
     * It corresponds to the last element of the non-reversed vector
     */
    const_reverse_iterator rbegin() const
    {
        return m_span.crbegin();
    }

    /**
     * @brief rend Iterator to the element following the last element of the reversed vector.
     * @return Returns a reverse iterator to the element following the last element of the reversed vector.
     *  It corresponds to the element preceding the first element of the non-reversed vector.
     * @warning This element acts as a placeholder; attempting to access it results in undefined behavior.
     */
    const_reverse_iterator rend() const
    {
        return m_span.crend();
    }

    /**
     * @brief crbegin Iterator to the first element of the reversed vector
     * @return Returns a const reverse iterator to the first element of the reversed vector.
     * It corresponds to the last element of the non-reversed vector
     */
    const_reverse_iterator crbegin() const
    {
        return m_span.crbegin();
    }

    /**
     * @brief crend Iterator to the element following the last element of the reversed vector.
     * @return Returns a reverse iterator to the element following the last element of the reversed vector.
     *  It corresponds to the element preceding the first element of the non-reversed vector.
     * @warning This element acts as a placeholder; attempting to access it results in undefined behavior.
     */
    const_reverse_iterator crend() const
    {
        return m_span.crend();
    }

    /**
     * @brief Get an Eigen map corresponding to the current generic vector (see https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html).
     */
    eigen_map_type toEigen()
    {
        return eigen_map_type(data(), size());
    }

    /**
     * @brief Get an Eigen const map corresponding to the current generic vector (see https://eigen.tuxfamily.org/dox/classEigen_1_1Map.html).
     */
    eigen_map_const_type toEigen() const
    {
        return eigen_map_const_type(data(), size());
    }

    /**
     * Forward declaration of Ref, which is used as a reference to the Vector (as &).
     */
    class Ref;

};

namespace BipedalLocomotion::GenericContainer {

/**
 * is_vector is a utility metafunction used to check if T is a GenericContainer::Vector.
 */
template <typename T>
struct is_vector : std::false_type
{
};

/**
 * is_vector is a utility metafunction used to check if T is a GenericContainer::Vector.
 */
template <typename T>
struct is_vector<Vector<T>> : std::true_type
{
};

/**
 * is_span_constructible is a utility metafunction to check if iDynTree::Span is constructible given a reference to Class.
 */
template <typename Class, typename = void>
struct is_span_constructible : std::false_type
{};

/**
 * is_span_constructible is a utility metafunction to check if iDynTree::Span is constructible given a reference to Class.
 */
template <typename Class>
struct is_span_constructible<Class,
                             typename std::enable_if<
                                 std::is_constructible<iDynTree::Span<typename container_data<Class>::type>, Class&>::value>::type>
    : std::true_type
{};

/**
 * is_vector_constructible is a utility metafunction to check if GenericContainer::Vector is constructible given a type T.
 */
template <typename T, typename = void, typename = void>
struct is_vector_constructible : std::false_type
{
};

/**
 * is_vector_constructible is a utility metafunction to check if GenericContainer::Vector is constructible given a type T.
 * This specialization first checks if T is an array, or if it is possible to deduce the type of vector,
 * and that it is not an Eigen matrix (Eigen defines the size() method also for matrices).
 * If not, this specialization is not used (SFINAE). Otherwise, given the vector type, it checks if a iDynTree::Span is construbile
 * or if the methods <code>data()<\code> and <code>size()<\code> are available. In addition, the type has not to be <code>bool<\code>.
 * If all the above checks are true, <code>is_vector_constructible<T>::value = true<\code>.
 */
template <typename T>
    struct is_vector_constructible<T,
                               typename std::enable_if<(std::is_array<T>::value || has_type_member<T>::value || is_data_available<T>::value) && !is_eigen_matrix<T>::value>::type,
    typename std::enable_if<(is_span_constructible<T>::value ||(is_data_available<T>::value && is_size_available<T>::value)) &&
                             !std::is_same<typename container_data<T>::type, bool>::value>::type> : std::true_type
{
};

/**
 * is_vector_ref_constructible is a utility metafunction to check if GenericContainer::Vector::Ref is constructible given a type T and a Container.
 */
template <typename T, typename Container, typename = void, typename = void>
struct is_vector_ref_constructible : std::false_type
{
};

/**
 * is_vector_constructible is a utility metafunction to check if GenericContainer::Vector::Ref is constructible given a type T and a Container.
 * This specialization first checks if a GenericContainer::Vector can be constructed given the Container.
 * If not, this specialization is not used (SFINAE). Otherwise, it checks that the data type of the Container is the same as T.
 * This is useful to avoid ambiguity in interfaces with overloaded methods using different types of Ref.
 * If all the above checks are true, <code>is_vector_ref_constructible<T>::value = true<\code>.
 */
template <typename T, typename Container>
struct is_vector_ref_constructible<T,
                                   Container,
                                   typename std::enable_if<GenericContainer::is_vector_constructible<Container>::value>::type,
                                   typename std::enable_if<std::is_same<std::remove_cv_t<T>, typename std::remove_cv_t<typename container_data<Container>::type>>::value>::type> : std::true_type
{
};

/**
 * @brief The VectorResizeMode enum
 * It defines if the vector can be resized or not.
 */
enum class VectorResizeMode
{
    Resizable,
    Fixed
};

/**
 * @brief Utility function with return a suitable <code>resize_function_type<\code> given the input class.
 * @param input Reference to vector from which the resize lambda is created
 * @returns It returns a lambda calling the <code>resize()<\code> method of the input class.
 *
 * \warning Do not deallocate input if the lambda returned by this method is still used, to avoid segfault.
 * In fact, the output lambda contains a pointer to input.
 */
template<typename Class>
typename Vector<typename container_data<Class>::type>::resize_function_type DefaultVectorResizer(Class& input)
{
    static_assert (is_resizable<Class>::value, "Class type is not resizable.");
    static_assert (is_span_constructible<Class>::value || (is_data_available<Class>::value && is_size_available<Class>::value),
                  "Cannot create a span given the provided class.");

    using value_type = typename container_data<Class>::type;

    if constexpr (is_span_constructible<Class>::value)
    {
        using index_type = typename Vector<value_type>::index_type;
        using resize_function = typename Vector<value_type>::resize_function_type;

        Class* inputPtr = &input;
        resize_function resizeLambda =
            [inputPtr](index_type newSize) -> iDynTree::Span<value_type>
        {
            inputPtr->resize(newSize);
            return iDynTree::make_span(*inputPtr);
        };

        return resizeLambda;
    }
    else
    {
        using index_type = decltype(std::declval<Class>().size());
        using resize_function = typename Vector<value_type>::resize_function_type;

        Class* inputPtr = &input;
        resize_function resizeLambda =
            [inputPtr](index_type newSize) -> iDynTree::Span<value_type>
        {
            inputPtr->resize(newSize);
            return iDynTree::make_span(inputPtr->data(), inputPtr->size());
        };

        return resizeLambda;
    }
}

/**
 * @brief Utility function to create a GenericContainer::Vector from a reference to another vector.
 * @param input The refence to an existing vector.
 * @param mode The resize mode. By default the output Vector is <code>Fixed<\code>.
 * @returns A GenericContainer::Vector pointing to the input vector.
 *
 * @warning The input object from which GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename Class>
Vector<typename container_data<Class>::type>
make_vector(Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    static_assert (!std::is_same<typename container_data<Class>::type, bool>::value,
                  "Cannot create a Vector of bool type. Memory is not contiguos." );
    static_assert (is_span_constructible<Class>::value || (is_data_available<Class>::value && is_size_available<Class>::value),
                  "Cannot create a span given the provided class.");

    using value_type = typename container_data<Class>::type;

    iDynTree::Span<value_type> span;

    if constexpr (is_span_constructible<Class>::value)
    {
        span = iDynTree::make_span(input);
    }
    else
    {
        span = iDynTree::make_span(input.data(), input.size());
    }

    if constexpr (is_resizable<Class>::value)
    {
        if (mode == VectorResizeMode::Resizable)
        {
            return Vector(span, DefaultVectorResizer(input));
        }
        else
        {
            return Vector(span);
        }
    }
    else
    {
        if (mode == VectorResizeMode::Resizable)
        {
            std::cerr << "[GenericContainer::Vector] " << type_name<Class>()
                      << " is not resizable. Returning a non-resizable container." << std::endl;
        }

        return Vector(span);
    }
}

/**
 * @brief Utility function to create a GenericContainer::Vector from a reference to another vector.
 * @param input The refence to an existing vector.
 * @param mode The resize mode. By default the output Vector is <code>Fixed<\code>.
 * @returns A GenericContainer::Vector pointing to the input vector.
 *
 * @warning This version is called if the input type is const. Hence, it cannot be resized.
 * It throws a warning if the selected mode is <code>VectorResizeMode::Resizable<\code>.
 *
 * @warning The input object from which GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template <typename Class>
Vector<const typename container_data<Class>::type>
make_vector(const Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    static_assert (!std::is_same<typename container_data<Class>::type, bool>::value,
                  "Cannot create a Vector of bool type. Memory is not contiguos." );
    static_assert(is_span_constructible<Class>::value
                      || (is_data_available<Class>::value && is_size_available<Class>::value),
                  "Cannot create a span given the provided class.");

    using value_type = typename container_data<decltype(input)>::type;

    iDynTree::Span<value_type> span;

    if constexpr (is_span_constructible<Class>::value)
    {
        span = iDynTree::make_span(input);
    } else
    {
        span = iDynTree::make_span(input.data(), input.size());
    }

    if (mode == VectorResizeMode::Resizable)
    {
        std::cerr << "[GenericContainer::Vector] The input type is const. Returning a non-resizable container." << std::endl;
    }

    return Vector(span);
}

/**
 * @brief Utility function to create a GenericContainer::Vector_ptr from a reference to another vector.
 * @param input The refence to an existing vector.
 * @param mode The resize mode. By default the output Vector is <code>Fixed<\code>.
 * @returns A GenericContainer::Vector_ptr.
 *
 * @warning The input object from which the underlying GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename Class>
Vector_ptr<typename container_data<Class>::type>
make_vector_ptr(Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    return std::make_shared<Vector<typename container_data<Class>::type>>(make_vector(input,mode));
}

/**
 * @brief Utility function to create a GenericContainer::Vector_ptr from a reference to another vector.
 * @param input The refence to an existing vector.
 * @param mode The resize mode. By default the output Vector is <code>Fixed<\code>.
 * @returns A GenericContainer::Vector_ptr.
 *
 * @warning This version is called if the input type is const. Hence, it cannot be resized.
 * It throws a warning if the selected mode is <code>VectorResizeMode::Resizable<\code>.
 *
 * @warning The input object from which the underlying GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename Class>
Vector_ptr<const typename container_data<Class>::type>
make_vector_ptr(const Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    return std::make_shared<Vector<const typename container_data<Class>::type>>(make_vector(input,mode));
}

/**
 * @brief Utility function to create a GenericContainer::Vector_ptr from a reference to another vector.
 * @param span Span of the existing container
 * @returns A GenericContainer::Vector_ptr.
 *
 * Since no resizeLambda is provided, it is assumed that the original container cannot be resized.
 *
 * @warning The input object from which the underlying GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename T>
Vector_ptr<T>
make_vector_ptr(iDynTree::Span<T> span)
{
    return std::make_shared<Vector<T>>(span);
}

/**
 * @brief Utility function to create a GenericContainer::Vector_ptr from a reference to another vector.
 * @param span Span of the existing container
 * @param resizeLambda User defined lambda to resize the original container
 *
 * @warning The input object from which the underlying GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename T>
Vector_ptr<T>
make_vector_ptr(iDynTree::Span<T> span, typename Vector<T>::resize_function_type resizeLambda)
{
    return std::make_shared<Vector<T>>(span, resizeLambda);
}

/**
 * @brief Utility function to create an Eigen map from a reference to another vector.
 * @param input The refence to an existing vector.
 * @returns The coresponding eigen map.
 *
 * @warning The input object from which GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename Class>
typename Vector<typename container_data<Class>::type>::eigen_map_type to_eigen(Class& input)
{
    static_assert (is_vector_constructible<Class>::value,
                  "Cannot create a Vector from the input class. Cannot know how to convert to Eigen." );

    if constexpr (is_vector<Class>::value)
    {
        return input.toEigen();
    }
    else
    {
        return make_vector(input).toEigen();
    }
}

/**
 * @brief Utility function to create an Eigen map from a reference to another vector.
 * @param input The refence to an existing vector.
 * @returns The coresponding eigen map.
 *
 * @warning The input object from which GenericContainer::Vector has been initialized should not be deallocated before it.
 * This would invalidate the pointer inside GenericContainer::Vector.
 */
template<typename Class>
typename Vector<const typename container_data<Class>::type>::eigen_map_const_type to_eigen(const Class& input)
{
    static_assert (is_vector_constructible<Class>::value,
                  "Cannot create a Vector from the input class. Cannot know how to convert to Eigen." );

    if constexpr (is_vector<Class>::value)
    {
        return input.toEigen();
    }
    else
    {
        return make_vector(input).toEigen();
    }
}

}

/**
 * The class ref is used a substitution to a classical reference to a Vector. The advantage of using this,
 * is that custom vectors (all those supported by GenericContainer::Vector) can be implicitly casted to Ref.
 * Ref does not allocate any memory in construction, hence can be used as a parameter to be passed by copy.
 * The = operator clones the content.
 * Ref inherits Vector<T>, hence it can be used as it was a Vector<T>.
 */
template <typename T>
class BipedalLocomotion::GenericContainer::Vector<T>::Ref : public BipedalLocomotion::GenericContainer::Vector<T>
{
public:

    /**
     * @brief A reference cannot exist on its own.
     */
    Ref() = delete;

    /**
     * @brief Copy constructor
     * @param other The ref from which to copy the context.
     */
    Ref(BipedalLocomotion::GenericContainer::Vector<T>::Ref& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    /**
     * @brief Move constructor
     * @param other The ref from which to get the context.
     */
    Ref(BipedalLocomotion::GenericContainer::Vector<T>::Ref&& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    /**
     * @brief Constructor from a GenericContainer::Vector<T>&
     * @param other The input vector from which the context is copied
     */
    Ref(BipedalLocomotion::GenericContainer::Vector<T>& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    /**
     * @brief Constructor from a GenericContainer::Vector<T>&
     * @param other The input vector from which the context is taken
     * In principle, Ref should be the reference of a Vector<T> which should remain alive while
     * Ref is alive. On the other hand, Vector<T> is only a pointer to some data which does not own.
     * Hence, Ref can remain alive even if the Vector<T> is deleted, provided that the original container
     * stays alive.
     */
    Ref(BipedalLocomotion::GenericContainer::Vector<T>&& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    /**
     * Constructor from another container.
     * This is used if:
     * - the input container is not a GenericContainer::Vector, to avoid ambiguities with other constructors
     * - the input container is not a string. This allows using Ref and string with overloaded methods.
     * - a GenericContainer::Vector<T>::Ref can be constructed from the Container
     * - T is not const
     * - the input container is not const.
     */
    template <class Vector, typename = typename std::enable_if<!GenericContainer::is_vector<Vector>::value &&
                                                               !std::is_same<Vector, std::string>::value &&
                                                               GenericContainer::is_vector_ref_constructible<T,Vector>::value &&
                                                               !std::is_const_v<T> &&
                                                               !is_container_const<Vector>::value>::type>
    Ref(Vector& input)
    {
        if constexpr (is_span_constructible<Vector>::value)
        {
            m_span = iDynTree::make_span(input);
        } else
        {
            m_span = iDynTree::make_span(input.data(), input.size());
        }

        if constexpr (BipedalLocomotion::is_resizable<Vector>::value)
        {
            m_resizeLambda = DefaultVectorResizer(input);
        }
        else
        {
            iDynTree::Span<T> copiedSpan = m_span;
            m_resizeLambda = [copiedSpan](index_type size){unused(size); return copiedSpan;};
        }

    }

    /**
     * Constructor from another container.
     * This is used if:
     * - the input container is not a GenericContainer::Vector, to avoid ambiguities with other constructors
     * - the input container is not a string. This allows using Ref and string with overloaded methods.
     * - a GenericContainer::Vector<T>::Ref can be constructed from the Container
     * - T is const.
     */
    template <class Vector, typename = typename std::enable_if<!GenericContainer::is_vector<Vector>::value &&
                                                               !std::is_same<Vector, std::string>::value &&
                                                               GenericContainer::is_vector_ref_constructible<T,Vector>::value &&
                                                               std::is_const_v<T>>::type>
    Ref(const Vector& input)
    {
        if constexpr (is_span_constructible<Vector>::value)
        {
            m_span = iDynTree::make_span(input);
        } else
        {
            m_span = iDynTree::make_span(input.data(), input.size());
        }

        iDynTree::Span<T> copiedSpan = m_span;
        m_resizeLambda = [copiedSpan](index_type size){unused(size); return copiedSpan;};

    }

    /**
    * Default constructor
    */
   ~Ref() = default;

   /**
    * @brief Copy operator
    * @param other The other Ref, from which the data is copied
    * @return A reference to the vector resulting from the copy.
    */
   Ref operator=(const Ref& other)
   {
       return static_cast<BipedalLocomotion::GenericContainer::Vector<T>&>(*this) = other;
   }

   /**
    * @brief Copy operator
    * @param other The other Ref, from which the data is copied
    * @return A reference to the vector resulting from the copy.
    */
   Ref operator=(Ref&& other)
   {
       return static_cast<BipedalLocomotion::GenericContainer::Vector<T>&>(*this) = other;
   }

};


#endif // BIPEDAL_LOCOMOTION_GENERIC_CONTAINER_VECTOR_H
