/**
 * @file GenericContainer.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H

#include <iDynTree/Core/Span.h>

#include <BipedalLocomotionControllers/GenericContainer/TemplateHelpers.h>

#include <functional>
#include <cassert>
#include <iostream>
#include <memory>

namespace BipedalLocomotionControllers {
namespace GenericContainer {

template <typename T>
class Vector;

template <typename T>
using Vector_ptr = std::shared_ptr<Vector<T>>;

}
}

template <typename T>
class BipedalLocomotionControllers::GenericContainer::Vector
{
public:

    using vector_element_type = typename iDynTree::Span<T>::element_type; //had to rename this otherwise make_span(GenericContainer::Vector) does not compile
    using value_type = typename iDynTree::Span<T>::value_type; //like element_type, but without an eventual const
    using index_type = typename iDynTree::Span<T>::index_type;
    using pointer = typename iDynTree::Span<T>::pointer;
    using reference = typename iDynTree::Span<T>::reference;
    using const_reference = const value_type&;

    using iterator = typename iDynTree::Span<T>::iterator;
    using const_iterator = typename iDynTree::Span<T>::const_iterator;
    using reverse_iterator = typename iDynTree::Span<T>::reverse_iterator;
    using const_reverse_iterator = typename iDynTree::Span<T>::const_reverse_iterator;

    using size_type = typename iDynTree::Span<T>::size_type;
    using resize_function_type = std::function<iDynTree::Span<T>(index_type)>;

private:
    iDynTree::Span<T> m_span;
    resize_function_type m_resizeLambda;

public:

    Vector() = delete; //Since the = copies the content, there would be no possibility to set the internal pointers.

    Vector(iDynTree::Span<T> span, resize_function_type resizeLambda)
    {
        m_span = span;
        m_resizeLambda = resizeLambda;
    }

    Vector(iDynTree::Span<T> span)
    {
        m_span = span;
        m_resizeLambda = [span](index_type size){unused(size); return span;};
    }

    ~Vector() = default;

    Vector(const Vector<T>& other) = delete; //It would not be clear if you are copying the pointers or the content.

    Vector(Vector<T>&& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

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

    void operator=(const Vector<T>& other)
    {
        clone(other);
    }

    void operator=(Vector<T>&& other)
    {
        clone(other);
    }


    bool resizeVector(index_type newSize)
    {
        m_span = m_resizeLambda(newSize);
        return m_span.size() == newSize;
    }

    void resize(index_type newSize)
    {
        bool ok = resizeVector(newSize);
        assert(ok);
        unused(ok);
    }

    index_type size() const
    {
        return m_span.size();
    }

    bool empty() const {
        return m_span.empty();
    }

    const_reference operator[](index_type idx) const
    {
        return m_span[idx];
    }

    reference operator[](index_type idx)
    {
        return m_span[idx];
    }

    vector_element_type getVal(index_type idx) const
    {
        return this->operator[](idx);
    }

    bool setVal(index_type idx, vector_element_type val)
    {
        if (idx >= 0 && idx < size())
        {
            this->operator[](idx) = val;
            return true;
        }
        return false;
    }

    const_reference at(index_type idx) const
    {
        return this->operator[](idx);
    }

    reference at(index_type idx)
    {
        return this->operator[](idx);
    }

    const_reference operator()(index_type idx) const
    {
        return this->operator[](idx);
    }

    reference operator()(index_type idx)
    {
        return this->operator[](idx);
    }

    pointer data() const
    {
        return m_span.data();
    }

    // [span.iter], span iterator support
    iterator begin()
    {
        return m_span.begin();
    }
    iterator end()
    {
        return m_span.end();
    }

    const_iterator begin() const
    {
        return m_span.cbegin();
    }
    const_iterator end() const
    {
        return m_span.cend();
    }

    const_iterator cbegin() const
    {
        return m_span.cbegin();
    }

    const_iterator cend() const
    {
        return m_span.cend();
    }

    reverse_iterator rbegin()
    {
        return m_span.rbegin();
    }

    reverse_iterator rend()
    {
        return m_span.rend();
    }

    const_reverse_iterator rbegin() const
    {
        return m_span.crbegin();
    }

    const_reverse_iterator rend() const
    {
        return m_span.crend();
    }

    const_reverse_iterator crbegin() const
    {
        return m_span.crbegin();
    }

    const_reverse_iterator crend() const
    {
        return m_span.crend();
    }

};

namespace BipedalLocomotionControllers::GenericContainer {

template <typename T>
struct is_vector : std::false_type
{
};

template <typename T>
struct is_vector<Vector<T>> : std::true_type
{
};

template <typename Class, typename = void>
struct is_span_constructible : std::false_type
{};

template <typename Class>
struct is_span_constructible<Class,
                             typename std::enable_if<
                                 std::is_constructible<iDynTree::Span<typename vector_data<Class>::type>, Class&>::value>::type>
    : std::true_type
{};

enum class VectorResizeMode
{
    Resizable,
    Fixed
};

template<typename Class>
typename Vector<typename vector_data<Class>::type>::resize_function_type DefaultVectorResizer(Class& input)
{
    static_assert (is_resizable<Class>::value, "Class type is not resizable.");
    static_assert (is_span_constructible<Class>::value || (is_data_available<Class>::value && is_size_available<Class>::value),
                  "Cannot create a span given the provided class.");

    using value_type = typename vector_data<Class>::type;

    if constexpr (is_span_constructible<Class>::value)
    {
        using index_type = typename Vector<value_type>::index_type;
        using resize_function = typename Vector<value_type>::resize_function_type;

        Class* inputPtr = &input;
        resize_function resizeLambda =
            [inputPtr](index_type newSize) -> iDynTree::Span<typename Class::value_type>
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

template<typename Class>
Vector<typename vector_data<Class>::type>
make_vector(Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    static_assert (is_span_constructible<Class>::value || (is_data_available<Class>::value && is_size_available<Class>::value),
                  "Cannot create a span given the provided class.");

    using value_type = typename vector_data<Class>::type;

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

template <typename Class>
Vector<const typename vector_data<Class>::type>
make_vector(const Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    static_assert(is_span_constructible<Class>::value
                      || (is_data_available<Class>::value && is_size_available<Class>::value),
                  "Cannot create a span given the provided class.");

    using value_type = typename vector_data<decltype(input)>::type;

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

template<typename Class>
Vector_ptr<typename vector_data<Class>::type>
make_vector_ptr(Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    return std::make_shared<Vector<typename vector_data<Class>::type>>(make_vector(input,mode));
}

template<typename Class>
Vector_ptr<const typename vector_data<Class>::type>
make_vector_ptr(const Class& input, VectorResizeMode mode = VectorResizeMode::Fixed)
{
    return std::make_shared<Vector<const typename vector_data<Class>::type>>(make_vector(input,mode));
}

template<typename T>
Vector_ptr<T>
make_vector_ptr(iDynTree::Span<T> span)
{
    return std::make_shared<Vector<T>>(span);
}

template<typename T>
Vector_ptr<T>
make_vector_ptr(iDynTree::Span<T> span, typename Vector<T>::resize_function_type resizeLambda)
{
    return std::make_shared<Vector<T>>(span, resizeLambda);
}

}


#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H
