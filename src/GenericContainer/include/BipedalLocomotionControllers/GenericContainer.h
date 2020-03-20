/**
 * @file GenericContainer.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H

#include <iDynTree/Core/Span.h>

#include <BipedalLocomotionControllers/TemplateHelpers.h>

#include <functional>
#include <cassert>
#include <iostream>

namespace BipedalLocomotionControllers {
    template <typename T>
    class GenericContainer;
}

template <typename T>
class BipedalLocomotionControllers::GenericContainer
{
public:

    using element_type = typename iDynTree::Span<T>::element_type;
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

    GenericContainer() = delete; //Since the = copies the content, there would be no possibility to set the internal pointers.

    GenericContainer(iDynTree::Span<T> span, resize_function_type resizeLambda)
    {
        m_span = span;
        m_resizeLambda = resizeLambda;
    }

    GenericContainer(iDynTree::Span<T> span)
    {
        m_span = span;
        m_resizeLambda = [span](index_type size){unused(size); return span;};
    }

    ~GenericContainer() = default;

    GenericContainer(const GenericContainer<T>& other) = delete; //It would not be clear if you are copying the pointers or the content.

    GenericContainer(GenericContainer<T>&& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    bool clone(const GenericContainer<T>& other)
    {
        if (size() != other.size())
        {
            if (!resizeContainer(other.size()))
            {
                std::cerr << "[Generic container] Failed to resize. Copy aborted" << std::endl;
                return false;
            }
        }

        for (index_type i = 0; i < size(); ++i)
        {
            this->operator[](i) = other[i];
        }

        return true;
    }

    void operator=(const GenericContainer<T>& other)
    {
        clone(other);
    }

    void operator=(GenericContainer<T>&& other)
    {
        clone(other);
    }


    bool resizeContainer(index_type newSize)
    {
        m_span = m_resizeLambda(newSize);
        return m_span.size() == newSize;
    }

    void resize(index_type newSize)
    {
        bool ok = resizeContainer(newSize);
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

    element_type getVal(index_type idx) const
    {
        return this->operator[](idx);
    }

    bool setVal(index_type idx, element_type val)
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

    const_reverse_iterator crbegin() const
    {
        return m_span.crbegin();
    }

    const_reverse_iterator crend() const
    {
        return m_span.crend();
    }

};

namespace BipedalLocomotionControllers {

    enum class GenericContainerMode
    {
        Resizable,
        Fixed
    };

    template<typename Class, typename = typename container_data<Class>::type>
    typename GenericContainer<typename container_data<Class>::type>::resize_function_type GenericContainerDefaultResizer(Class& input)
    {
        static_assert (is_resizable<Class>::value, "Class type is not resizable.");
        static_assert (is_span_constructible<Class>::value || (is_data_available<Class>::value && is_size_available<Class>::value),
                      "Cannot create a span given the provided class.");

        using value_type = typename container_data<Class>::type;

        if constexpr (is_span_constructible<Class>::value)
        {
            using index_type = typename GenericContainer<value_type>::index_type;
            using resize_function = typename GenericContainer<value_type>::resize_function_type;

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
            using resize_function = typename GenericContainer<value_type>::resize_function_type;

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

    template<typename Class, typename = typename container_data<Class>::type>
    GenericContainer<typename container_data<Class>::type>
    makeGenericContainer(Class& input, GenericContainerMode mode = GenericContainerMode::Fixed)
    {
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
            if (mode == GenericContainerMode::Resizable)
            {
                return GenericContainer(span, GenericContainerDefaultResizer(input));
            }
            else
            {
                return GenericContainer(span);
            }
        }
        else
        {
            if (mode == GenericContainerMode::Resizable)
            {
                std::cerr << "[GenericContainer] " << type_name<Class>()
                          << " is not resizable. Returning a non-resizable container." << std::endl;
            }

            return GenericContainer(span);
        }
    }

    template <typename Class, typename = const typename container_data<Class>::type>
    GenericContainer<const typename container_data<Class>::type>
    makeGenericContainer(const Class& input, GenericContainerMode mode = GenericContainerMode::Fixed)
    {
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

        if (mode == GenericContainerMode::Resizable)
        {
            std::cerr << "[GenericContainer] The input type is const. Returning a non-resizable container." << std::endl;
        }

        return GenericContainer(span);
    }
}


#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H
