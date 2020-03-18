/**
 * @file GenericContainer.h
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H
#define BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H

#include <iDynTree/Core/Span.h>
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
    iDynTree::Span<T> m_span;
    std::function<iDynTree::Span<T>(size_t)> m_resizeLambda;

public:

    using element_type = typename iDynTree::Span<T>::element_type;
    using value_type = typename iDynTree::Span<T>::value_type;
    using index_type = typename iDynTree::Span<T>::index_type;
    using pointer = typename iDynTree::Span<T>::pointer;
    using reference = typename iDynTree::Span<T>::reference;

    using iterator = typename iDynTree::Span<T>::iterator;
    using const_iterator = typename iDynTree::Span<T>::const_iterator;
    using reverse_iterator = typename iDynTree::Span<T>::reverse_iterator;
    using const_reverse_iterator = typename iDynTree::Span<T>::const_reverse_iterator;

    using size_type = typename iDynTree::Span<T>::size_type;

    GenericContainer(iDynTree::Span<T> span, std::function<iDynTree::Span<T>(size_t)> resizeLambda)
    {
        m_span = span;
        m_resizeLambda = resizeLambda;
    }

    GenericContainer(iDynTree::Span<T> span)
    {
        m_span = span;
        m_resizeLambda = [span](size_type size){return span;};
    }

    ~GenericContainer() = default;

    GenericContainer(const GenericContainer<T>& other) = delete;

    GenericContainer(GenericContainer<T>&& other)
    {
        m_span = other.m_span;
        m_resizeLambda = other.m_resizeLambda;
    }

    bool clone(const GenericContainer<T>& other)
    {
        if (size() != other.size())
        {
            if (!resize(other.size()))
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


    bool resize(index_type newSize)
    {
        m_span = m_resizeLambda(newSize);
        return m_span.size() == newSize;
    }

    index_type size() const
    {
        m_span.size();
    }

    bool empty() const {
        return m_span.empty();
    }

    reference operator[](index_type idx) const
    {
        return m_span[idx];
    }

    double getVal(index_type idx) const
    {
        return this->operator[](idx);
    }

    bool setVal(index_type idx, double val)
    {
        if (idx >= 0 && idx < size())
        {
            this->operator[](idx) = val;
            return true;
        }
        return false;
    }

    reference at(index_type idx) const
    {
        return this->operator[](idx);
    }

    reference operator()(index_type idx) const
    {
        return this->operator[](idx);
    }

    pointer data() const
    {
        return m_span.data();
    }

    // [span.iter], span iterator support
    iterator begin() const
    {
        return m_span.begin();
    }
    iterator end() const
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

    reverse_iterator rbegin() const
    {
        return m_span.rbegin();
    }

    reverse_iterator rend() const
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
    template<typename Class>
    GenericContainer<typename Class::value_type> makeResizableGenericContainer(Class& input)
    {
        Class* inputPtr = &input;
        std::function<iDynTree::Span<typename Class::value_type>(size_t)> resizeLambda =
            [inputPtr](size_t newSize) -> iDynTree::Span<typename Class::value_type>
        {
            inputPtr->resize(newSize);
            return iDynTree::make_span(*inputPtr);
        };

        return GenericContainer(iDynTree::make_span(input), resizeLambda);
    }
}


#endif // BIPEDAL_LOCOMOTION_CONTROLLERS_GENERICCONTAINER_H
