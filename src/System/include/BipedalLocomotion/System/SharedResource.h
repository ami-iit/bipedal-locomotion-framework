/**
 * @file SharedResource.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_SHARED_RESOURCE_H
#define BIPEDAL_LOCOMOTION_SYSTEM_SHARED_RESOURCE_H

#include <memory>
#include <mutex>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Basic class that contains a shared resource between two Block classes. Since this class is a
 * shared resource you can use it only with shared_ptr. For this reason the only way to create a
 * shared resource is to use the SharedResource::create() static method. If want a visual
 * interpretation of shared resource you may imagine it as the wire connecting two blocks running in
 * two separate threads. The implementation of SharedResource is thread safe.
 */
template <class T> class SharedResource
{
    T m_resource; /**< The resource */
    mutable std::mutex m_mutex; /**< The mutex used to protect the resource */

    SharedResource() = default;

public:
    using Ptr = std::shared_ptr<SharedResource<T>>;

    /**
     * Set the resource
     */
    inline void set(const T& resource);

    /**
     * Get the resource.
     * @return the copy of the object inside the shared resource.
     */
    inline T get() const;

    /**
     * Method used to create a shared resource.
     * @return a pointer of a shared resource.
     */
    static Ptr create();
};

template <class T> void SharedResource<T>::set(const T& resource)
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_resource = resource;
}

template <class T> T SharedResource<T>::get() const
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_resource;
}

template <class T> typename SharedResource<T>::Ptr SharedResource<T>::create()
{
    return std::shared_ptr<SharedResource<T>>(new SharedResource<T>());
}

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_SHARED_RESOURCE_H
