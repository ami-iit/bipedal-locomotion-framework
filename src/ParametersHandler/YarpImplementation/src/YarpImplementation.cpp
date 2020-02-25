/**
 * @file YarpImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

#include <string>

using namespace BipedalLocomotionControllers::ParametersHandler;

YarpImplementation::YarpImplementation(const yarp::os::Searchable& searchable)
{
    set(searchable);
}

void YarpImplementation::set(const yarp::os::Searchable &searchable)
{
    m_container.fromString(searchable.toString());
}

YarpImplementation::weak_ptr YarpImplementation::getGroup(const std::string& name) const
{
    auto& group = m_container.findGroup(name);
    if (group.isNull())
        return YarpImplementation::make_shared();

    return YarpImplementation::make_shared(group);
}

std::string YarpImplementation::toString() const
{
    return m_container.toString();
}

bool YarpImplementation::isEmpty() const
{
    // if the toString method returns an empty string means that the handler is empty
    return m_container.toString().empty();
}
