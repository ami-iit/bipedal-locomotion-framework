/**
 * @file YarpImplementation.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h>

using namespace BipedalLocomotionControllers::ParametersHandler;

YarpImplementation::YarpImplementation(const yarp::os::Searchable& searchable)
    : m_searchable(searchable)
{
}

std::unique_ptr<IParametersHandler<YarpImplementation>> YarpImplementation::getGroup(const std::string& name) const
{
    auto& group = m_searchable.findGroup(name);
    if (group.isNull())
        return nullptr;

    return std::make_unique<YarpImplementation>(group);
}
