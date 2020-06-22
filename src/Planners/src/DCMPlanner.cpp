/**
 * @file DCMPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Planners/DCMPlanner.h>

using namespace BipedalLocomotion::Planners;

bool DCMPlanner::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
}

bool DCMPlanner::setContactPhaseList(std::weak_ptr<const ContactPhaseList> contactPhaseList)
{
    if(contactPhaseList.expired())
    {
        std::cerr << "[DCMPlanner::setContactPhaseList] The contactPhaseList pointer is expired. "
                     "Please pass a valid pointer."
                  << std::endl;
        return false;
    }

    m_contactPhaseList = contactPhaseList;
    return true;
}

void DCMPlanner::setInitialState(const DCMPlannerState& initialState)
{
    m_initialState = initialState;
}
