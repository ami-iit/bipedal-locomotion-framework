/**
 * @file DCMPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/Planners/DCMPlanner.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::Contacts;

bool DCMPlanner::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
}

bool DCMPlanner::setContactPhaseList(const ContactPhaseList& contactPhaseList)
{
    m_contactPhaseList = contactPhaseList;
    return true;
}

void DCMPlanner::setInitialState(const DCMPlannerState& initialState)
{
    m_initialState = initialState;
}

bool DCMPlanner::setDCMReference(Eigen::Ref<const Eigen::MatrixXd> dcmReference)
{
    return true;
};
