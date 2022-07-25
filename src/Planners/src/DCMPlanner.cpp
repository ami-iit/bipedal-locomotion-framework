/**
 * @file DCMPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Planners/DCMPlanner.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::Contacts;

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
