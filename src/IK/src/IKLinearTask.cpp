/**
 * @file IKLinearTask.cpp
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <BipedalLocomotion/IK/IKLinearTask.h>

using namespace BipedalLocomotion::IK;

bool IKLinearTask::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    return true;
}
