/**
 * @file TaskSpaceInverseDynamics.cpp
 * @authors Ines Sorrentino
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TSID/TaskSpaceInverseDynamics.h>

using namespace BipedalLocomotion::TSID;

bool TaskSpaceInverseDynamics::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    return true;
};


bool TaskSpaceInverseDynamics::setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    return true;
}
