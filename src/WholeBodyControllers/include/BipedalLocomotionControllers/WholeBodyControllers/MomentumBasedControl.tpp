/**
 * @file MomentumBasedControl.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControl.h>

namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{
template <class T>
bool MomentumBasedControl::initialize(std::weak_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                      const iDynTree::VectorDynSize& maxJointsPosition,
                                      const iDynTree::VectorDynSize& minJointsPosition)
{
    if (!m_controllers[WalkingState::DoubleSupport]->initialize(handler,
                                                                "DOUBLE_SUPPORT",
                                                                maxJointsPosition,
                                                                minJointsPosition))
    {
        std::cerr << "[MomentumBasedControl::initialize] Unable to initialize the double support "
                     "controller"
                  << std::endl;
        return false;
    }

    if (!m_controllers[WalkingState::SingleSupportLeft]->initialize(handler,
                                                                    "SINGLE_SUPPORT_LEFT",
                                                                    maxJointsPosition,
                                                                    minJointsPosition))
    {
        std::cerr << "[MomentumBasedControl::initialize] Unable to initialize the single support "
                     "left controller"
                  << std::endl;
        return false;
    }

    if (!m_controllers[WalkingState::SingleSupportRight]->initialize(handler,
                                                                     "SINGLE_SUPPORT_RIGHT",
                                                                     maxJointsPosition,
                                                                     minJointsPosition))
    {
        std::cerr << "[MomentumBasedControl::initialize] Unable to initialize the single support "
                     "right controller"
                  << std::endl;
        return false;
    }

    return true;
}

} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers
