/**
 * @file MomentumBasedControl.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControl.h>

using namespace BipedalLocomotionControllers::ParametersHandler;
using namespace BipedalLocomotionControllers::WholeBodyControllers;


MomentumBasedControl::MomentumBasedControl(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : m_currentWalkingState(WalkingState::DoubleSupport)

{
    m_controllers.insert(
        {WalkingState::DoubleSupport, std::make_unique<MomentumBasedControlHelper>(kinDyn)});
    m_controllers.insert(
        {WalkingState::SingleSupportLeft, std::make_unique<MomentumBasedControlHelper>(kinDyn)});
    m_controllers.insert(
        {WalkingState::SingleSupportRight, std::make_unique<MomentumBasedControlHelper>(kinDyn)});
}

bool MomentumBasedControl::initialize(IParametersHandler::weak_ptr handler,
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

void MomentumBasedControl::setFeetState(bool isLeftInContact, bool isRightInContact)
{
    if (isLeftInContact && isRightInContact)
        m_currentWalkingState = WalkingState::DoubleSupport;
    else if (isLeftInContact)
        m_currentWalkingState = WalkingState::SingleSupportLeft;
    else if (isRightInContact)
        m_currentWalkingState = WalkingState::SingleSupportRight;
    else
        assert(false);
}

void MomentumBasedControl::setCentroidalMomentumReference(
    const iDynTree::SpatialForceVector& momentumSecondDerivative,
    const iDynTree::SpatialForceVector& momentumDerivative,
    const iDynTree::SpatialForceVector& momentum,
    const iDynTree::Vector3& centerOfMass)
{
    m_controllers[m_currentWalkingState]->setCentroidalMomentumReference(momentumSecondDerivative,
                                                                         momentumDerivative,
                                                                         momentum,
                                                                         centerOfMass);
}

bool MomentumBasedControl::setMeasuredContactWrench(
    const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches)
{
    return m_controllers[m_currentWalkingState]->setMeasuredContactWrench(contactWrenches);
}

void MomentumBasedControl::setContactState(const iDynTree::Transform& transform,
                                           const std::string& name)
{
    // if the frame is not in contact with the environment setContactState() does not update any
    // contact model
    m_controllers[m_currentWalkingState]->setContactState(name, true, transform);
}

void MomentumBasedControl::setTransformationReference(const iDynTree::SpatialAcc& acceleration,
                                                      const iDynTree::Twist& twist,
                                                      const iDynTree::Transform& transform,
                                                      const std::string& name)
{
    // if the frame is not in contact with the environment setContactState() does not update any
    // contact model
    m_controllers[m_currentWalkingState]->setContactState(name, true, transform);

    // if there is not any tracking Cartesian tasks associated to the frame
    // setTransformationReference() does not update any elements
    m_controllers[m_currentWalkingState]->setTransformationReference(acceleration,
                                                                     twist,
                                                                     transform,
                                                                     name);
}

void MomentumBasedControl::setFootUpperBoundNormalForce(const std::string& name,
                                                        const double& force)
{
    m_controllers[m_currentWalkingState]->setFootUpperBoundNormalForce(name, force);
}

void MomentumBasedControl::setRotationReference(const iDynTree::Vector3& acceleration,
                                                const iDynTree::Vector3& velocity,
                                                const iDynTree::Rotation& rotation,
                                                const std::string& name)
{
    m_controllers[m_currentWalkingState]->setRotationReference(acceleration,
                                                               velocity,
                                                               rotation,
                                                               name);
}

void MomentumBasedControl::setRegularizationReference(const iDynTree::VectorDynSize& acceleration,
                                                      const iDynTree::VectorDynSize& velocity,
                                                      const iDynTree::VectorDynSize& position,
                                                      const std::string& name)
{
    m_controllers[m_currentWalkingState]->setRegularizationReference(acceleration,
                                                                     velocity,
                                                                     position,
                                                                     name);
}

void MomentumBasedControl::setJointState(const iDynTree::VectorDynSize& velocity,
                                         const iDynTree::VectorDynSize& position)
{
    m_controllers[m_currentWalkingState]->setJointState(velocity, position);
}

bool MomentumBasedControl::solve()
{
    return m_controllers[m_currentWalkingState]->solve();
}

iDynTree::VectorDynSize MomentumBasedControl::getDesiredAcceleration()
{
    return m_controllers[m_currentWalkingState]->getDesiredAcceleration();
}
