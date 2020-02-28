/**
 * @file MomentumBasedControl.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControl.h>

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

void MomentumBasedControl::setFeetState(bool isLeftInContact, bool isRightInContact)
{
    if (isLeftInContact && isRightInContact)
    {
        std::cerr << "double" << std::endl;
        m_currentWalkingState = WalkingState::DoubleSupport;
    }

    else if (isLeftInContact)
    {
        std::cerr << "left" << std::endl;
        m_currentWalkingState = WalkingState::SingleSupportLeft;
    }

    else if (isRightInContact)
    {
        std::cerr << "right" << std::endl;
        m_currentWalkingState = WalkingState::SingleSupportRight;
    } else
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
