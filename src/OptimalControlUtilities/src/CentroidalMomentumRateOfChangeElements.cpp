/**
 * @file CentroidalMomentumRateOfChangeElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumRateOfChangeElements.h>

#include <iDynTree/Model/Model.h>

#include <numeric>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

CentroidalLinearMomentumRateOfChangeElement::CentroidalLinearMomentumRateOfChangeElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    PIDController<iDynTree::Vector3> controller,
    const VariableHandler& handler,
    const FramesInContact& framesInContact)
    : ControlTask(kinDyn)
    , m_pid(controller)
{
    m_name = "Centroidal Linear Momentum rate of change Element";

    // get the robot weight expressed in the inertial frame. (The z axis points upwards)
    m_robotMass = m_kinDynPtr->model().getTotalMass();
    double gravity = 9.81;
    m_robotWeight.zero();
    m_robotWeight(2) = -gravity * m_robotMass;

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    for (const auto& frame : framesInContact)
    {
        const auto& frameNameInVariableHandler = frame.identifierInVariableHandler();
        const auto& indexInVariableHandler = handler.getVariable(frameNameInVariableHandler);

        if (!indexInVariableHandler.isValid())
            throw std::runtime_error("[CentroidalLinearMomentumElementWithCompliantContact::"
                                     "CentroidalLinearMomentumElementWithCompliantContact] "
                                     "Undefined label "
                                     + frameNameInVariableHandler + " in the variablehandler");

        // set constant elements in the A matrix
        // A = [ I O I O ...]
        iDynTree::toEigen(m_A).block(0, indexInVariableHandler.offset, 3, 3).setIdentity();
    }

}

const iDynTree::VectorDynSize& CentroidalLinearMomentumRateOfChangeElement::getB()
{
    // Get the output of the PD controller
    m_b = m_pid.getControllerOutput();
    return m_b;
}

void CentroidalLinearMomentumRateOfChangeElement::setReference(
    const iDynTree::Vector3& centroidalLinearMomentumSecondDerivative,
    const iDynTree::Vector3& centroidalLinearMomentumDerivative,
    const iDynTree::Vector3& centroidalLinearMomentum,
    const iDynTree::Vector3& centerOfMass) noexcept
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    iDynTree::Vector3 comTimesMass;
    iDynTree::toEigen(comTimesMass) = iDynTree::toEigen(centerOfMass) * m_robotMass;

    m_pid.setReference(centroidalLinearMomentumSecondDerivative,
                       centroidalLinearMomentumDerivative,
                       centroidalLinearMomentum,
                       comTimesMass);
}

void CentroidalLinearMomentumRateOfChangeElement::setMeasuredContactForces(
    const std::vector<iDynTree::LinearForceVector3>& contactForces)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    // The centroidalLinearMomentumDerivative is equal to the sum of the contact forces and the
    // weight acting on the system
    auto centroidalLinearMomentumDerivative = m_robotWeight;
    centroidalLinearMomentumDerivative = std::accumulate(contactForces.begin(),
                                                         contactForces.end(),
                                                         centroidalLinearMomentumDerivative);

    iDynTree::Vector3 comTimesMass;
    iDynTree::toEigen(comTimesMass) = iDynTree::toEigen(m_kinDynPtr->getCenterOfMassPosition()) * m_robotMass;

    m_pid.setFeedback(centroidalLinearMomentumDerivative,
                      m_kinDynPtr->getCentroidalTotalMomentum().getLinearVec3(),
                      comTimesMass);
}

void CentroidalLinearMomentumRateOfChangeElement::setGains(const iDynTree::Vector3& kd,
                                                           const iDynTree::Vector3& kp,
                                                           const iDynTree::Vector3& ki)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    m_pid.setGains(kd, kp, ki);
}
