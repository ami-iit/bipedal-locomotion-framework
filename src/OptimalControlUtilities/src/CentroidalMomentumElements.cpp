/**
 * @file CentroidalMomentumElement.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumElements.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

// Centroidal Linear Momentum
CentroidalLinearMomentumElement::CentroidalLinearMomentumElement(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const std::vector<std::string>& framesInContact)
    : ControlTask(kinDyn)
{
    m_name = "Centroidal Linear Momentum Element";

    // get the robot mass
    m_robotMass = m_kinDynPtr->model().getTotalMass();

    // initialize the PD
    m_pd = std::make_unique<LinearPD<iDynTree::Vector3>>();
    m_zero.zero();

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    for (const std::string& frameName : framesInContact)
    {
        iDynTree::IndexRange variableIndex = handler.getVariable(frameName);
        if (!variableIndex.isValid())
            throw std::runtime_error("[CentroidalLinearMomentumElement::"
                                     "CentroidalLinearMomentumElement] Undefined frame named "
                                     + frameName + " in the variableHandler");

        iDynTree::toEigen(m_A).block(0, variableIndex.offset, 3, 3).setIdentity();
    }
}

void CentroidalLinearMomentumElement::setGain(const iDynTree::Vector3& kp)
{
    m_pd->setGains(kp, m_zero);
}

void CentroidalLinearMomentumElement::setDesiredCentroidalLinearMomentum(
    const iDynTree::Vector3& centroidalLinearMomentumDerivative,
    const iDynTree::Vector3& centroidalLinearMomentum)
{
    // TODO probably it can be optimized
    // u = centroidalLinearMomentumDerivative_des + kp (centroidalLinearMomentum_des -
    // centroidalLinearMomentum)
    m_pd->setDesiredTrajectory(centroidalLinearMomentumDerivative,
                               m_zero,
                               centroidalLinearMomentum);
}

const iDynTree::VectorDynSize& CentroidalLinearMomentumElement::getB()
{
    m_pd->setFeedback(m_zero, m_kinDynPtr->getCentroidalTotalMomentum().getLinearVec3());

    double gravity = 9.81;
    m_b = m_pd->getControllerOutput();
    m_b(2) += gravity * m_robotMass;

    return m_b;
}

// Centroidal Angular momentum
CentroidalAngularMomentumElement::CentroidalAngularMomentumElement(std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
                                                                   const VariableHandler& handler,
                                                                   const std::vector<std::pair<std::string, std::string>>& framesInContact)
    : ControlTask(kinDyn)
{
    m_name = "Centroidal Angular Momentum Element";

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_b.resize(3);
    m_b.zero();

    // initialize the PD
    m_pd = std::make_unique<LinearPD<iDynTree::Vector3>>();
    m_zero.zero();

    for (const auto& frame : framesInContact)
    {
        Frame frameInContact;
        frameInContact.indexRangeInElement = handler.getVariable(frame.first);
        frameInContact.indexInModel = m_kinDynPtr->model().getFrameIndex(frame.second);

        if (!frameInContact.indexRangeInElement.isValid())
            throw std::runtime_error("[CentroidalAngularMomentumElement::"
                                     "CentroidalAngularMomentumElement] Undefined frame named "
                                     + frame.first + " in the variableHandler");

        if (frameInContact.indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[CentroidalAngularMomentumElement::"
                                     "CentroidalAngularMomentumElement] Undefined frame named "
                                     + frame.second + " in the model");

        m_framesInContact.push_back(frameInContact);

        // the matrix A relative to one contact is
        // A = [skew_symmetric; I]
        iDynTree::toEigen(m_A)
            .block(0, frameInContact.indexRangeInElement.offset + 3, 3, 3)
            .setIdentity();
    }
}

void CentroidalAngularMomentumElement::setGain(const iDynTree::Vector3& kp)
{
    m_pd->setGains(kp, m_zero);
}

void CentroidalAngularMomentumElement::setDesiredCentroidalAngularMomentum(
    const iDynTree::Vector3& centroidalAngularMomentumDerivative,
    const iDynTree::Vector3& centroidalAngularMomentum)
{
    // TODO probably it can be optimized
    // u = centroidalAngularMomentumDerivative_des + kp (centroidalAngularMomentum_des -
    // centroidalAngularMomentum)
    m_pd->setDesiredTrajectory(centroidalAngularMomentumDerivative,
                               m_zero,
                               centroidalAngularMomentum);
}

const iDynTree::MatrixDynSize& CentroidalAngularMomentumElement::getA()
{
    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    for (const auto& frame : m_framesInContact)
    {
        iDynTree::toEigen(m_A).block(0, frame.indexRangeInElement.offset, 3, 3) = iDynTree::skew(
            iDynTree::toEigen(m_kinDynPtr->getWorldTransform(frame.indexInModel).getPosition())
            - iDynTree::toEigen(com));
    }
    return m_A;
}

const iDynTree::VectorDynSize& CentroidalAngularMomentumElement::getB()
{
    m_pd->setFeedback(m_zero, m_kinDynPtr->getCentroidalTotalMomentum().getAngularVec3());
    iDynTree::toEigen(m_b) = iDynTree::toEigen(m_pd->getControllerOutput());
    return m_b;
}
