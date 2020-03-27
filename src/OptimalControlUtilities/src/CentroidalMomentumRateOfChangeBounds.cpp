/**
 * @file CentroidalMomentumRateOfChangeElements.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumRateOfChangeBounds.h>

#include <iDynTree/Model/Model.h>

#include <numeric>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

CentroidalAngularMomentumRateOfChangeBounds::CentroidalAngularMomentumRateOfChangeBounds(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn,
    const VariableHandler& handler,
    const FramesInContact& framesInContact,
    const iDynTree::Vector3& angularMomentumUpperBound,
    const iDynTree::Vector3& angularMomentumLowerBound,
    const double& dT)
    : InequalityConstraintElement(kinDyn)
    , m_angularMomentumUpperBound(angularMomentumUpperBound)
    , m_angularMomentumLowerBound(angularMomentumLowerBound)
    , m_dT(dT)
{
    // required for the integrator
    using namespace BipedalLocomotionControllers::Simulator;

    m_name = "Centroidal Angular Momentum rate of change Bounds [ ";

    // resize and reset matrices
    m_A.resize(3, handler.getNumberOfVariables());
    m_A.zero();
    m_l.resize(3);
    m_l.zero();
    m_u.resize(3);
    m_u.zero();

    for (const auto& frame : framesInContact)
    {
        const auto& frameNameInVariableHandler = frame.identifierInVariableHandler();
        const auto& frameNameInModel = frame.identifierInModel();
        const auto& indexInVariableHandler = handler.getVariable(frameNameInVariableHandler);
        const auto& indexInModel = m_kinDynPtr->model().getFrameIndex(frameNameInModel);

        m_name += "( Label: " + frameNameInVariableHandler + ", Frame: " + frameNameInModel + " ) ";

        if (!indexInVariableHandler.isValid())
            throw std::runtime_error("[CentroidalAngularMomentumElementWithCompliantContact::"
                                     "CentroidalAngularMomentumElementWithCompliantContact] "
                                     "Undefined label "
                                     + frameNameInVariableHandler + " in the variablehandler");

        if (indexInModel == iDynTree::FRAME_INVALID_INDEX)
            throw std::runtime_error("[CentroidalAngularMomentumElementWithCompliantContact::"
                                     "CentroidalLineartMomentumElementWithCompliantContact] "
                                     "Undefined farme "
                                     + frameNameInModel + " in the model");

        // set constant elements in the A matrix
        // A = [ * I * I ...]
        iDynTree::toEigen(m_A).middleCols<3>(indexInVariableHandler.offset + 3).setIdentity();
        iDynTree::toEigen(m_A).middleCols<3>(indexInVariableHandler.offset + 3)
            = iDynTree::toEigen(m_A).middleCols<3>(indexInVariableHandler.offset + 3) * m_dT * m_dT
              / 2.0;

        m_framesInContact.insert(
            {frameNameInVariableHandler,
             {indexInVariableHandler, indexInModel, /*compliant contact = */ true}});
    }

    m_name += "]";
}

void CentroidalAngularMomentumRateOfChangeBounds::computeBound(
    const iDynTree::Vector3& angularMomentumBound, iDynTree::VectorDynSize& bound)
{
    using iDynTree::toEigen;

    toEigen(bound) = toEigen(angularMomentumBound)
                   - toEigen(m_kinDynPtr->getCentroidalTotalMomentum().getAngularVec3())
                   - toEigen(m_angularMomentumDerivative) * m_dT;

    iDynTree::Vector3 comVelocity = m_kinDynPtr->getCenterOfMassVelocity();
    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;
        toEigen(bound)
            -= iDynTree::skew(
                   toEigen(m_kinDynPtr->getFrameVel(frame.identifierInModel()).getLinearVec3())
                   - toEigen(comVelocity))
               * toEigen(frame.contactWrench().getLinearVec3()) * m_dT * m_dT / 2;
    }
}

const iDynTree::VectorDynSize& CentroidalAngularMomentumRateOfChangeBounds::getUpperBound()
{
    computeBound(m_angularMomentumUpperBound, m_u);
    std::cerr << "angular upper bound" << m_angularMomentumUpperBound.toString() << std::endl;
    std::cerr << "upper bound" << m_u.toString() << std::endl;

    return m_u;
}

const iDynTree::VectorDynSize& CentroidalAngularMomentumRateOfChangeBounds::getLowerBound()
{
    computeBound(m_angularMomentumLowerBound, m_l);
    std::cerr << "lower bound" << m_l.toString() << std::endl;
    return m_l;
}

const iDynTree::MatrixDynSize& CentroidalAngularMomentumRateOfChangeBounds::getA()
{
    using iDynTree::toEigen;

    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    for (const auto& frameInContact : m_framesInContact)
    {
        const auto& frame = frameInContact.second;

        toEigen(m_A).middleCols<3>(frame.identifierInVariableHandler().offset) = iDynTree::skew(
            toEigen(m_kinDynPtr->getWorldTransform(frame.identifierInModel()).getPosition())
            - toEigen(com)) * m_dT * m_dT / 2;
    }

    std::cerr << m_A.toString() << std::endl;

    return m_A;
}

bool CentroidalAngularMomentumRateOfChangeBounds::setMeasuredContactWrenches(
    const std::unordered_map<std::string ,iDynTree::Wrench>& contactWrenches)
{
    // u = centroidalLinearMomentumSecondDerivative_des
    //    + kd (centroidalLinearMomentumDerivative_des - centroidalLinearMomentumDerivative)
    //    + kp (centroidalLinearMomentum_des - centroidalLinearMomentum)

    iDynTree::Vector3 angularMomentumDerivative;
    angularMomentumDerivative.zero();

    iDynTree::Position com;
    com = m_kinDynPtr->getCenterOfMassPosition();

    // check for all frame in contact the corresponding contact wrench
    for(auto& frameInContact : m_framesInContact)
    {
        const std::string& key = frameInContact.first;
        auto& frame = frameInContact.second;

        // check if the key is associated to a frame
        const auto& contactWrench = contactWrenches.find(key);
        if (contactWrench == contactWrenches.end())
        {
            std::cerr << "[CentroidalAngularMomentumRateOfChangeBounds::"
                         "setMeasuredContactWrenches] The label "
                      << key << " is not associated to any contact wrench" << std::endl;
            return false;
        }
        const iDynTree::Wrench& wrench = contactWrench->second;

        // store wrench
        frame.contactWrench() = wrench;

        // compute angular momentum derivative
        iDynTree::toEigen(m_angularMomentumDerivative)
            += iDynTree::toEigen(wrench.getAngularVec3())
               + iDynTree::skew(toEigen(
                     m_kinDynPtr->getWorldTransform(frame.identifierInModel()).getPosition() - com))
                     * iDynTree::toEigen(wrench.getLinearVec3());
    }
    return true;
}
