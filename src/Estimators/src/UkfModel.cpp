/**
 * @file UkfModel.cpp
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <map>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <BipedalLocomotion/RobotDynamicsEstimator/UkfModel.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::Estimators::RobotDynamicsEstimator;

void UkfModel::unpackState()
{
    m_jointVelocityState = m_currentState.segment(m_stateVariableHandler.getVariable("ds").offset,
                                                  m_stateVariableHandler.getVariable("ds").size);

    for (int subModelIdx = 0; subModelIdx < m_subModelList.size(); subModelIdx++)
    {
        // Take sub-model joint velocities, motor torques, friction torques, ft wrenches, ext
        // contact wrenches
        for (int jointIdx = 0; jointIdx < m_subModelList[subModelIdx].getModel().getNrOfDOFs();
             jointIdx++)
        {
            m_subModelJointVel[subModelIdx](jointIdx)
                = m_jointVelocityState(m_subModelList[subModelIdx].getJointMapping()[jointIdx]);

            m_subModelJointMotorTorque[subModelIdx](jointIdx)
                = m_currentState[m_stateVariableHandler.getVariable("tau_m").offset
                                 + m_subModelList[subModelIdx].getJointMapping()[jointIdx]];

            m_subModelFrictionTorque[subModelIdx](jointIdx)
                = m_currentState[m_stateVariableHandler.getVariable("tau_F").offset
                                 + m_subModelList[subModelIdx].getJointMapping()[jointIdx]];
        }

        for (auto& [key, value] : m_subModelList[subModelIdx].getFTList())
        {
            m_FTMap[key] = m_currentState.segment(m_stateVariableHandler.getVariable(key).offset,
                                                  m_stateVariableHandler.getVariable(key).size);
        }

        for (auto& [key, value] : m_subModelList[subModelIdx].getExternalContactList())
        {
            m_extContactMap[key]
                = m_currentState.segment(m_stateVariableHandler.getVariable(key).offset,
                                         m_stateVariableHandler.getVariable(key).size);
        }
    }
}

bool UkfModel::updateState()
{
    constexpr auto logPrefix = "[UkfModel::updateState]";

    // Update kindyn full model
    m_kinDynFullModel->setRobotState(m_ukfInput.robotBasePose.transform(),
                                     m_ukfInput.robotJointPositions,
                                     iDynTree::make_span(m_ukfInput.robotBaseVelocity.data(),
                                                         manif::SE3d::Tangent::DoF),
                                     m_jointVelocityState,
                                     m_gravity);

    // compute joint acceleration per each sub-model
    for (int subModelIdx = 0; subModelIdx < m_subModelList.size(); subModelIdx++)
    {
        // Get sub-model base pose
        m_worldTBase = Conversions::toManifPose(m_kinDynFullModel->getWorldTransform(
            m_kinDynWrapperList[subModelIdx]->getFloatingBase()));

        // Get sub-model joint position
        for (int jointIdx = 0; jointIdx < m_subModelList[subModelIdx].getModel().getNrOfDOFs();
             jointIdx++)
        {
            m_subModelJointPos[subModelIdx](jointIdx)
                = m_ukfInput
                      .robotJointPositions[m_subModelList[subModelIdx].getJointMapping()[jointIdx]];
        }

        // Get sub-model base vel
        if (!m_kinDynFullModel->getFrameVel(m_kinDynWrapperList[subModelIdx]->getFloatingBase(),
                                            iDynTree::make_span(m_subModelBaseVelTemp.data(),
                                                                manif::SE3d::Tangent::DoF)))
        {
            log()->error("{} Failed while getting the base frame velocity.", logPrefix);
            return false;
        }

        // Set the sub-model state
        m_kinDynWrapperList[subModelIdx]
            ->setRobotState(m_worldTBase.transform(),
                            m_subModelJointPos[subModelIdx],
                            iDynTree::make_span(m_subModelBaseVelTemp.data(),
                                                manif::SE3d::Tangent::DoF),
                            m_subModelJointVel[subModelIdx],
                            m_gravity);

        m_totalTorqueFromContacts[subModelIdx].setZero();

        // Contribution of FT measurements
        for (const auto& [key, value] : m_subModelList[subModelIdx].getFTList())
        {
            m_wrench = (int)value.forceDirection * m_FTMap[key].array();

            if (!m_kinDynWrapperList[subModelIdx]
                     ->getFrameFreeFloatingJacobian(value.index, m_tempJacobianList[subModelIdx]))
            {
                log()->error("{} Failed while getting the jacobian for the frame `{}`.", logPrefix, value.frame);
                return false;
            }

            m_totalTorqueFromContacts[subModelIdx].noalias()
                += m_tempJacobianList[subModelIdx].transpose() * m_wrench;
        }

        // Contribution of unknown external contacts
        for (const auto & [key, value] : m_subModelList[subModelIdx].getExternalContactList())
        {
            if (!m_kinDynWrapperList[subModelIdx]
                     ->getFrameFreeFloatingJacobian(value, m_tempJacobianList[subModelIdx]))
            {
                log()->error("{} Failed while getting the jacobian for the frame `{}`.", logPrefix, value);
                return false;
            }

            m_totalTorqueFromContacts[subModelIdx]
                += m_tempJacobianList[subModelIdx].transpose() * m_extContactMap[key];
        }

        if (subModelIdx == 0)
        {
            if (!m_kinDynWrapperList[subModelIdx]
                     ->forwardDynamics(m_subModelJointMotorTorque[subModelIdx],
                                       m_subModelFrictionTorque[subModelIdx],
                                       m_totalTorqueFromContacts[subModelIdx].tail(
                                           m_kinDynWrapperList[subModelIdx]
                                               ->getNrOfDegreesOfFreedom()),
                                       m_ukfInput.robotBaseAcceleration.coeffs(),
                                       m_subModelJointAcc[subModelIdx]))
            {
                log()->error("{} Cannot compute the inverse dynamics.", logPrefix);
                return false;
            }

            m_subModelNuDot[subModelIdx].head(6) = m_ukfInput.robotBaseAcceleration.coeffs();
            m_subModelNuDot[subModelIdx].tail(
                m_kinDynWrapperList[subModelIdx]->getNrOfDegreesOfFreedom())
                = m_subModelJointAcc[subModelIdx];
        } else
        {
            if (!m_kinDynWrapperList[subModelIdx]
                     ->forwardDynamics(m_subModelJointMotorTorque[subModelIdx],
                                       m_subModelFrictionTorque[subModelIdx],
                                       m_totalTorqueFromContacts[subModelIdx],
                                       m_subModelNuDot[subModelIdx]))
            {
                log()->error("{} Cannot compute the inverse dynamics.", logPrefix);
                return false;
            }

            m_subModelJointAcc[subModelIdx] = m_subModelNuDot[subModelIdx].tail(
                m_kinDynWrapperList[subModelIdx]->getNrOfDegreesOfFreedom());
        }

        // Assign joint acceleration using the correct indeces
        for (int jointIdx = 0; jointIdx < m_subModelList[subModelIdx].getJointMapping().size();
             jointIdx++)
        {
            m_jointAccelerationState[m_subModelList[subModelIdx].getJointMapping()[jointIdx]]
                = m_subModelJointAcc[subModelIdx][jointIdx];
        }
    }

    return true;
}
