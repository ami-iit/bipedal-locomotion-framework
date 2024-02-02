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
    m_jointVelocityState
        = m_currentState.segment(m_stateVariableHandler.getVariable("JOINT_VELOCITIES").offset,
                                 m_stateVariableHandler.getVariable("JOINT_VELOCITIES").size);

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
                = m_currentState[m_stateVariableHandler.getVariable("MOTOR_TORQUES").offset
                                 + m_subModelList[subModelIdx].getJointMapping()[jointIdx]];

            m_subModelFrictionTorque[subModelIdx](jointIdx)
                = m_currentState[m_stateVariableHandler.getVariable("FRICTION_TORQUES").offset
                                 + m_subModelList[subModelIdx].getJointMapping()[jointIdx]];
        }

        for (auto& [key, value] : m_subModelList[subModelIdx].getFTList())
        {
            m_FTMap[key]
                = m_currentState
                      .segment(m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).offset,
                               m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).size);
        }

        for (auto& [key, value] : m_subModelList[subModelIdx].getExternalContactList())
        {
            m_extContactMap[key]
                = m_currentState
                      .segment(m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).offset,
                               m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).size);
        }

        for (const auto& [key, value] : m_subModelList[subModelIdx].getAccelerometerList())
        {
            m_accMap[key]
                = m_currentState
                      .segment(m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).offset,
                               m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).size);
        }

        for (const auto& [key, value] : m_subModelList[subModelIdx].getGyroscopeList())
        {
            m_gyroMap[key]
                = m_currentState
                      .segment(m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).offset,
                               m_stateVariableHandler.getVariable(m_stateToUkfNames[key]).size);
        }
    }
}

bool UkfModel::updateState()
{
    constexpr auto logPrefix = "[UkfModel::updateState]";

    m_baseVelocity.setZero();
    m_baseAcceleration.setZero();

    // The full model shares the same base as the first submodel
    for (const auto& [key, value] : m_subModelList[0].getGyroscopeList())
    {
        if (m_subModelList[0].getImuBaseFrameName() == value.frame)
        {
            m_baseVelocity.coeffs().tail(3) = m_gyroMap[key];
        }
    }

    manif::SE3d baseHimu; // Transform from the base frame to the imu frame.

    // Get transform matrix from imu to base
    baseHimu = Conversions::toManifPose(
        m_kinDynFullModel->getRelativeTransform(m_kinDynFullModel->getFloatingBase(),
                                                m_subModelList[0].getImuBaseFrameName()));

    m_baseVelocity.coeffs().tail(3).noalias() = baseHimu.rotation() * m_baseVelocity.coeffs().tail(3);

    m_gravity.setZero();

    // Set the robot state of the kindyn of the full model using the sensor proper acceleration
    m_kinDynFullModel->setRobotState(m_ukfInput.robotBasePose.transform(),
                                     m_ukfInput.robotJointPositions,
                                     iDynTree::make_span(m_baseVelocity.data(),
                                                         manif::SE3d::Tangent::DoF),
                                     m_jointVelocityState,
                                     m_gravity);

    // Compute acceleration of the submodel bases from imu measurements
    for (int subModelIdx = 0; subModelIdx < m_subModelList.size(); subModelIdx++)
    {
        // Get sub-model base pose expressed in the world frame that is used to set the kindyn state of the submodel
        m_worldTBase = Conversions::toManifPose(m_kinDynFullModel->getWorldTransform(
            m_kinDynWrapperList[subModelIdx]->getFloatingBase()));

        // Get sub-model joint positions used to set the kindyn state of the submodel
        for (int jointIdx = 0; jointIdx < m_subModelList[subModelIdx].getModel().getNrOfDOFs();
             jointIdx++)
        {
            m_subModelJointPos[subModelIdx](jointIdx)
                = m_ukfInput
                      .robotJointPositions[m_subModelList[subModelIdx].getJointMapping()[jointIdx]];
        }

        // Get the transform matrix from imu to the base frame of the submodel
        baseHimu = Conversions::toManifPose(
            m_kinDynWrapperList[subModelIdx]
                ->getRelativeTransform(m_kinDynWrapperList[subModelIdx]->getFloatingBase(),
                                       m_subModelList[subModelIdx].getImuBaseFrameName()));

        // Set the base velocity of the submodel from the gyroscope measurement rotating it in the base frame
        for (const auto& [key, value] : m_subModelList[subModelIdx].getGyroscopeList())
        {
            if (m_subModelList[subModelIdx].getImuBaseFrameName() == value.frame)
            {
                m_baseVelocity.coeffs().tail(3) = baseHimu.rotation() * m_gyroMap[key];
            }
        }

        m_bOmegaIB = m_baseVelocity.coeffs().tail(3);

        for (const auto& [key, value] : m_subModelList[subModelIdx].getAccelerometerList())
        {
            if (m_subModelList[subModelIdx].getImuBaseFrameName() == value.frame)
            {
                m_tempAccelerometerVelocity = Conversions::toManifTwist(
                    m_kinDynWrapperList[subModelIdx]->getFrameVel(
                    value.frame));

                // See reference: https://traversaro.github.io/traversaro-phd-thesis/traversaro-phd-thesis.pdf
                // Paragraph 4.4.2
                m_baseAcceleration.coeffs().head(3).noalias()
                    = baseHimu.rotation() * m_accMap[key]
                      - m_bOmegaIB.cross(m_bOmegaIB.cross(baseHimu.translation()));
            }
        }

        // Set robot state
        m_kinDynWrapperList[subModelIdx]->setRobotState(m_worldTBase.transform(),
                                                        m_subModelJointPos[subModelIdx],
                                                        iDynTree::make_span(m_baseVelocity.data(),
                                                        manif::SE3d::Tangent::DoF),
                                                        m_subModelJointVel[subModelIdx],
                                                        m_gravity);

        m_totalTorqueFromContacts[subModelIdx].setZero();

        // Contribution of FT measurements
        for (const auto& [key, value] : m_subModelList[subModelIdx].getFTList())
        {

            m_wrench = (int)value.forceDirection * m_FTMap[key].array();

            if (!m_kinDynWrapperList[subModelIdx]
                     ->getFrameFreeFloatingJacobian(value.frameIndex,
                                                    m_tempJacobianList[subModelIdx]))
            {
                log()->error("{} Failed while getting the jacobian for the frame `{}`.",
                             logPrefix,
                             value.frame);
                return false;
            }

            m_totalTorqueFromContacts[subModelIdx].noalias()
                += m_tempJacobianList[subModelIdx].transpose() * m_wrench;
        }

        // Contribution of unknown external contacts
        for (const auto& [key, value] : m_subModelList[subModelIdx].getExternalContactList())
        {
            if (!m_kinDynWrapperList[subModelIdx]
                     ->getFrameFreeFloatingJacobian(value.frameIndex,
                                                    m_tempJacobianList[subModelIdx]))
            {
                log()->error("{} Failed while getting the jacobian for the frame `{}`.",
                             logPrefix,
                             value.frame);
                return false;
            }

            m_totalTorqueFromContacts[subModelIdx]
                += m_tempJacobianList[subModelIdx].transpose() * m_extContactMap[key];
        }

        if (!m_kinDynWrapperList[subModelIdx]
                 ->forwardDynamics(m_subModelJointMotorTorque[subModelIdx],
                                   m_subModelFrictionTorque[subModelIdx],
                                   m_totalTorqueFromContacts[subModelIdx].tail(
                                       m_kinDynWrapperList[subModelIdx]->getNrOfDegreesOfFreedom()),
                                   m_baseAcceleration,
                                   m_subModelJointAcc[subModelIdx]))
        {
            log()->error("{} Cannot compute the inverse dynamics.", logPrefix);
            return false;
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
