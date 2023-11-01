/*
 * @file BaseEstimatorFromFootIMU.cpp
 * @authors Guglielmo Cervettini
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be
 * modified and distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/FloatingBaseEstimators/BaseEstimatorFromFootIMU.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion;

Eigen::Vector3d toXYZ(const Eigen::Matrix3d& r)
{
    Eigen::Vector3d output;
    double& thetaX = output[0]; // Roll
    double& thetaY = output[1]; // Pitch
    double& thetaZ = output[2]; // Yaw

    if (r(2, 0) < +1)
    {
        if (r(2, 0) > -1)
        {
            thetaY = asin(-r(2, 0));
            thetaZ = atan2(r(1, 0), r(0, 0));
            thetaX = atan2(r(2, 1), r(2, 2));
        } else // r20 = -1
        {
            // Not a unique solution: thetaX - thetaZ = atan2(-r12 , r11)
            thetaY = +M_PI / 2;
            thetaZ = -atan2(-r(1, 2), r(1, 1));
            thetaX = 0;
        }
    } else // r20 = +1
    {
        // Not a unique solution: thetaX + thetaZ = atan2(-r12, r11)
        thetaY = -M_PI / 2;
        thetaZ = atan2(-r(1, 2), r(1, 1));
        thetaX = 0;
    }

    return output;
}

using namespace BipedalLocomotion::Estimators;

bool BaseEstimatorFromFootIMU::setModel(const iDynTree::Model& model)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::setModel]";

    m_isModelSet = false;

    m_model = model;

    if (!m_kinDyn.loadRobotModel(model))
    {
        log()->error("{} Unable to load the model.", logPrefix);
        return false;
    }

    m_isModelSet = true;

    return true;
}

bool BaseEstimatorFromFootIMU::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::initialize]";

    m_isInitialized = false;

    if (!m_isModelSet)
    {
        log()->error("{} No iDynTree::Model has been set.", logPrefix);
        return false;
    }

    if (!m_kinDyn.isValid())
    {
        log()->error("{} iDynTree::KinDynComputations m_kinDyn is not valid.", logPrefix);
        return false;
    }

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    auto populateParameter
        = [logPrefix](std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                      const std::string& paramName,
                      auto& parameter) -> bool {
        auto ptr = handler.lock();
        if (ptr == nullptr)
        {
            log()->error("{} Invalid parameter handler for the parameter '{}'",
                         logPrefix,
                         paramName);
            return false;
        }

        if (!ptr->getParameter(paramName, parameter))
        {
            log()->error("{} Unable to find the parameter '{}'", logPrefix, paramName);
            return false;
        }
        return true;
    };

    // Base link of the robot (whose pose must be estimated)
    std::string baseFrameName;
    auto baseEstimatorPtr = ptr->getGroup("BASE_ESTIMATOR").lock();
    bool ok
        = populateParameter(baseEstimatorPtr->getGroup("MODEL_INFO"), "base_frame", baseFrameName);

    // Frame associated to the foot of the robot (whose orientation is measured)
    ok = populateParameter(baseEstimatorPtr->getGroup("MODEL_INFO"), "foot_frame", m_footFrameName);

    ok = ok && populateParameter(baseEstimatorPtr, "foot_width_in_m", m_footWidth);
    ok = ok && populateParameter(baseEstimatorPtr, "foot_length_in_m", m_footLength);

    // resetting the vector of foot corners to be sure it is correctly initialized.
    m_cornersInInertialFrame.clear();

    // Set the 4 foot vertices in World reference frame [dimensions in meters]
    m_cornersInInertialFrame.emplace_back(+m_footLength / 2, -m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(+m_footLength / 2, +m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(-m_footLength / 2, +m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(-m_footLength / 2, -m_footWidth / 2, 0);

    m_gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    m_footFrameIndex = m_kinDyn.getFrameIndex(m_footFrameName);
    if (m_footFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_footFrameName);
        return false;
    }
    m_baseFrame = m_kinDyn.getFrameIndex(baseFrameName);
    if (m_baseFrame == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, baseFrameName);
        return false;
    }

    m_frameName = m_kinDyn.getFrameName(m_footFrameIndex);
    m_linkIndex = m_model.getFrameLink(m_footFrameIndex);

    if (!m_kinDyn.setFloatingBase(m_model.getLinkName(m_linkIndex)))
    {
        log()->error("{} Unable to set the floating base.", logPrefix);
        return false;
    }

    iDynTree::Transform frame_H_link = m_model.getFrameTransform(m_footFrameIndex).inverse();
    m_frame_H_link = Conversions::toManifPose(frame_H_link);

    m_trasOld.setZero();
    m_rpyOld.setZero();
    m_yawDrift.setIdentity();
    m_state.sphereShadowCorners.resize(m_cornersInInertialFrame.size());
    m_state.sphereFootCorners.resize(m_cornersInInertialFrame.size());

    m_isInitialized = true;

    return ok;
}

bool BaseEstimatorFromFootIMU::setInput(const BaseEstimatorFromFootIMUInput& input)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::setInput]";
    m_isInputSet = false;
    m_input = input;

    if (!m_isInitialized)
    {
        log()->error("{} The estimator is not initialized.", logPrefix);
        return false;
    }

    m_isInputSet = true;
    return true;
}

bool BaseEstimatorFromFootIMU::advance()
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::advance]";

    m_isOutputValid = false;

    if (!m_isInputSet)
    {
        log()->error("{} The estimator input has not been set.", logPrefix);
        return false;
    }

    // `desiredFootPose` is intended as the output of the footstep planner.

    // extracting the position part of the `desiredFootPose`.
    m_desiredTranslation = m_input.desiredFootPose.translation();

    // extracting the orientation part of the `desiredFootPose` and expressing it
    // through RPY Euler angles.
    m_desiredRotation = m_input.desiredFootPose.rotation();
    m_desiredRPY = toXYZ(m_desiredRotation);
    m_desiredRotationCasted = manif::SO3d(m_desiredRPY(0), m_desiredRPY(1), m_desiredRPY(2));

    // casting the measured foot orientation manif::SO3d --> Eigen::Matrix3d.
    m_measuredRotation = m_input.measuredRotation.rotation();
    // expressing this orientation through RPY Euler angles.
    m_measuredRPY = toXYZ(m_measuredRotation);

    // desired Yaw is used instead of measured Yaw.
    // m_measuredRPY(2) = m_desiredRPY(2);

    // Manual correction of the measured Roll, Pitch and Yaw.
    // double temp_roll = -m_measuredRPY(1);
    // double temp_pitch = -m_measuredRPY(0);
    // m_measuredRPY(0) = temp_roll;
    // m_measuredRPY(1) = temp_pitch;
    // m_measuredRPY(2) = -m_measuredRPY(2);

    // manif::SO3d rotation matrix that employs: measured Roll, measured Pitch,
    // desired Yaw.
    m_measuredRotationCorrected = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), m_measuredRPY(2));
    m_measuredTilt = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), 0.0);
    // Dani's idea:
    // consider either roll or pitch, whichever is larger in magnitude, and set the
    // other to zero.
    // m_measuredTilt = manif::SO3d(m_measuredRPY(0), 0.0, 0.0);

    // manif::SE3d pose matrix that employs: desired Position, measured Roll,
    // measured Pitch, desired Yaw.
    Eigen::Vector3d noTras(0, 0, 0);
    manif::SE3d T_foot_raw(noTras, m_measuredRotationCorrected);
    manif::SE3d T_foot_tilt(noTras, m_measuredTilt);
    manif::SE3d T_foot(m_desiredTranslation, m_desiredRotationCasted);

    // finding the positions of the foot corners in world frame given `T_foot_raw`
    // pose matrix.

    // resetting the vector of transformed foot corners from previous iteration.
    m_transformedFootCorners.clear();

    // for each corner we compute the position in the inertial frame
    for (const auto& corner : m_cornersInInertialFrame)
    {
        m_transformedFootCorners.emplace_back(T_foot_tilt.act(corner));
    }

    // The center of the foot sole is at ground level --> some corners may be
    // under ground level. The foot may need to be translated by a 3D offset value.

    // extracting the vertical quotes of the foot corners and finding the lowest
    // corner.
    Eigen::VectorXd cornersZ;
    cornersZ.resize(m_transformedFootCorners.size());
    cornersZ.setZero();
    int index = 0;
    for (const auto& corner : m_transformedFootCorners)
    {
        cornersZ[index] = corner[2];
        index++;
    }

    // finding the index of the lowest corner.
    double minZ = cornersZ[0];
    int indexMinZ = 0;
    for (int i = 1; i < cornersZ.size(); i++)
    {
        if (cornersZ[i] < minZ)
        {
            minZ = cornersZ[i];
            indexMinZ = i;
        }
    }
    m_state.supportCornerIndex = indexMinZ;

    // finding the index of the highest corner.
    // double maxZ = cornersZ[0];
    // int highestCornerIndex = 0;
    // for (int i = 1; i < cornersZ.size(); i++)
    // {
    //     if (cornersZ[i] > maxZ)
    //     {
    //         maxZ = cornersZ[i];
    //         highestCornerIndex = i;
    //     }
    // }

    // double deltaZ = cornersZ[highestCornerIndex] - cornersZ[m_state.supportCornerIndex];
    // std::cerr << "Foot deltaZ: " << deltaZ << std::endl;

    // checking that the index of the lowest corner is within the range [0, 3].
    if (!(0 <= m_state.supportCornerIndex <= 3))
    {
        log()->error("{} Foot vertex index out of bounds (0, 3): {}.",
                     logPrefix,
                     m_state.supportCornerIndex);
        return false;
    }

    // finding the offset vector needed to bring the lowest corner back to its
    // desired position.
    Eigen::Vector3d p_desired(0, 0, 0);
    Eigen::Vector3d vertexOffset(0, 0, 0);
    // p_desired = m_yawDrift.act(m_cornersInInertialFrame[m_state.supportCornerIndex]);
    // vertexOffset = p_desired -
    // m_yawDrift.act(m_transformedFootCorners[m_state.supportCornerIndex]);

    p_desired = (m_cornersInInertialFrame[m_state.supportCornerIndex]);
    vertexOffset = p_desired - (m_transformedFootCorners[m_state.supportCornerIndex]);

    // std::cerr << "shadowCorners: " << p_desired.transpose() << std::endl;
    // std::cerr << "footCorners: "
    //           << m_yawDrift.act(m_transformedFootCorners[m_state.supportCornerIndex]).transpose()
    //           << std::endl;
    // std::cerr << "vertexOffset: " << vertexOffset.transpose() << std::endl;
    // if (vertexOffset.norm() > 0.1)
    // {
    //     log()->error("{} Foot vertex offset too large: {}.", logPrefix, vertexOffset.norm());
    //     return false;
    // }
    // transforming the offset vector into a translation matrix.
    manif::SE3d T_vertexOffset(vertexOffset, manif::SO3d::Identity());
    // manif::SE3d T_vertexOffset(manif::SE3d::Identity());
    for (int i = 0; i < m_cornersInInertialFrame.size(); i++)
    {
        m_state.sphereShadowCorners[i] = m_yawDrift.act(m_cornersInInertialFrame[i]);
        m_state.sphereFootCorners[i]
            = T_vertexOffset.act(m_yawDrift.act(m_transformedFootCorners[i]));
    }

    // obtaining the final foot pose using both measured and desired quantities.
    // cordinate change is performed from foot sole frame to foot link frame.
    m_measuredFootPose = T_foot * m_yawDrift * T_vertexOffset * T_foot_tilt * m_frame_H_link;
    m_resetFootCorners = T_foot_raw;

    Eigen::VectorXd baseVelocity(6);
    baseVelocity.setZero();

    // setting the robot state in terms of stance foot pose and joint positions.
    if (!m_kinDyn.setRobotState(m_measuredFootPose.transform(),
                                m_input.jointPositions,
                                baseVelocity,
                                m_input.jointVelocities,
                                m_gravity))
    {
        log()->error("{} Unable to set the robot state.", logPrefix);
        return false;
    }

    // calculating the pose of the root link given the robot state.
    m_state.basePose = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_baseFrame));
    m_kinDyn.getCenterOfMassPosition(m_state.centerOfMassPosition);

    // updating m_cornersInLocalFrame when the foot is nearly flat
    // double flatnessThreshold = 0.0001;
    // if (deltaZ <= flatnessThreshold)
    // extracting the position part of the reset corners pose.
    Eigen::Vector3d deltaTras(0, 0, 0);
    Eigen::Vector3d deltaRPY(0, 0, 0);

    auto tras = m_resetFootCorners.translation();
    // foot z is completely canceled
    tras(2) = 0.0;
    deltaTras = tras - m_trasOld;
    m_trasOld = tras;
    // extracting the orientation part of the reset corners pose and expressing it through RPY
    // Euler angles.
    auto rot = m_resetFootCorners.rotation();
    auto rpy = toXYZ(rot);
    // Roll and Pitch are completely canceled.
    rpy(0) = 0.0;
    rpy(1) = 0.0;
    deltaRPY = rpy - m_rpyOld;
    m_rpyOld = rpy;
    // manif::SO3d rotation matrix that employs: zero Roll, zero Pitch, measured Yaw.
    auto deltaYaw = manif::SO3d(deltaRPY(0), deltaRPY(1), deltaRPY(2));
    // manif::SE3d reset corners pose matrix.
    // manif::SE3d T_reset_corners(deltaTras, deltaYaw);
    Eigen::Vector3d tr(0, 0, 0);
    manif::SE3d T_reset_corners(tr, deltaYaw);

    std::vector<Eigen::Vector3d> tempCorners;
    tempCorners.resize(m_cornersInInertialFrame.size());
    int j = 0;
    for (const auto& corner : m_cornersInInertialFrame)
    {
        tempCorners[j] = T_reset_corners.act(corner);
        j++;
    }
    Eigen::Vector3d resetOffset(0, 0, 0);
    resetOffset = m_cornersInInertialFrame[m_state.supportCornerIndex]
                  - tempCorners[m_state.supportCornerIndex];
    // std::cerr << "resetOffset: " << resetOffset.transpose() << std::endl;
    // manif::SE3d T_resetOffset(resetOffset, manif::SO3d::Identity());
    manif::SE3d T_resetOffset(resetOffset, deltaYaw);

    manif::SE3d temp;
    temp = T_resetOffset * m_yawDrift;
    m_yawDrift = temp;

    // j = 0;
    // for (const auto& corner : tempCorners)
    // {
    //     m_cornersInInertialFrame[j] = T_resetOffset.act(corner);
    //     j++;
    // }

    m_isOutputValid = true;

    return true;
}

bool BaseEstimatorFromFootIMU::isOutputValid() const
{
    return m_isOutputValid;
}

const BaseEstimatorFromFootIMUState& BaseEstimatorFromFootIMU::getOutput() const
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::getOutput]";

    if (!m_isOutputValid)
    {
        log()->error("{} The output is not valid.", logPrefix);
    }

    return m_state; // m_state.basePose is the actual output
}

// OPTIONAL METHOD - TO FORCE THE STATE

void BaseEstimatorFromFootIMU::setState(const BaseEstimatorFromFootIMUState& state)
{
    m_state = state;
}
