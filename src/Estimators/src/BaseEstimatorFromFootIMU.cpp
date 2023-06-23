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

Eigen::Vector3d toZYX(const Eigen::Matrix3d& r)
{
    Eigen::Vector3d output;
    double& thetaZ = output[0]; // Roll
    double& thetaY = output[1]; // Pitch
    double& thetaX = output[2]; // Yaw

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
    bool ok = populateParameter(ptr->getGroup("MODEL_INFO"), "base_frame", baseFrameName);

    // Frame associated to the foot of the robot (whose orientation is measured)
    ok = populateParameter(ptr->getGroup("MODEL_INFO"), "foot_frame", m_footFrameName);

    ok = ok && populateParameter(ptr, "foot_width_in_m", m_footWidth);
    ok = ok && populateParameter(ptr, "foot_length_in_m", m_footLength);

    // Set the 4 foot vertices in World reference frame [dimensions in meters]
    m_cornersInLocalFrame.emplace_back(+m_footWidth / 2, +m_footLength / 2, 0);
    m_cornersInLocalFrame.emplace_back(-m_footWidth / 2, +m_footLength / 2, 0);
    m_cornersInLocalFrame.emplace_back(-m_footWidth / 2, -m_footLength / 2, 0);
    m_cornersInLocalFrame.emplace_back(+m_footWidth / 2, -m_footLength / 2, 0);

    m_gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    m_frameIndex = m_kinDyn.getFrameIndex(m_footFrameName);
    if (m_frameIndex == iDynTree::FRAME_INVALID_INDEX)
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

    m_frameName = m_kinDyn.getFrameName(m_frameIndex);
    m_linkIndex = m_model.getFrameLink(m_frameIndex);

    if (!m_kinDyn.setFloatingBase(m_model.getLinkName(m_linkIndex)))
    {
        log()->error("{} Unable to set the floating base.", logPrefix);
        return false;
    }

    iDynTree::Transform frame_H_link = m_model.getFrameTransform(m_frameIndex).inverse();
    m_frame_H_link = Conversions::toManifPose(frame_H_link);

    m_isInitialized = true;

    return ok;
}

bool BaseEstimatorFromFootIMU::setInput(const BaseEstimatorFromFootIMUInput& input)
{
    m_input = input;
    return true;
}

bool BaseEstimatorFromFootIMU::advance()
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::advance]";

    m_isOutputValid = false;

    if (!m_isInitialized)
    {
        log()->error("{} The estimator is not initialized.", logPrefix);
        return false;
    }

    // `desiredFootPose` is intended as the output of the footstep planner.

    // extracting the position part of the `desiredFootPose`.
    m_desiredTranslation = m_input.desiredFootPose.translation();

    // extracting the orientation part of the `desiredFootPose` and expressing it
    // through RPY Euler angles.
    m_desiredRotation = m_input.desiredFootPose.rotation();
    m_desiredRPY = toZYX(m_desiredRotation);

    // casting the measured foot orientation manif::SO3d --> Eigen::Matrix3d.
    m_measuredRotation = m_input.measuredRotation.rotation();
    // expressing this orientation through RPY Euler angles.
    m_measuredRPY = toZYX(m_measuredRotation);

    // desired Yaw is used instead of measured Yaw.
    m_measuredRPY(2) = m_desiredRPY(2);

    // manif::SO3d rotation matrix that employs: measured Roll, measured Pitch,
    // desired Yaw.
    m_measuredRotationCorrected = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), m_measuredRPY(2));

    // manif::SE3d pose matrix that employs: desired Position, measured Roll,
    // measured Pitch, desired Yaw.
    manif::SE3d T_foot_raw(m_desiredTranslation, m_measuredRotationCorrected);

    // finding the positions of the foot corners in world frame given `T_foot_raw`
    // pose matrix.

    // for each corner we compute the position in the inertial frame
    for (const auto& corner : m_cornersInLocalFrame)
    {
        m_transformedFootCorners.emplace_back(T_foot_raw.act(corner));
    }

    // The center of the foot sole is at ground level --> some corners may be
    // under ground level. The foot may need to be lifted by an offset value.

    // extracting the vertical quotes of the foot corners and finding the lowest
    // corner.
    auto lowestCorner = std::min_element(m_transformedFootCorners.begin(),
                                         m_transformedFootCorners.end(),
                                         [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                                             return a[2] < b[2];
                                         });

    // finding the index of the lowest corner.
    int supportCornerIndex = std::distance(m_transformedFootCorners.begin(), lowestCorner);

    Eigen::Vector3d p_desired(0, 0, 0);
    Eigen::Vector3d vertexOffset(0, 0, 0);

    // finding the offset vector needed to bring the lowest corner back to its
    // desired position.
    switch (supportCornerIndex)
    {
    case 0:
        p_desired = m_input.desiredFootPose.act(m_cornersInLocalFrame[supportCornerIndex]);
        vertexOffset = p_desired - m_transformedFootCorners[supportCornerIndex];
        break;
    case 1:
        p_desired = m_input.desiredFootPose.act(m_cornersInLocalFrame[supportCornerIndex]);
        vertexOffset = p_desired - m_transformedFootCorners[supportCornerIndex];
        break;
    case 2:
        p_desired = m_input.desiredFootPose.act(m_cornersInLocalFrame[supportCornerIndex]);
        vertexOffset = p_desired - m_transformedFootCorners[supportCornerIndex];
        break;
    case 3:
        p_desired = m_input.desiredFootPose.act(m_cornersInLocalFrame[supportCornerIndex]);
        vertexOffset = p_desired - m_transformedFootCorners[supportCornerIndex];
        break;
    default:
        log()->error("{} Foot vertex index out of bounds (0, 3): {}.",
                     logPrefix,
                     supportCornerIndex);
        return false;
    }

    // transforming the offset vector into a translation matrix.
    manif::SE3d T_vertexOffset(vertexOffset, manif::SO3d::Identity());

    // obtaining the final foot pose using both measured and desired quantities.
    // cordinate change is performed from foot sole frame to foot link frame.
    m_measuredFootPose = T_vertexOffset * T_foot_raw * m_frame_H_link;

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
