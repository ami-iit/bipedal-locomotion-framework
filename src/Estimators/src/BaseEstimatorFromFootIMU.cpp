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

Eigen::Vector3d toXYZ(Eigen::Ref<const Eigen::Matrix3d> r)
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

    if (!m_kinDyn.loadRobotModel(model))
    {
        log()->error("{} Unable to load the model.", logPrefix);
        return false;
    }

    m_model = model;

    return true;
}

bool BaseEstimatorFromFootIMU::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::initialize]";

    m_isInitialized = false;

    if (!m_model.isValid())
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
    // Frame associated to the base of the robot (whose pose is estimated)
    bool ok = populateParameter(ptr->getGroup("MODEL_INFO"), "base_frame", m_baseFrameName);

    // Frame associated to the left foot of the robot (whose orientation is measured)
    ok = ok && populateParameter(ptr->getGroup("MODEL_INFO"), "foot_frame_L", m_footFrameName_L);
    // Frame associated to the right foot of the robot (whose orientation is measured)
    ok = ok && populateParameter(ptr->getGroup("MODEL_INFO"), "foot_frame_R", m_footFrameName_R);
    // Foot dimensions. Same for both feet.
    ok = ok && populateParameter(ptr, "foot_width_in_m", m_footWidth);
    ok = ok && populateParameter(ptr, "foot_length_in_m", m_footLength);

    if (!ok)
    {
        log()->error("{} Unable to retrieve all the parameters.", logPrefix);
        return false;
    }

    m_gravity << 0, 0, -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    m_baseFrameIndex = m_kinDyn.getFrameIndex(m_baseFrameName);
    if (m_baseFrameIndex == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_baseFrameName);
        return false;
    }

    m_footFrameIndex_L = m_kinDyn.getFrameIndex(m_footFrameName_L);
    if (m_footFrameIndex_L == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_footFrameName_L);
        return false;
    }
    iDynTree::Transform frame_H_link_L = m_model.getFrameTransform(m_footFrameIndex_L).inverse();
    m_footFrame_H_link_L = Conversions::toManifPose(frame_H_link_L);

    m_footFrameIndex_R = m_kinDyn.getFrameIndex(m_footFrameName_R);
    if (m_footFrameIndex_R == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} Invalid frame named: {}", logPrefix, m_footFrameName_R);
        return false;
    }
    iDynTree::Transform frame_H_link_R = m_model.getFrameTransform(m_footFrameIndex_R).inverse();
    m_footFrame_H_link_R = Conversions::toManifPose(frame_H_link_R);

    // resetting the vector of foot corners to be sure it is correctly initialized.
    m_cornersInInertialFrame.clear();

    // Set the 4 foot vertices in World reference frame [dimensions in meters]
    m_cornersInInertialFrame.emplace_back(+m_footLength / 2, -m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(+m_footLength / 2, +m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(-m_footLength / 2, +m_footWidth / 2, 0);
    m_cornersInInertialFrame.emplace_back(-m_footLength / 2, -m_footWidth / 2, 0);

    m_yawOld = 0.0;
    m_T_yawDrift.setIdentity();
    m_measuredFootPose.setIdentity();
    m_T_walk.setIdentity();
    m_state.stanceFootShadowCorners.resize(m_cornersInInertialFrame.size());
    m_state.stanceFootCorners.resize(m_cornersInInertialFrame.size());

    m_isInitialized = true;

    return ok;
}

bool BaseEstimatorFromFootIMU::setInput(const BaseEstimatorFromFootIMUInput& input)
{
    constexpr auto logPrefix = "[BaseEstimatorFromFootIMU::setInput]";
    m_isInputSet = false;

    if (!m_isInitialized)
    {
        log()->error("{} The estimator must be initialized before setting the input.", logPrefix);
        return false;
    }

    m_input = input;
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

    // checking the stance foot
    if (m_input.isLeftStance && m_input.isRightStance)
    {
        log()->error("{} Both feet are stance feet. The estimator accept only one stance foot.",
                     logPrefix);
        return false;
    }
    if (!m_input.isLeftStance && !m_input.isRightStance)
    {
        log()->error("{} No stance foot set. The estimator needs one stance foot in input.",
                     logPrefix);
        return false;
    }

    const manif::SE3d& stanceFootFrame_H_link = m_input.isLeftStance ? m_footFrame_H_link_L
                                                                     : m_footFrame_H_link_R;
    const int stanceFootFrameIndex = m_input.isLeftStance ? m_footFrameIndex_L : m_footFrameIndex_R;
    // TODO check m_measuredRotation
    const manif::SO3d& measuredRotation = m_input.isLeftStance ? m_input.measuredRotation_L
                                                               : m_input.measuredRotation_R;

    const manif::SO3Tangentd& measuredVelocity = m_input.isLeftStance
                                                     ? m_input.measuredAngularVelocity_L
                                                     : m_input.measuredAngularVelocity_R;

    m_stanceLinkIndex = m_model.getFrameLink(stanceFootFrameIndex);
    if (!m_kinDyn.setFloatingBase(m_model.getLinkName(m_stanceLinkIndex)))
    {
        log()->error("{} Unable to set the stance foot as floating base.", logPrefix);
        return false;
    }

    // yaw removed from the measured rotation
    m_measuredRPY = toXYZ(measuredRotation.rotation());
    auto desiredFootRPY = toXYZ(m_input.stanceFootPose.rotation());

    auto cleanedRotation = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), desiredFootRPY(2));


    const manif::SE3d I_H_foot_rotation = manif::SE3d(m_input.stanceFootPose.translation(), cleanedRotation);

    // expressing measured orientation through RPY Euler angles.
   


    // get the tilt only
    m_measuredTilt = manif::SO3d(m_measuredRPY(0), m_measuredRPY(1), 0.0);

    // manif::SE3d pose matrix that employs: offset Position, measured Roll, measured Pitch, offset
    // Yaw.
    manif::SE3d T_foot_tilt(m_noTras, m_measuredTilt);

    // TODO this can be simplified
    // resetting the vector of transformed foot corners from previous iteration.
    m_tiltedFootCorners.clear();
    // for each corner we compute the position in the inertial frame
    for (const auto& corner : m_cornersInInertialFrame)
    {
        m_tiltedFootCorners.emplace_back(T_foot_tilt.act(corner));
    }

    // The center of the foot sole is at ground level --> some corners may be
    // under ground level. The foot may need to be translated by a 3D offset value.

    // extracting the vertical quotes of the foot corners and finding the lowest
    // corner.

    // find the lowest corner
    m_state.supportCornerIndex
        = std::distance(m_tiltedFootCorners.begin(),
                        std::min_element(m_tiltedFootCorners.begin(),
                                         m_tiltedFootCorners.end(),
                                         [](const auto& a, const auto& b) { return a[2] < b[2]; }));

    // finding the translation vector needed to bring the lowest corner back to its
    // untilted position.
    const Eigen::Vector3d supportCornerTranslation
        = m_cornersInInertialFrame[m_state.supportCornerIndex]
          - m_tiltedFootCorners[m_state.supportCornerIndex]; // TODO: change if
                                                             // flat ground
                                                             // assumption is
                                                             // removed

    // transforming the offset vector into a translation matrix.
    manif::SE3d T_supportCornerTranslation(supportCornerTranslation, manif::SO3d::Identity());

    // obtaining the final foot pose using both measured and offset quantities.
    // cordinate change is performed from foot sole frame to foot link frame.
    m_measuredFootPose = I_H_foot_rotation * T_supportCornerTranslation * stanceFootFrame_H_link;

    Eigen::VectorXd baseVelocity(6);

    // the linear velocity is wrong
    auto angularVelocityInLinkFrame = stanceFootFrame_H_link.asSO3().inverse().act(measuredVelocity.coeffs());
    auto cornerInLinkFrame = stanceFootFrame_H_link.inverse().act(
        m_cornersInInertialFrame[m_state.supportCornerIndex]);

    baseVelocity.head<3>()
        = m_measuredFootPose.asSO3().act(cornerInLinkFrame.cross(angularVelocityInLinkFrame));
    baseVelocity.tail<3>() = cleanedRotation.act(measuredVelocity.coeffs());

    // setting the robot state in terms of stance foot pose and joint positions.
    if (!m_kinDyn.setRobotState(m_measuredFootPose.transform(),
                                m_input.jointPositions,
                                baseVelocity,
                                m_input.jointVelocities,
                                m_gravity))
    {
        log()->error("{} Unable to set the robot state from the stance foot pose.", logPrefix);
        return false;
    }

    // calculating the output of the estimator given the robot state.
    m_state.basePose = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_baseFrameIndex));
    m_state.footPose_L = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_footFrameIndex_L));
    m_state.footPose_R = Conversions::toManifPose(m_kinDyn.getWorldTransform(m_footFrameIndex_R));
    m_state.baseVelocity = Conversions::toManifTwist(m_kinDyn.getFrameVel(m_baseFrameIndex));
    m_state.footVelocity_L = Conversions::toManifTwist(m_kinDyn.getFrameVel(m_footFrameIndex_L));
    m_state.footVelocity_R = Conversions::toManifTwist(m_kinDyn.getFrameVel(m_footFrameIndex_R));

    for (int i = 0; i < m_cornersInInertialFrame.size(); i++)
    {
        m_state.stanceFootShadowCorners[i]
            = m_T_walk.act(m_T_yawDrift.act(m_cornersInInertialFrame[i]));
        m_state.stanceFootCorners[i] = m_T_walk.act(
            m_T_yawDrift.act(T_supportCornerTranslation.act(m_tiltedFootCorners[i])));
    }

    double orientationError_L
        = (toXYZ(m_state.footPose_L.rotation()) - toXYZ(m_input.measuredRotation_L.rotation()))
              .norm();
    double orientationError_R
        = (toXYZ(m_state.footPose_R.rotation()) - toXYZ(m_input.measuredRotation_R.rotation()))
              .norm();
    double orientationErrorThreshold = 0.01; // [rad]. TODO: get parameter from config file. 0,01
                                             // rad = 0,5729578 deg.

    // if ((orientationError_L > orientationErrorThreshold)
    //     || (orientationError_R > orientationErrorThreshold))
    // {
    //     log()->warn("{} Foot orientation error above {} threshold: {} Left, {} Right.",
    //                 logPrefix,
    //                 orientationErrorThreshold,
    //                 orientationError_L,
    //                 orientationError_R);
    // }
    // std::cerr << "L FOOT ROTATION IN: " <<
    // toXYZ(m_input.measuredRotation_L.rotation()).transpose() << std::endl; std::cerr << "L FOOT
    // ROTATION OUT: " << toXYZ(m_state.footPose_L.rotation()).transpose() << std::endl; std::cerr
    // << "R FOOT ROTATION IN: " << toXYZ(m_input.measuredRotation_R.rotation()).transpose() <<
    // std::endl; std::cerr << "R FOOT ROTATION OUT: " <<
    // toXYZ(m_state.footPose_R.rotation()).transpose() << std::endl;

    // updating the stance foot flags.
    if (m_input.isLeftStance)
    {
        m_isLastStanceFoot_L = true;
        m_isLastStanceFoot_R = false;
    }
    if (m_input.isRightStance)
    {
        m_isLastStanceFoot_L = false;
        m_isLastStanceFoot_R = true;
    }

    m_isOutputValid = true;

    return true;
}

bool BaseEstimatorFromFootIMU::isOutputValid() const
{
    return m_isOutputValid;
}

const BaseEstimatorFromFootIMUState& BaseEstimatorFromFootIMU::getOutput() const
{
    return m_state; // m_state.basePose is the actual output
}

// OPTIONAL METHOD - TO FORCE THE STATE

void BaseEstimatorFromFootIMU::setState(const BaseEstimatorFromFootIMUState& state)
{
    m_state = state;
}
