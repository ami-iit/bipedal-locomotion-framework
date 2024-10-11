/**
 * @file LeggedOdometry.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <chrono>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/Model.h>
#include <manif/manif.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion::Conversions;
using namespace BipedalLocomotion::Contacts;

namespace BipedalLocomotion
{
    namespace Estimators
    {
        enum class LOSwitching
        {
            latest, /**< recently switched contact frame index */
            lastActive, /**< earliest switched contact frame index */
            useExternalUpdate /**< update the fixed frame externally*/
        };

        enum class LOVelComputation
        {
            singleFrame,  /**< use fixed frame holonomic constraint for base velocity computation */
            multiFrameAverage, /**< use all fixed frame holonomic constraint average contribution to base velocity computation */
            multiFrameLeastSquare, /**< use all fixed frame holonomic constraint for base velocity computation usig least square fit */
            multiFrameLeastSquareWithJVel /**< use all fixed frame holonomic constraint for base velocity computation usig least square fit with joint velocity regularization*/
        };
    }
}

class LeggedOdometry::Impl
{
public:
    /**
     *  Change fixed frame for the legged odometry
     */
    bool changeFixedFrame(const iDynTree::FrameIndex& newIdx,
                          std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
    /**
     *  Update internal state of the legged odometry block
     */
    bool updateInternalState(FloatingBaseEstimators::Measurements& meas,
                             FloatingBaseEstimator::ModelComputations& modelComp,
                             FloatingBaseEstimators::InternalState& state,
                             FloatingBaseEstimators::Output& out);
    /**
     *  Update internal contact states
     */
    void updateInternalContactStates(FloatingBaseEstimators::Measurements& meas,
                                     FloatingBaseEstimator::ModelComputations& modelComp,
                                     std::map<int, EstimatedContact>& contacts);
    iDynTree::FrameIndex getLatestSwitchedContact(const std::map<int, EstimatedContact>& contacts);
    iDynTree::FrameIndex getLastActiveContact(const std::map<int, EstimatedContact>& contacts,  const int& activeIndex);

    /**
     *  Single fixed frame
     */
    bool computeBaseVelocityUsingFixedFrameConstraint(FloatingBaseEstimators::Measurements& meas,
                                                         FloatingBaseEstimator::ModelComputations& modelComp,
                                                         FloatingBaseEstimators::Output& out);
    /**
     * All fixed frame least square solution with joint velocities in the solution vector
     */
    bool computeBaseVelocityUsingAllFixedFrameConstraint(const FloatingBaseEstimators::InternalState& state,
                                                         FloatingBaseEstimators::Measurements& meas,
                                                         FloatingBaseEstimator::ModelComputations& modelComp,
                                                         FloatingBaseEstimators::Output& out);
    /**
     * All fixed frames average
     */
    bool computeBaseVelocityUsingAllFixedFrameAverage(const FloatingBaseEstimators::InternalState& state,
                                                      FloatingBaseEstimators::Measurements& meas,
                                                      FloatingBaseEstimator::ModelComputations& modelComp,
                                                      FloatingBaseEstimators::Output& out);

    /**
     * All fixed frame least square solution without joint velocities in the solution vector
     */
    bool computeBaseVelocityUsingAllFixedFrames(const FloatingBaseEstimators::InternalState& state,
                                                FloatingBaseEstimators::Measurements& meas,
                                                FloatingBaseEstimator::ModelComputations& modelComp,
                                                FloatingBaseEstimators::Output& out);

    /**
     * Reset internal state of the legged odometry block
     */
    void resetInternal(FloatingBaseEstimator::ModelComputations& modelComp,
                       const FloatingBaseEstimators::Measurements& meas);

    // parameters
    std::string m_initialFixedFrame; /**<  Fixed frame at initialization assumed to be in rigid contact with the environment*/
    std::string m_initialRefFrameForWorld; /**< Reference frame for the world at initialization */
    iDynTree::FrameIndex m_initialFixedFrameIdx{iDynTree::FRAME_INVALID_INDEX};  /**< Index of fixed frame at initialization */
    iDynTree::FrameIndex m_initialRefFrameForWorldIdx{iDynTree::FRAME_INVALID_INDEX};  /**< Index of world reference frame at initialization */
    manif::SE3d m_refFrame_H_world; /**< pose of world wrt reference frame at initialization */

    bool m_odometryInitialized{false}; /**< flag to check if odometry was initialized */

    iDynTree::FrameIndex m_prevFixedFrameIdx{iDynTree::FRAME_INVALID_INDEX};
    iDynTree::FrameIndex m_currentFixedFrameIdx{iDynTree::FRAME_INVALID_INDEX};

    manif::SE3d m_world_H_fixedFrame; /**< Pose of fixed frame wrt world */
    LOSwitching switching{LOSwitching::latest};  /**< contact switching pattern */
    LOVelComputation velComp{LOVelComputation::multiFrameLeastSquare}; /**< base velocity estimation method*/

    Eigen::MatrixXd m_contactJacobian; /**< mixed velocity jacobian of fixed frame with respect to floating base */
    Eigen::MatrixXd m_contactJacobianBase; /**< base sub-block of mixed velocity jacobian of fixed frame with respect to floating base */
    Eigen::MatrixXd m_contactJacobianShape; /**< shape sub-block of mixed velocity jacobian of fixed frame with respect to floating base */
    Eigen::VectorXd m_vBase; /**< mixed velocity representation of base link */
    const int m_spatialDim{6};
    const int m_baseOffset{0}; /**< offset in contact Jacobian for the base sub-block*/
    const int m_shapeOffset{6}; /**< offset in contact Jacobian for the shape sub-block*/

    // for velocity computations
    double wLin{1.0};
    double wAng{0.5};
    double wJVel{10.0};
    double reg{1e-6};
    Eigen::VectorXd m_sDotFilt;
};

LeggedOdometry::LeggedOdometry() : m_pimpl(std::make_unique<Impl>())
{
    m_state.imuOrientation.setIdentity();
    m_state.imuPosition.setZero();
    m_state.imuLinearVelocity.setZero();
    m_state.rContactFrameOrientation.setIdentity();
    m_state.rContactFramePosition.setZero();
    m_state.lContactFrameOrientation.setIdentity();
    m_state.lContactFramePosition.setZero();
    m_state.accelerometerBias.setZero();
    m_state.gyroscopeBias.setZero();
    m_state.imuAngularVelocity.setZero();

    m_useIMUForAngVelEstimate = false;
    m_useIMUVelForBaseVelComputation = false;

    m_statePrev = m_state;
    m_estimatorOut.state = m_state;

    m_estimatorOut.baseTwist.setZero();
    m_estimatorOut.basePose.setIdentity();

    m_meas.acc.setZero();   // unused
    m_meas.gyro.setZero();  // unused
    m_meas.lfInContact = false;
    m_meas.rfInContact = false;

    m_pimpl->m_vBase.resize(m_pimpl->m_spatialDim);
    m_pimpl->m_vBase.setZero();

    m_measPrev = m_meas;
}

LeggedOdometry::~LeggedOdometry() = default;

bool LeggedOdometry::customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    const std::string_view printPrefix = "[LeggedOdometry::customInitialization] ";
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << printPrefix << "The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    auto loHandler = handle->getGroup("LeggedOdom");
    auto lohandle = loHandler.lock();

    if (!lohandle->getParameter("initial_fixed_frame", m_pimpl->m_initialFixedFrame))
    {
        std::cerr <<  printPrefix <<
        "The parameter handler could not find \" initial_fixed_frame \" in the configuration file."
        << std::endl;
        return false;
    }
    else
    {
        m_pimpl->m_initialFixedFrameIdx = m_modelComp.kinDyn()->model().getFrameIndex(m_pimpl->m_initialFixedFrame);
        if (m_pimpl->m_initialFixedFrameIdx == iDynTree::FRAME_INVALID_INDEX)
        {
            std::cerr << printPrefix << "Specified fixed frame not available in the loaded URDF Model."
            << std::endl;
            return false;
        }
    }

    std::vector<double> initialWorldOrientationInRefFrame{1., 0., 0., 0.};
    std::vector<double> initialWorldPositionInRefFrame{0., 0., 0.};
    if (!lohandle->getParameter("initial_ref_frame_for_world", m_pimpl->m_initialRefFrameForWorld))
    {
        std::cerr << printPrefix <<
        "The parameter handler could not find \"initial_ref_frame_for_world \" in the configuration file. Setting \"initial_fixed_frame\" as reference frame for world"
        << std::endl;
        m_pimpl->m_initialRefFrameForWorld = m_pimpl->m_initialFixedFrame;
        m_pimpl->m_initialRefFrameForWorldIdx = m_pimpl->m_initialFixedFrameIdx;
    }
    else
    {
        m_pimpl->m_initialRefFrameForWorldIdx = m_modelComp.kinDyn()->model().getFrameIndex(m_pimpl->m_initialRefFrameForWorld);
        if (m_pimpl->m_initialRefFrameForWorldIdx == iDynTree::FRAME_INVALID_INDEX)
        {
            std::cerr << printPrefix << "Specified reference frame for world not available in the loaded URDF Model."
            << std::endl;
            return false;
        }

        // setup initial states
        if (!lohandle->getParameter("initial_world_orientation_in_ref_frame", GenericContainer::make_vector(initialWorldOrientationInRefFrame, GenericContainer::VectorResizeMode::Fixed)))
        {
            std::cerr <<  printPrefix << "The parameter handler could not find \"initial_world_orientation_in_ref_frame\" in the configuration file. Setting to identity" << std::endl;
        }

        if (!lohandle->getParameter("initial_world_position_in_ref_frame", GenericContainer::make_vector(initialWorldPositionInRefFrame, GenericContainer::VectorResizeMode::Fixed)))
        {
            std::cerr << printPrefix << "The parameter handler could not find \"initial_world_position_in_ref_frame\" in the configuration file. Setting to zero." << std::endl;
        }
    }

    std::string switching;
    if (lohandle->getParameter("switching_pattern", switching))
    {
        std::vector<std::string> options{"latest", "lastActive", "useExternal"};
        if (switching == options[0])
        {
            m_pimpl->switching = LOSwitching::latest;
        }
        else if (switching == options[1])
        {
            m_pimpl->switching = LOSwitching::lastActive;
        }
        else if (switching == options[2])
        {
            m_pimpl->switching = LOSwitching::useExternalUpdate;
        }
        else
        {
        	std::cerr << printPrefix << "Setting the switching of fixed frame to default, based on latest active contacts." << std::endl;
        }
    }

    std::string velComp;
    if (lohandle->getParameter("vel_computation_method", velComp))
    {
        std::vector<std::string> options{"single", "multiAvg", "multLSJvel", "multiLS"};
        if (velComp == options[0])
        {
            m_pimpl->velComp = LOVelComputation::singleFrame;
        }
        else if (velComp == options[1])
        {
            m_pimpl->velComp = LOVelComputation::multiFrameAverage;
        }
        else if (velComp == options[2])
        {
            m_pimpl->velComp = LOVelComputation::multiFrameLeastSquareWithJVel;
        }
        else
        {
            m_pimpl->velComp = LOVelComputation::multiFrameLeastSquare;
        }
    }

    if (!lohandle->getParameter("wLin", m_pimpl->wLin))
    {
        m_pimpl->wLin = 1.0;
    }

    if (!lohandle->getParameter("wAng", m_pimpl->m_initialFixedFrame))
    {
        m_pimpl->wAng = 0.5;
    }

    if (!lohandle->getParameter("wJVel", m_pimpl->m_initialFixedFrame))
    {
        m_pimpl->wJVel = 10.0;
    }

    if (!lohandle->getParameter("reg", m_pimpl->m_initialFixedFrame))
    {
        m_pimpl->reg = 1e-6;
    }

    Eigen::Quaterniond refFrame_quat_world = Eigen::Quaterniond(initialWorldOrientationInRefFrame[0], initialWorldOrientationInRefFrame[1],
                                                                initialWorldOrientationInRefFrame[2], initialWorldOrientationInRefFrame[3]); // here loaded as w x y z
    refFrame_quat_world.normalize();
    Eigen::Vector3d refFrame_p_world;
    refFrame_p_world << initialWorldPositionInRefFrame[0], initialWorldPositionInRefFrame[1], initialWorldPositionInRefFrame[2];
    m_pimpl->m_refFrame_H_world = manif::SE3d(refFrame_p_world, refFrame_quat_world);

    auto nrJoints = m_modelComp.kinDyn()->getNrOfDegreesOfFreedom();
    m_pimpl->m_contactJacobian.resize(m_pimpl->m_spatialDim, m_pimpl->m_spatialDim + nrJoints);
    m_pimpl->m_contactJacobianBase.resize(m_pimpl->m_spatialDim, m_pimpl->m_spatialDim);
    m_pimpl->m_contactJacobianShape.resize(m_pimpl->m_spatialDim, nrJoints);
    m_pimpl->m_contactJacobian.setZero();
    m_pimpl->m_contactJacobianBase.setZero();
    m_pimpl->m_contactJacobianShape.setZero();
    return true;
}

void LeggedOdometry::Impl::updateInternalContactStates(FloatingBaseEstimators::Measurements& meas,
                                                       FloatingBaseEstimator::ModelComputations& modelComp,
                                                       std::map<int, EstimatedContact>& contactStates)
{
    for (const auto& obs : meas.stampedContactsStatus)
    {
        const auto& measContact = obs.second;
        const auto& idx = obs.first;

        if (contactStates.find(idx) != contactStates.end())
        {
            contactStates.at(idx).setContactStateStamped({measContact.isActive, measContact.switchTime});
            contactStates.at(idx).lastUpdateTime = measContact.lastUpdateTime;
        }
        else
        {
            BipedalLocomotion::Contacts::EstimatedContact newContact;
            newContact.setContactStateStamped({measContact.isActive, measContact.switchTime});
            newContact.lastUpdateTime = measContact.lastUpdateTime;
            newContact.index = idx;
            newContact.name = modelComp.kinDyn()->getFrameName(idx);
            contactStates[idx] = newContact;
        }
    }
    meas.stampedContactsStatus.clear();
}

iDynTree::FrameIndex LeggedOdometry::Impl::getLatestSwitchedContact(const std::map<int, BipedalLocomotion::Contacts::EstimatedContact>& contacts)
{
    std::string_view printPrefix = "[LeggedOdometry::Impl::getLatestContact] ";
    if (contacts.size() < 1)
    {
        std::cerr << printPrefix << "No contact data available." << std::endl;
        return iDynTree::FRAME_INVALID_INDEX;
    }

    bool atleastOneActiveContact{false};
    int latestContactIdx{-1};

    // we initialize latestTime = -1 so assuming no contact has a switch time lower than 0, we find
    // the contact with the highest switch time
    std::chrono::nanoseconds latestTime{-1}; // assuming time cannot be negative

    for (const auto& [idx, contact] : contacts)
    {
        if (contact.isActive)
        {
            atleastOneActiveContact = true;
            if (contact.switchTime > latestTime)
            {
                latestContactIdx = idx;
                latestTime = contact.switchTime;
            }
        }
    }

    if (!atleastOneActiveContact)
    {
        std::cerr << printPrefix << "No active contacts." << std::endl;
        return iDynTree::FRAME_INVALID_INDEX;
    }
    return latestContactIdx;
}

iDynTree::FrameIndex LeggedOdometry::Impl::getLastActiveContact(const std::map<int, BipedalLocomotion::Contacts::EstimatedContact>& contacts, const int& activeIndex)
{
    std::string_view printPrefix = "[LeggedOdometry::Impl::getLastActiveContact] ";
    if (contacts.size() < 1)
    {
        std::cerr << printPrefix << "No contact data available." << std::endl;
        return iDynTree::FRAME_INVALID_INDEX;
    }

    bool atleastOneActiveContact{false};
    if (contacts.find(activeIndex) != contacts.end())
    {
        if (contacts.at(activeIndex).isActive)
        {
            return activeIndex;
        }
    }

    // here we are interested in finding the last active contact so we are looking for the minimum
    // switchTime
    std::chrono::nanoseconds latestTime{std::chrono::nanoseconds::max()};
    int latestContactIdx{-1};
    for (const auto& [idx, contact] : contacts)
    {
        if (contact.isActive)
        {
            atleastOneActiveContact = true;
            if (contact.switchTime < latestTime)
            {
                latestContactIdx = idx;
                latestTime = contact.switchTime;
            }
        }
    }

    if (!atleastOneActiveContact)
    {
        std::cerr << printPrefix << "No active contacts." << std::endl;
        return iDynTree::FRAME_INVALID_INDEX;
    }

    return latestContactIdx;
}

bool LeggedOdometry::resetEstimator()
{
    m_pimpl->resetInternal(m_modelComp, m_meas);
    m_pimpl->updateInternalState(m_measPrev, m_modelComp, m_state, m_estimatorOut);

    if (!updateBaseStateFromIMUState(m_state, m_measPrev,
                                     m_estimatorOut.basePose, m_estimatorOut.baseTwist))
    {
        std::cerr << "[LeggedOdometry::resetEstimator]" << "Could not update base state from IMU state." << std::endl;
        return false;
    }

    return true;
}

bool LeggedOdometry::resetEstimator(const std::string& refFrameForWorld,
                                    const Eigen::Quaterniond& worldOrientationInRefFrame,
                                    const Eigen::Vector3d& worldPositionInRefFrame)
{
    std::string_view printPrefix = "[LeggedOdometry::resetEstimator] ";
    auto refFrameIdx = m_modelComp.kinDyn()->model().getFrameIndex(refFrameForWorld);
    if (refFrameIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << printPrefix << "Frame unavailable in the loaded URDF model." << std::endl;
        return false;

    }
    m_pimpl->m_refFrame_H_world = manif::SE3d(worldPositionInRefFrame, worldOrientationInRefFrame);
    m_pimpl->m_initialRefFrameForWorldIdx = refFrameIdx;

    resetEstimator();

    return true;
}

void LeggedOdometry::Impl::resetInternal(FloatingBaseEstimator::ModelComputations& modelComp,
                                         const FloatingBaseEstimators::Measurements& meas)
{
    if (m_currentFixedFrameIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        m_currentFixedFrameIdx = m_initialFixedFrameIdx;
    }

    modelComp.kinDyn()->setJointPos(meas.encoders);

    manif::SE3d refFrame_H_fixedFrame  = toManifPose(modelComp.kinDyn()->
                                                     getRelativeTransform(m_initialRefFrameForWorldIdx,
                                                                          m_currentFixedFrameIdx));

    m_world_H_fixedFrame = m_refFrame_H_world.inverse()*refFrame_H_fixedFrame;

    m_prevFixedFrameIdx = m_currentFixedFrameIdx;
}

int LeggedOdometry::getFixedFrameIdx()
{
    return m_pimpl->m_currentFixedFrameIdx;
}

manif::SE3d& LeggedOdometry::getFixedFramePose() const
{
    return m_pimpl->m_world_H_fixedFrame;
}

bool LeggedOdometry::Impl::updateInternalState(FloatingBaseEstimators::Measurements& meas,
                                               FloatingBaseEstimator::ModelComputations& modelComp,
                                               FloatingBaseEstimators::InternalState& state,
                                               FloatingBaseEstimators::Output& out)
{
    const std::string_view printPrefix = "[LeggedOdometry::Impl::updateInternalState] ";
    manif::SE3d fixedFrame_H_imu = toManifPose(modelComp.kinDyn()->
                                               getRelativeTransform(m_currentFixedFrameIdx,
                                                                    modelComp.kinDyn()->model().getFrameIndex(modelComp.baseLinkIMU())));
    manif::SE3d world_H_imu = m_world_H_fixedFrame*fixedFrame_H_imu;

    state.imuOrientation = world_H_imu.quat();
    state.imuPosition = world_H_imu.translation();

    for (auto& [idx, contact] : state.supportFrameData)
    {
        manif::SE3d world_H_contact;
        if (idx == m_currentFixedFrameIdx)
        {
            contact.pose = m_world_H_fixedFrame;
        }
        else
        {
            manif::SE3d fixedFrame_H_contact = toManifPose(modelComp.kinDyn()->
                                                       getRelativeTransform(m_currentFixedFrameIdx, idx));
            contact.pose = m_world_H_fixedFrame*fixedFrame_H_contact;
        }

        // TODO{@prashanthr05} deprecate and remove the following in future versions
        if (contact.name == modelComp.leftFootContactFrame())
        {
            state.lContactFrameOrientation = contact.pose.quat();
            state.lContactFramePosition = contact.pose.translation();
        }

        if (contact.name == modelComp.rightFootContactFrame())
        {
            state.rContactFrameOrientation = contact.pose.quat();
            state.rContactFramePosition = contact.pose.translation();
        }
    }

    if (m_odometryInitialized)
    {
        bool ok{false};
        // compute base velocity
        if (velComp == LOVelComputation::singleFrame)
        {
            ok = computeBaseVelocityUsingFixedFrameConstraint(meas, modelComp, out);
        }
        else if (velComp == LOVelComputation::multiFrameLeastSquare)
        {
            ok = computeBaseVelocityUsingAllFixedFrames(state, meas, modelComp, out);
        }
        else if (velComp == LOVelComputation::multiFrameLeastSquareWithJVel)
        {
            ok = computeBaseVelocityUsingAllFixedFrameConstraint(state, meas, modelComp, out);
        }
        else if (velComp == LOVelComputation::multiFrameAverage)
        {
            ok = computeBaseVelocityUsingAllFixedFrameAverage(state, meas, modelComp, out);
        }

        if (!ok)
        {
            std::cerr << printPrefix << "Could not compute base velocity using fixed frame constraints." << std::endl;
            return false;
        }
    }
    return true;
}

bool LeggedOdometry::Impl::computeBaseVelocityUsingFixedFrameConstraint(FloatingBaseEstimators::Measurements& meas,
                                                                           FloatingBaseEstimator::ModelComputations& modelComp,
                                                                           FloatingBaseEstimators::Output& out)
{
    const std::string_view printPrefix = "[LeggedOdometry::Impl::computeBaseIMUVelocityUsingFixedFrameConstraint] ";

    // here we do the computations with respect to the base link frame and not the base link IMU frame
    // accordingly, since `m_useIMUVelForBaseVelComputation` is set to false, the IMU velocity in the internal state
    // remains un-updated, we directly update the base twist in the output
    if (!modelComp.kinDyn()->getFrameFreeFloatingJacobian(m_currentFixedFrameIdx, m_contactJacobian))
    {
        std::cerr << printPrefix << "Could not compute contact Jacobian."
        << std::endl;
        return false;
    }

    m_contactJacobianBase = m_contactJacobian.block<6, 6>(m_baseOffset, m_baseOffset);
    m_contactJacobianShape = m_contactJacobian.block(m_baseOffset, m_shapeOffset, m_spatialDim, modelComp.kinDyn()->getNrOfDegreesOfFreedom());

    m_vBase = -(m_contactJacobianBase.inverse()) * m_contactJacobianShape * meas.encodersSpeed;
    out.baseTwist = m_vBase;

    return true;
}

bool LeggedOdometry::Impl::computeBaseVelocityUsingAllFixedFrameConstraint(const FloatingBaseEstimators::InternalState& state,
                                                                              FloatingBaseEstimators::Measurements& meas,
                                                                              FloatingBaseEstimator::ModelComputations& modelComp,
                                                                              FloatingBaseEstimators::Output& out)
{
    const std::string_view printPrefix = "[LeggedOdometry::Impl::computeBaseIMUVelocityUsingAllFixedFrameConstraint] ";

    // here we do the computations with respect to the base link frame and not the base link IMU frame
    // accordingly, since `m_useIMUVelForBaseVelComputation` is set to false, the IMU velocity in the internal state
    // remains un-updated, we directly update the base twist in the output

    std::size_t nrContacts{0};
    std::size_t nrJoints{modelComp.kinDyn()->getNrOfDegreesOfFreedom()};

    // run two loops (alteratively once can do conservative resize of the matrices)
    for (const auto& [idx, contact] : state.supportFrameData)
    {
        if (contact.isActive)
        {
            nrContacts++;
        }
    }

    if (nrContacts == 0)
    {
        std::cerr << printPrefix << "No contacts available. Unable to compute base velocity."
                << std::endl;
        return false;
    }


    // given number of contacts resize the matrices for weighted least square
    int n = m_spatialDim*nrContacts + nrJoints;
    int m = m_spatialDim + nrJoints;
    Eigen::MatrixXd A(n, m);

    // Set zero contact velocities and joint velocities
    Eigen::VectorXd y(n);
    y.head(n-nrJoints).setZero();
    y.tail(nrJoints) = meas.encodersSpeed;

    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd Delta = reg*Eigen::MatrixXd::Identity(m, m);

    // run another loop to populate matrices
    std::size_t cIdx{0};
    for (const auto& [idx, contact] : state.supportFrameData)
    {
        if (contact.isActive)
        {
            if (!modelComp.kinDyn()->getFrameFreeFloatingJacobian(idx, m_contactJacobian))
            {
                std::cerr << printPrefix << "Could not compute contact Jacobian."
                << std::endl;
                return false;
            }

            A.block(m_spatialDim*cIdx, m_baseOffset, m_spatialDim, m) = m_contactJacobian;
            int vec3Offset{3};
            W.block(m_spatialDim*cIdx, m_spatialDim*cIdx, vec3Offset, vec3Offset) *= wLin;
            W.block(m_spatialDim*cIdx+vec3Offset, m_spatialDim*cIdx+vec3Offset, vec3Offset, vec3Offset) *= wAng;
            cIdx ++;
        }
    }


    A.bottomRows(nrJoints) << Eigen::MatrixXd::Zero( nrJoints, m_spatialDim), Eigen::MatrixXd::Identity(nrJoints, nrJoints);
    W.bottomRightCorner(nrJoints, nrJoints) *= wJVel;

    // least square solution
    Eigen::MatrixXd S = (A.transpose()*W*A + Delta).inverse();
    Eigen::VectorXd xOptimal = S*A.transpose()*W*y;

    m_vBase = xOptimal.head(m_spatialDim);
    m_sDotFilt = xOptimal.tail(nrJoints);
    out.baseTwist = m_vBase;
    return true;
}


bool LeggedOdometry::Impl::computeBaseVelocityUsingAllFixedFrameAverage(const FloatingBaseEstimators::InternalState& state,
                                                                        FloatingBaseEstimators::Measurements& meas,
                                                                        FloatingBaseEstimator::ModelComputations& modelComp,
                                                                        FloatingBaseEstimators::Output& out)
{
    const std::string_view printPrefix = "[LeggedOdometry::Impl::computeBaseIMUVelocityUsingAllFixedFrameConstraint] ";

    // here we do the computations with respect to the base link frame and not the base link IMU frame
    // accordingly, since `m_useIMUVelForBaseVelComputation` is set to false, the IMU velocity in the internal state
    // remains un-updated, we directly update the base twist in the output

    std::size_t nrContacts{0};
    // run two loops (alteratively once can do conservative resize of the matrices)
    Eigen::VectorXd sumV = Eigen::MatrixXd::Zero(6, 1);
    for (const auto& [idx, contact] : state.supportFrameData)
    {
        if (contact.isActive)
        {
            nrContacts++;
            if (!modelComp.kinDyn()->getFrameFreeFloatingJacobian(idx, m_contactJacobian))
            {
                std::cerr << printPrefix << "Could not compute contact Jacobian."
                << std::endl;
                return false;
            }

            m_contactJacobianBase = m_contactJacobian.block<6, 6>(m_baseOffset, m_baseOffset);
            m_contactJacobianShape = m_contactJacobian.block(m_baseOffset, m_shapeOffset, m_spatialDim, modelComp.kinDyn()->getNrOfDegreesOfFreedom());

            sumV += -(m_contactJacobianBase.inverse()) * m_contactJacobianShape * meas.encodersSpeed;
        }
    }

    if (nrContacts == 0)
    {
        std::cerr << printPrefix << "No contacts available. Unable to compute base velocity."
                << std::endl;
        return false;
    }

    m_vBase = sumV/nrContacts;
    out.baseTwist = m_vBase;
    return true;
}


bool LeggedOdometry::Impl::computeBaseVelocityUsingAllFixedFrames(const FloatingBaseEstimators::InternalState& state,
                                                                  FloatingBaseEstimators::Measurements& meas,
                                                                  FloatingBaseEstimator::ModelComputations& modelComp,
                                                                  FloatingBaseEstimators::Output& out)
{
    const std::string_view printPrefix = "[LeggedOdometry::Impl::computeBaseVelocityUsingAllFixedFrames] ";

    // here we do the computations with respect to the base link frame and not the base link IMU frame
    // accordingly, since `m_useIMUVelForBaseVelComputation` is set to false, the IMU velocity in the internal state
    // remains un-updated, we directly update the base twist in the output

    std::size_t nrContacts{0};
    // run two loops (alteratively once can do conservative resize of the matrices)
    for (const auto& [idx, contact] : state.supportFrameData)
    {
        if (contact.isActive)
        {
            nrContacts++;
        }
    }

    if (nrContacts == 0)
    {
        std::cerr << printPrefix << "No contacts available. Unable to compute base velocity."
                << std::endl;
        return false;
    }


    // given number of contacts resize the matrices for weighted least square
    int n = m_spatialDim*nrContacts;
    int m = m_spatialDim;
    Eigen::MatrixXd A(n, m);
    Eigen::VectorXd y(n);

    Eigen::MatrixXd W = Eigen::MatrixXd::Identity(n, n);
    Eigen::MatrixXd Delta = reg*Eigen::MatrixXd::Identity(m, m);

    // run another loop to populate matrices
    std::size_t cIdx{0};
    for (const auto& [idx, contact] : state.supportFrameData)
    {
        if (contact.isActive)
        {
            if (!modelComp.kinDyn()->getFrameFreeFloatingJacobian(idx, m_contactJacobian))
            {
                std::cerr << printPrefix << "Could not compute contact Jacobian."
                << std::endl;
                return false;
            }

            m_contactJacobianBase = m_contactJacobian.block<6, 6>(m_baseOffset, m_baseOffset);
            m_contactJacobianShape = m_contactJacobian.block(m_baseOffset, m_shapeOffset, m_spatialDim, modelComp.kinDyn()->getNrOfDegreesOfFreedom());

            A.block(m_spatialDim*cIdx, m_baseOffset, m_spatialDim, m_spatialDim) = m_contactJacobianBase;
            y.segment(m_spatialDim*cIdx, m_spatialDim) = - m_contactJacobianShape*meas.encodersSpeed;

            int vec3Offset{3};
            W.block(m_spatialDim*cIdx, m_spatialDim*cIdx, vec3Offset, vec3Offset) *= wLin;
            W.block(m_spatialDim*cIdx+vec3Offset, m_spatialDim*cIdx+vec3Offset, vec3Offset, vec3Offset) *= wAng;

            cIdx ++;
        }
    }

    // least square solution
    Eigen::MatrixXd S = (A.transpose()*W*A + Delta).inverse();
    Eigen::VectorXd xOptimal = S*A.transpose()*W*y;

    m_vBase = xOptimal;
    out.baseTwist = m_vBase;

    return true;
}

bool LeggedOdometry::updateKinematics(FloatingBaseEstimators::Measurements& meas,
                                      const double& /*dt*/)
{
    const std::string_view printPrefix = "[LeggedOdometry::updateKinematics] ";
    if ( (meas.encoders.size() != m_modelComp.nrJoints()) ||
         (meas.encodersSpeed.size() != m_modelComp.nrJoints()))
    {
        std::cerr << printPrefix << "Kinematic measurements size mismatch. Please call setKinematics() before calling advance()."
        << std::endl;
        return false;
    }

    // update the internal contact state
    m_pimpl->updateInternalContactStates(meas, m_modelComp, m_state.supportFrameData);

    // initialization step
    if (!m_pimpl->m_odometryInitialized)
    {
        if (!m_modelComp.kinDyn()->setJointPos(iDynTree::make_span(meas.encoders.data(), meas.encoders.size())))
        {
            std::cerr << printPrefix << "Unable to set joint positions."
            << std::endl;
            return false;
        }

        m_pimpl->resetInternal(m_modelComp, meas);

        // update internal states
        if (!m_pimpl->updateInternalState(meas, m_modelComp, m_state, m_estimatorOut))
        {
            std::cerr << printPrefix << "Could not update internal state of the estimator." << std::endl;
            return false;
        }

        m_pimpl->m_odometryInitialized = true;
        std::cout << printPrefix << "Initialized LeggedOdometry." << std::endl;
        return true;
    }

    // run step
    if (m_pimpl->switching != LOSwitching::useExternalUpdate)
    {
        // change fixed frame depending on switch times
        iDynTree::FrameIndex newIdx{iDynTree::FRAME_INVALID_INDEX};
        if (m_pimpl->switching == LOSwitching::latest)
        {
            newIdx = m_pimpl->getLatestSwitchedContact(m_state.supportFrameData);
        }
        else if (m_pimpl->switching == LOSwitching::lastActive)
        {
            newIdx = m_pimpl->getLastActiveContact(m_state.supportFrameData, m_pimpl->m_currentFixedFrameIdx);
        }

        if (newIdx == iDynTree::FRAME_INVALID_INDEX)
        {
            std::cerr << printPrefix << "The assumption of atleast one active contact is broken. This may lead ot unexpected results." << std::endl;
            return false;
        }

        if (newIdx != m_pimpl->m_currentFixedFrameIdx)
        {
            if (!m_pimpl->changeFixedFrame(newIdx, m_modelComp.kinDyn()))
            {
                std::cerr << printPrefix << "Unable to change the fixed frame. This may lead ot unexpected results." << std::endl;
                return false;
            }
        }
    }

    // TODO{@prashanthr05} remove contacts if outdated
    // should we do this  in a top-level at FloatingBaseEstimators.cpp ?
    // since it might be useful also for other estimators

    // update internal states
    if (!m_pimpl->updateInternalState(m_measPrev, m_modelComp, m_state, m_estimatorOut))
    {
        std::cerr << printPrefix << "Could not update internal state of the estimator." << std::endl;
        return false;
    }
    return true;
}

bool LeggedOdometry::changeFixedFrame(const std::string& frameName)
{
    const std::string_view printPrefix = "[LeggedOdometry::changeFixedFrame] ";
    auto frameIdx = m_modelComp.kinDyn()->getFrameIndex(frameName);
    if (frameIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        std::cerr << printPrefix << "Specified frame unavailable in the loaded model." << std::endl;
        return false;
    }

    return changeFixedFrame(frameIdx);
}

bool LeggedOdometry::changeFixedFrame(const std::ptrdiff_t& newIdx)
{
    const std::string_view printPrefix = "[LeggedOdometry::changeFixedFrame] ";
    if (m_pimpl->switching != LOSwitching::useExternalUpdate)
    {
        std::cerr << printPrefix << "Unable to change fixed frame externally, since the estimator was not loaded with the option." << std::endl;
        return false;
    }

    if (newIdx != m_pimpl->m_currentFixedFrameIdx)
    {
        if (!m_pimpl->changeFixedFrame(newIdx, m_modelComp.kinDyn()))
        {
            std::cerr << printPrefix << "Unable to change new fixed frame." << std::endl;
            return false;
        }
    }

    return true;
}

bool LeggedOdometry::Impl::changeFixedFrame(const iDynTree::FrameIndex& newIdx,
                                            std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    const std::string_view printPrefix = "[LeggedOdometry::changeFixedFrame] ";
    if (newIdx == iDynTree::FRAME_INVALID_INDEX || !kinDyn->model().isValidFrameIndex(newIdx))
    {
        std::cerr << printPrefix << "Specified frame index not available in the loaded URDF Model."
            << std::endl;
            return false;
    }

    manif::SE3d oldFixed_H_newFixed  = toManifPose(kinDyn->getRelativeTransform(m_currentFixedFrameIdx, newIdx));

    m_world_H_fixedFrame = (m_world_H_fixedFrame*oldFixed_H_newFixed);
    m_prevFixedFrameIdx = m_currentFixedFrameIdx;
    m_currentFixedFrameIdx = newIdx;

    std::cout << printPrefix << "Fixed frame changed to " << kinDyn->model().getFrameName(m_currentFixedFrameIdx) << "." << std::endl;
    return true;
}


bool LeggedOdometry::changeFixedFrame(const std::ptrdiff_t& newIdx,
                                      const Eigen::Quaterniond& frameOrientationInWorld,
                                      const Eigen::Vector3d& framePositionInWorld)
{
    const std::string_view printPrefix = "[LeggedOdometry::changeFixedFrame] ";

    if (newIdx == iDynTree::FRAME_INVALID_INDEX || !m_modelComp.kinDyn()->model().isValidFrameIndex(newIdx))
    {
        std::cerr << printPrefix << "Specified frame index not available in the loaded URDF Model."
            << std::endl;
            return false;
    }

    m_pimpl->m_world_H_fixedFrame.quat(frameOrientationInWorld);
    m_pimpl->m_world_H_fixedFrame.translation(framePositionInWorld);
    m_pimpl->m_currentFixedFrameIdx = newIdx;

    // Since this method is an external call we need to remember
    // to update internal state and also the base state from the IMU state
    // update internal states
    if (!m_pimpl->updateInternalState(m_measPrev, m_modelComp, m_state, m_estimatorOut))
    {
        std::cerr << printPrefix << "Could not update internal state of the estimator." << std::endl;
        return false;
    }

    if (!updateBaseStateFromIMUState(m_state, m_measPrev,
                                     m_estimatorOut.basePose, m_estimatorOut.baseTwist))
    {
        std::cerr << printPrefix << "Could not update base state from IMU state." << std::endl;
        return false;
    }

    return true;
}
