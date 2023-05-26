/**
 * @file MANNAutoregressive.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/SO3Dynamics.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ML/MANN.h>
#include <BipedalLocomotion/ML/MANNAutoregressive.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Model/Model.h>
#include <manif/SE3.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

struct MANNAutoregressive::Impl
{
    MANN mann;
    MANNInput mannInput;
    iDynTree::KinDynComputations kinDyn;
    MANNAutoregressiveOutput output;

    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};
    std::chrono::nanoseconds dT;

    std::shared_ptr<ContinuousDynamicalSystem::SO3Dynamics> baseOrientationDynamics;
    ContinuousDynamicalSystem::ForwardEuler<ContinuousDynamicalSystem::SO3Dynamics> integrator;

    Eigen::Vector3d gravity;

    AutoregressiveState state;

    struct ContactInfo
    {
        Contacts::DiscreteGeometryContact discreteGeometryContact;
        Contacts::EstimatedContact* estimatedContactPtr{nullptr}; /**< Pointer to the estimated
                                                                     contact saved as output. */
    };

    ContactInfo rightFoot;
    ContactInfo leftFoot;
    ContactInfo* supportFootPtr{nullptr}; /**< Pointer to the support foot. It will point to an
                                               object stored in the class. */
    Eigen::MatrixXd supportFootJacobian;
    int supportCornerIndex{-1};
    Eigen::Vector3d projectedContactPositionInWorldFrame{Eigen::Vector3d::Zero()};

    int rootIndex;
    int chestIndex;

    bool isRobotStopped;

    bool isOutputValid{false};

    enum class FSM
    {
        Idle,
        Initialized,
        Reset,
        Running,
    };

    FSM fsmState{FSM::Idle};

    Math::SchmittTrigger leftFootSchmittTrigger;
    Math::SchmittTrigger rightFootSchmittTrigger;

    void trajectoryBlending(Eigen::Ref<const Eigen::Matrix2Xd> mannOutputMatrix,
                            Eigen::Ref<const Eigen::Matrix2Xd> desiredMatrix,
                            const double& tau,
                            Eigen::Ref<Eigen::Matrix2Xd> out);

    void updateContact(const double referenceHeight,
                       Math::SchmittTrigger& trigger,
                       Math::SchmittTriggerState& triggerState,
                       Contacts::EstimatedContact& contact);
};

void MANNAutoregressive::Impl::updateContact(const double referenceHeight,
                                             Math::SchmittTrigger& trigger,
                                             Math::SchmittTriggerState& triggerState,
                                             Contacts::EstimatedContact& contact)
{
    Math::SchmittTriggerInput footSchmittInput;
    footSchmittInput.rawValue = referenceHeight;
    footSchmittInput.time = this->currentTime;

    trigger.setInput(footSchmittInput);
    trigger.advance();
    triggerState = trigger.getState();

    if (!contact.isActive && triggerState.state)
    {
        using namespace BipedalLocomotion::Conversions;
        contact.isActive = true;
        contact.switchTime = this->currentTime;
        contact.pose = toManifPose(this->kinDyn.getWorldTransform(contact.index));

    } else if (contact.isActive && !triggerState.state)
    {
        contact.isActive = false;
    }

    contact.lastUpdateTime = this->currentTime;
}

MANNAutoregressive::MANNAutoregressive()
    : m_pimpl(std::make_unique<Impl>())
{
}

MANNAutoregressive::~MANNAutoregressive() = default;

bool MANNAutoregressive::setRobotModel(const iDynTree::Model& model)
{
    constexpr std::size_t twistSize = 6;

    // resize the jacobian of the support foot that will be used in the class to evaluate the
    // velocity of the rootlink assuming that the stance foot has zero velocity
    m_pimpl->supportFootJacobian.resize(twistSize, twistSize + model.getNrOfDOFs());

    return m_pimpl->kinDyn.loadRobotModel(model);
}

bool MANNAutoregressive::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[MANNAutoregressive::initialize]";

    if (!m_pimpl->kinDyn.isValid())
    {
        log()->error("{} The model is not valid, please call setRobotModel before initialize.",
                     logPrefix);
        return false;
    }

    auto ptr = paramHandler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameters handler.", logPrefix);
        return false;
    }

    // populate the left and right contact info
    m_pimpl->leftFoot.estimatedContactPtr = &m_pimpl->output.leftFoot;
    m_pimpl->rightFoot.estimatedContactPtr = &m_pimpl->output.rightFoot;

    // the following lambda function is useful to get the name of the indexes of all the frames
    // required by the class
    auto getFrameIndex
        = [this, ptr, logPrefix](const std::string& frameParamName, int& frameIndex) -> bool {
        std::string frameName;
        if (!ptr->getParameter(frameParamName, frameName))
        {
            log()->error("{} Unable to find the parameter named '{}'.", logPrefix, frameParamName);
            return false;
        }

        iDynTree::FrameIndex tmp = this->m_pimpl->kinDyn.model().getFrameIndex(frameName);
        if (tmp == iDynTree::LINK_INVALID_INDEX)
        {
            log()->error("{} Unable to find the frame named '{}' in the model.",
                         logPrefix,
                         frameName);
            return false;
        }

        frameIndex = tmp;

        return true;
    };

    // the following lambda function is useful to the position of the corners in the feet
    auto getContactCorners = [ptr, logPrefix](const std::string& contactName,
                                              Contacts::DiscreteGeometryContact& contact) -> bool {
        auto contactHandler = ptr->getGroup(contactName).lock();
        if (contactHandler == nullptr)
        {
            log()->error("{} Unable to load the group for the contact named '{}'.",
                         logPrefix,
                         contactName);
            return false;
        }

        // get the number of corners
        int numberOfCorners;
        if (!contactHandler->getParameter("number_of_corners", numberOfCorners))
        {
            log()->error("{} Unable to get the number of corners.", logPrefix);
            return false;
        }
        contact.corners.resize(numberOfCorners);

        // for each corner we need to get its position in the frame attached to the foot
        for (std::size_t j = 0; j < numberOfCorners; j++)
        {
            if (!contactHandler->getParameter("corner_" + std::to_string(j),
                                              contact.corners[j].position))
            {
                // prepare the error
                std::string cornersNames;
                for (std::size_t k = 0; k < numberOfCorners; k++)
                {
                    cornersNames += " corner_" + std::to_string(k);
                }

                log()->error("{} Unable to load the corner number {}. Please provide the corners "
                             "having one of the following names: {}.",
                             logPrefix,
                             j,
                             cornersNames);

                return false;
            }
        }
        return true;
    };

    auto initializeSchmittTrigger
        = [logPrefix](std::shared_ptr<const ParametersHandler::IParametersHandler> paramHandler,
                      const std::string& footGroupName,
                      Math::SchmittTrigger& trigger) -> bool {
        auto ptr = paramHandler->getGroup(footGroupName).lock();
        if (ptr == nullptr)
        {
            log()->error("{} Invalid parameter handler for the Schmitt trigger named: {}.",
                         logPrefix,
                         footGroupName);
            return false;
        }

        auto clone = ptr->clone();
        double onThreshold, offThreshold;
        if (!clone->getParameter("on_threshold", onThreshold) || onThreshold < 0)
        {
            log()->error("{} Invalid parameter 'on_threshold' for the Schmitt trigger named: {}. "
                         "Please remember it must be a positive number.",
                         logPrefix,
                         footGroupName);
            return false;
        }

        if (!clone->getParameter("off_threshold", offThreshold) || offThreshold < 0)
        {
            log()->error("{} Invalid parameter 'off_threshold' for the Schmitt trigger named: {}. "
                         "Please remember it must be a positive number.",
                         logPrefix,
                         footGroupName);
            return false;
        }

        // now we force the parameters to be negative. This is required since in this case, we
        // desire the trigger to return True when the foot is in contact and False otherwise. To
        // achieve this, considering that the foot height is always positive, we provide the
        // negative of the foot height to the trigger. This ensures that the trigger returns True
        // when the foot is close to the ground.
        clone->setParameter("on_threshold", -onThreshold);
        clone->setParameter("off_threshold", -offThreshold);

        if (!trigger.initialize(clone))
        {
            log()->error("{} Unable to initialize the Schmitt trigger named {}.",
                         logPrefix,
                         footGroupName);
            return false;
        }
        return true;
    };

    // TODO should we reset the base orientation to identity
    m_pimpl->baseOrientationDynamics = std::make_shared<ContinuousDynamicalSystem::SO3Dynamics>();
    m_pimpl->baseOrientationDynamics->setState({manif::SO3d::Identity()});
    m_pimpl->integrator.setDynamicalSystem(m_pimpl->baseOrientationDynamics);

    if (!ptr->getParameter("sampling_time", m_pimpl->dT))
    {
        log()->error("{} Unable to find the parameter named '{}'.", logPrefix, "sampling_time");
        return false;
    }
    m_pimpl->integrator.setIntegrationStep(m_pimpl->dT);

    std::string forwardDirection;
    if (!ptr->getParameter("forward_direction", forwardDirection) || forwardDirection != "x")
    {
        log()->error("{} Only forward_direction equal to 'x' is supported.", logPrefix);
        return false;
    }

    // the gravity is not used by this class, however the kindyn computation object requires this
    // information when the internal state is updated. For this reason we avoid to allocate the
    // memory every cycle and we keep it here.
    m_pimpl->gravity.setZero();
    m_pimpl->gravity[2] = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

    // set the indices required by the trajectory generator
    bool ok = getFrameIndex("root_link_frame_name", m_pimpl->rootIndex);
    ok = ok && getFrameIndex("chest_link_frame_name", m_pimpl->chestIndex);
    ok = ok
         && getFrameIndex("left_foot_frame_name", m_pimpl->leftFoot.discreteGeometryContact.index);
    ok = ok
         && getFrameIndex("right_foot_frame_name",
                          m_pimpl->rightFoot.discreteGeometryContact.index);
    ok = ok && getFrameIndex("left_foot_frame_name", m_pimpl->leftFoot.estimatedContactPtr->index);
    ok = ok
         && getFrameIndex("right_foot_frame_name", m_pimpl->rightFoot.estimatedContactPtr->index);

    // Get the position of the corners in the foot frame
    ok = ok && getContactCorners("LEFT_FOOT", m_pimpl->leftFoot.discreteGeometryContact);
    ok = ok && getContactCorners("RIGHT_FOOT", m_pimpl->rightFoot.discreteGeometryContact);

    auto mannParamHandler = ptr->getGroup("MANN").lock();
    if (mannParamHandler == nullptr)
    {
        log()->error("{} Unable to find the group named 'MANN'.", logPrefix);
        return false;
    }

    // add the joint size in the handler
    auto cloneHandler = mannParamHandler->clone();
    cloneHandler->setParameter("number_of_joints", int(m_pimpl->kinDyn.getNrOfDegreesOfFreedom()));
    ok = ok && m_pimpl->mann.initialize(cloneHandler);

    ok = ok && initializeSchmittTrigger(ptr, "LEFT_FOOT", m_pimpl->leftFootSchmittTrigger);
    ok = ok && initializeSchmittTrigger(ptr, "RIGHT_FOOT", m_pimpl->rightFootSchmittTrigger);
    if (ok)
    {
        // here we assume that the system starts in double support
        Math::SchmittTriggerState dummyState;
        dummyState.state = true;
        m_pimpl->leftFootSchmittTrigger.setState(dummyState);
        m_pimpl->rightFootSchmittTrigger.setState(dummyState);
        m_pimpl->fsmState = Impl::FSM::Initialized;
    }
    return ok;
}

bool MANNAutoregressive::reset(const MANNInput& input,
                               const Contacts::EstimatedContact& leftFoot,
                               const Contacts::EstimatedContact& rightFoot,
                               const manif::SE3d& basePosition,
                               const manif::SE3Tangentd& baseVelocity)
{
    AutoregressiveState state;

    // 51 is the length of 1 second of past trajectory stored in the autoregressive state. Since the
    // original mocap data are collected at 50 Hz, and therefore the trajectory generation is
    // assumed to proceed at 50 Hz, we need 50 datapoints to store the past second of trajectory.
    // Along with the present datapoint, they sum up to 51!
    constexpr size_t lengthOfPresentPlusPastTrajectory = 51;
    state.pastProjectedBasePositions
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{0.0, 0.0}};
    state.pastProjectedBaseVelocity
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{0.0, 0.0}};
    state.pastFacingDirection
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{1.0, 0.0}};
    state.I_H_FD = manif::SE2d::Identity();
    state.previousMannInput = input;

    Math::SchmittTriggerState dummyStateLeft;
    dummyStateLeft.state = leftFoot.isActive;

    Math::SchmittTriggerState dummyStateRight;
    dummyStateRight.state = rightFoot.isActive;

    return this->reset(input,
                       leftFoot,
                       dummyStateLeft,
                       rightFoot,
                       dummyStateRight,
                       basePosition,
                       baseVelocity,
                       state,
                       std::chrono::nanoseconds::zero());
}

bool MANNAutoregressive::reset(const MANNInput& input,
                               const Contacts::EstimatedContact& leftFoot,
                               const Math::SchmittTriggerState& leftFootSchimittTriggerState,
                               const Contacts::EstimatedContact& rightFoot,
                               const Math::SchmittTriggerState& rightFootSchimittTriggerState,
                               const manif::SE3d& basePosition,
                               const manif::SE3Tangentd& baseVelocity,
                               const AutoregressiveState& autoregressiveState,
                               const std::chrono::nanoseconds& time)
{
    constexpr auto logPrefix = "[MANNAutoregressive::reset]";

    if (m_pimpl->fsmState != Impl::FSM::Initialized && m_pimpl->fsmState != Impl::FSM::Running
        && m_pimpl->fsmState != Impl::FSM::Reset)
    {
        log()->error("{} MANNAutoregressive has not been initialized.", logPrefix);
        return false;
    }

    auto updateFoot = [logPrefix](const char* footName,
                                  const Contacts::EstimatedContact& newContact,
                                  Contacts::EstimatedContact& out) -> bool {
        if (out.index != newContact.index)
        {
            log()->error("{} The index of the left {} is different from the one provided. "
                         "Provided: {}, Expected: {}.",
                         logPrefix,
                         footName,
                         newContact.index,
                         out.index);
            return false;
        }
        out = newContact;
        return true;
    };

    if (!updateFoot("left", leftFoot, m_pimpl->output.leftFoot)
        || !updateFoot("right", rightFoot, m_pimpl->output.rightFoot))
    {
        return false;
    }

    m_pimpl->mannInput = input;
    m_pimpl->currentTime = time;
    m_pimpl->isRobotStopped = true;

    if (!m_pimpl->kinDyn.setRobotState(basePosition.transform(),
                                       input.jointPositions,
                                       baseVelocity.coeffs(),
                                       input.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to reset the kindyncomputations object.", logPrefix);
        return false;
    }

    // the output is not valid since we are resetting the trajectory planner
    m_pimpl->isOutputValid = false;
    m_pimpl->supportCornerIndex = -1;
    m_pimpl->supportFootPtr = nullptr;
    m_pimpl->state = autoregressiveState;

    if (!m_pimpl->baseOrientationDynamics->setState({basePosition.quat()}))
    {
        log()->error("{} Unable to reset the base orientation dynamics.", logPrefix);
        return false;
    }

    if (!m_pimpl->mann.setInput(m_pimpl->mannInput))
    {
        log()->error("{} Unable to set the inputs to MANN.", logPrefix);
        return false;
    }

    m_pimpl->leftFootSchmittTrigger.setState(leftFootSchimittTriggerState);
    m_pimpl->rightFootSchmittTrigger.setState(rightFootSchimittTriggerState);

    m_pimpl->fsmState = Impl::FSM::Reset;

    return true;
}

bool MANNAutoregressive::setInput(const Input& input)
{
    // If the output of the network is valid, then the network has been called at least once
    constexpr auto logPrefix = "[MANNAutoregressive::setInput]";

    if (m_pimpl->fsmState != Impl::FSM::Reset && !this->isOutputValid())
    {
        log()->error("{} The network has not been reset. Please call reset().", logPrefix);
        return false;
    }

    if (!this->isOutputValid())
    {
        return true;
    }

    // get the output
    const auto& mannOutput = m_pimpl->mann.getOutput();

    // the joint positions and velocities are considered as new input
    m_pimpl->mannInput.jointPositions = m_pimpl->mann.getOutput().jointPositions;
    m_pimpl->mannInput.jointVelocities = m_pimpl->mann.getOutput().jointVelocities;

    // TODO here we assume that the x direction of the chest frame points forward. This may
    // cause issues on some robots (e.g., iCubV2.5)
    Eigen::Vector2d torsoDirection
        = iDynTree::toEigen(m_pimpl->kinDyn.getWorldTransform(m_pimpl->chestIndex).getRotation())
              .topLeftCorner<2, 1>();
    torsoDirection.normalize();

    // TODO here we assume that the x direction of the root frame points forward. This may cause
    // issues on some robots (e.g., iCubV2.5)
    const iDynTree::Transform I_H_root = m_pimpl->kinDyn.getWorldTransform(m_pimpl->rootIndex);
    Eigen::Vector2d rootDirection = iDynTree::toEigen(I_H_root.getRotation()).topLeftCorner<2, 1>();
    rootDirection.normalize();

    const Eigen::Vector2d newFacingDirection = torsoDirection + rootDirection;
    const double theta = std::atan2(newFacingDirection[1], newFacingDirection[0]);
    const manif::SE2d I_H_current_FD
        = manif::SE2d(I_H_root.getPosition()[0], I_H_root.getPosition()[1], theta);

    const manif::SE2d current_FD_H_previous_FD = I_H_current_FD.inverse() * m_pimpl->state.I_H_FD;

    auto updatePreviousInputWithTransform = [](std::deque<Eigen::Vector2d>& pastTrajectory,
                                               const auto& transformation,
                                               const Eigen::Vector2d& newDataPoint) {
        // past trajectory is seen as first in first out queue
        pastTrajectory.pop_front();
        for (auto& pastPoint : pastTrajectory)
        {
            pastPoint = transformation.act(pastPoint);
        }
        pastTrajectory.push_back(newDataPoint);
    };

    auto performSubsampling = [](const std::deque<Eigen::Vector2d>& pastTrajectory,
                                 Eigen::Ref<Eigen::Matrix2Xd> mannInput) -> void {
        const int sampleDelta = pastTrajectory.size() / (mannInput.cols() - 1);

        for (int i = 0; i < mannInput.cols(); i++)
        {
            mannInput.col(i) = pastTrajectory[i * sampleDelta];
        }
    };

    const auto current_FD_R_previous_FD = manif::SO2d(current_FD_H_previous_FD.angle());
    updatePreviousInputWithTransform(m_pimpl->state.pastProjectedBasePositions,
                                     current_FD_H_previous_FD,
                                     Eigen::Vector2d::Zero());
    updatePreviousInputWithTransform(m_pimpl->state.pastFacingDirection,
                                     current_FD_R_previous_FD,
                                     Eigen::Vector2d{1.0, 0.0});
    updatePreviousInputWithTransform(m_pimpl->state.pastProjectedBaseVelocity,
                                     current_FD_R_previous_FD,
                                     Eigen::Vector2d::Zero());

    performSubsampling(m_pimpl->state.pastProjectedBasePositions,
                       m_pimpl->mannInput.basePositionTrajectory.leftCols<6>());
    performSubsampling(m_pimpl->state.pastFacingDirection,
                       m_pimpl->mannInput.facingDirectionTrajectory.leftCols<6>());
    performSubsampling(m_pimpl->state.pastProjectedBaseVelocity,
                       m_pimpl->mannInput.baseVelocitiesTrajectory.leftCols<6>());

    m_pimpl->trajectoryBlending(mannOutput.futureBasePositionTrajectory,
                                input.desiredFutureBaseTrajectory,
                                1.5,
                                m_pimpl->mannInput.basePositionTrajectory.rightCols<6>());

    m_pimpl->trajectoryBlending(mannOutput.futureFacingDirectionTrajectory,
                                input.desiredFutureFacingDirections,
                                1.3,
                                m_pimpl->mannInput.facingDirectionTrajectory.rightCols<6>());

    m_pimpl->trajectoryBlending(mannOutput.futureBaseVelocitiesTrajectory,
                                input.desiredFutureBaseVelocities,
                                1.3,
                                m_pimpl->mannInput.baseVelocitiesTrajectory.rightCols<6>());

    if (!m_pimpl->mann.setInput(m_pimpl->mannInput))
    {
        log()->error("{} Unable to set the input to MANN network", logPrefix);
        return false;
    }

    constexpr double toleranceRobotStopped = 0.05;
    m_pimpl->isRobotStopped
        = m_pimpl->mannInput.basePositionTrajectory
              .isApprox(m_pimpl->state.previousMannInput.basePositionTrajectory,
                        toleranceRobotStopped)
          && m_pimpl->mannInput.baseVelocitiesTrajectory
                 .isApprox(m_pimpl->state.previousMannInput.baseVelocitiesTrajectory,
                           toleranceRobotStopped)
          && m_pimpl->mannInput.facingDirectionTrajectory
                 .isApprox(m_pimpl->state.previousMannInput.facingDirectionTrajectory,
                           toleranceRobotStopped)
          && m_pimpl->mannInput.jointVelocities
                 .isApprox(m_pimpl->state.previousMannInput.jointVelocities, toleranceRobotStopped)
          && m_pimpl->mannInput.jointPositions
                 .isApprox(m_pimpl->state.previousMannInput.jointPositions, toleranceRobotStopped);

    m_pimpl->state.I_H_FD = I_H_current_FD;
    m_pimpl->state.previousMannInput = m_pimpl->mannInput;

    return true;
}

void MANNAutoregressive::Impl::trajectoryBlending(
    Eigen::Ref<const Eigen::Matrix2Xd> mannOutputMatrix,
    Eigen::Ref<const Eigen::Matrix2Xd> desiredMatrix,
    const double& tau,
    Eigen::Ref<Eigen::Matrix2Xd> out)
{
    for (int i = 0; i < mannOutputMatrix.cols(); i++)
    {
        const double T = std::pow((i + 1.0) / double(mannOutputMatrix.cols()), tau);
        out.col(i) = (1 - T) * mannOutputMatrix.col(i) + T * desiredMatrix.col(i);
    }
}

bool MANNAutoregressive::advance()
{
    using namespace std::chrono_literals;
    using namespace BipedalLocomotion::GenericContainer::literals;

    constexpr auto logPrefix = "[MANNAutoregressive::advance]";

    // invalidate the output
    m_pimpl->isOutputValid = false;

    // check the status of the state machine.
    if (m_pimpl->fsmState != Impl::FSM::Reset && m_pimpl->fsmState != Impl::FSM::Running)
    {
        log()->error("{} You need to reset the network before calling advance the first time. "
                     "Please call reset().",
                     logPrefix);
        return false;
    }

    // perform one iteration of MANN
    if (!m_pimpl->mann.advance() || !m_pimpl->mann.isOutputValid())
    {
        log()->error("{} Unable to compute the output of the network.", logPrefix);
        return false;
    }

    // get the output of the network
    const auto& mannOutput = m_pimpl->mann.getOutput();

    // Extract the new base orientation from the output
    // we reset the kinDynObject
    // TODO initialize also the base orientation dynamics

    // Integrate the base orientation
    // if the robot is stopped (i.e, if the current mann input and the previous one are the same)
    // we set the yaw rate equal to zero
    const double yawRate = m_pimpl->isRobotStopped ? 0 : mannOutput.projectedBaseVelocity.angle();
    manif::SO3Tangentd baseAngularVelocity(Eigen::Vector3d{0, 0, yawRate});
    m_pimpl->baseOrientationDynamics->setControlInput({baseAngularVelocity});
    m_pimpl->integrator.integrate(0s, m_pimpl->dT);

    // The following code is required to compute a kinematicaly feasible base position. This problem
    // is solved by applying legged odometry and considering the robot foot composed by n corners.
    // The number and the location of the corner are set in the parameter handler.
    manif::SE3d I_H_base
        = manif::SE3d(iDynTree::toEigen(m_pimpl->kinDyn.getWorldBaseTransform().getPosition()),
                      m_pimpl->integrator.getSolution().get_from_hash<"LieGroup"_h>());

    manif::SE3d::Tangent baseVelocity = manif::SE3d::Tangent::Zero();
    if (!m_pimpl->kinDyn.setRobotState(I_H_base.transform(),
                                       mannOutput.jointPositions,
                                       baseVelocity.coeffs(),
                                       mannOutput.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to set the robot state in the kindyncomputations object.",
                     logPrefix);
        return false;
    }

    // The following function evaluate the position of the corners of a foot in the inertial frame
    auto evaluateFootCornersPosition
        = [this](int index,
                 const std::vector<Contacts::Corner>& corners) -> std::vector<Eigen::Vector3d> {
        using namespace BipedalLocomotion::Conversions;

        std::vector<Eigen::Vector3d> footCorners;
        const manif::SE3d I_H_foot = toManifPose(m_pimpl->kinDyn.getWorldTransform(index));

        // for each corner we compute the position in the inertial frame
        for (const auto& corner : corners)
        {
            footCorners.emplace_back(I_H_foot.act(corner.position));
        }

        return footCorners;
    };

    // get the position of the corners of the left and right foot in the inertial frame
    const std::vector<Eigen::Vector3d> leftFootCorners
        = evaluateFootCornersPosition(m_pimpl->leftFoot.discreteGeometryContact.index,
                                      m_pimpl->leftFoot.discreteGeometryContact.corners);
    const std::vector<Eigen::Vector3d> rightFootCorners
        = evaluateFootCornersPosition(m_pimpl->rightFoot.discreteGeometryContact.index,
                                      m_pimpl->rightFoot.discreteGeometryContact.corners);

    // find the support vertex
    auto leftLowerCorner = std::min_element(leftFootCorners.begin(),
                                            leftFootCorners.end(),
                                            [](const Eigen::Vector3d& a, const Eigen::Vector3d& b) {
                                                return a[2] < b[2];
                                            });
    auto rightLowerCorner = std::min_element(rightFootCorners.begin(),
                                             rightFootCorners.end(),
                                             [](const Eigen::Vector3d& a,
                                                const Eigen::Vector3d& b) { return a[2] < b[2]; });

    // find the support foot associated to the support vertex
    auto supportFootPtr = &m_pimpl->leftFoot;
    auto supportCorner = leftLowerCorner;
    int supportCornerIndex = std::distance(leftFootCorners.begin(), leftLowerCorner);
    if ((*leftLowerCorner)[2] > (*rightLowerCorner)[2])
    {
        supportFootPtr = &m_pimpl->rightFoot;
        supportCorner = rightLowerCorner;
        // find the position of the support corner in the array
        supportCornerIndex = std::distance(rightFootCorners.begin(), rightLowerCorner);
    }

    // we update the projected contact position in the world frame if and only if the support foot
    // changed
    // TODO(Giulio): In case you want to update the projected contact position if the support foot
    // did not change but the lower corner yes you should add this condition in the following if
    // "|| (supportFootPtr == m_pimpl->supportFootPtr && supportCornerIndex != m_pimpl->supportCornerIndex)""
    // In the original version of Adherent this behaviour was enabled.
    if (supportFootPtr != m_pimpl->supportFootPtr)
    {
        m_pimpl->supportCornerIndex = supportCornerIndex;
        m_pimpl->projectedContactPositionInWorldFrame[0] = (*supportCorner)[0];
        m_pimpl->projectedContactPositionInWorldFrame[1] = (*supportCorner)[1];
    }

    // compute the base position with legged odometry
    const iDynTree::Transform base_H_supportFoot
        = m_pimpl->kinDyn.getRelativeTransform(m_pimpl->rootIndex,
                                               supportFootPtr->discreteGeometryContact.index);
    const iDynTree::Transform supportVertex_H_base
        = (iDynTree::Transform(iDynTree::Rotation::Identity(),
                               iDynTree::Position(supportFootPtr->discreteGeometryContact
                                                      .corners[m_pimpl->supportCornerIndex]
                                                      .position))
               .inverse()
           * base_H_supportFoot.inverse());

    const iDynTree::Transform I_H_supportVertex
        = iDynTree::Transform(m_pimpl->kinDyn
                                  .getWorldTransform(supportFootPtr->discreteGeometryContact.index)
                                  .getRotation(),
                              iDynTree::Position(m_pimpl->projectedContactPositionInWorldFrame));
    const iDynTree::Transform I_H_base_iDynTree = I_H_supportVertex * supportVertex_H_base;

    I_H_base.translation(iDynTree::toEigen(I_H_base_iDynTree.getPosition()));

    // compute the base velocity
    if (!m_pimpl->kinDyn.getFrameFreeFloatingJacobian(supportFootPtr->discreteGeometryContact.index,
                                                      m_pimpl->supportFootJacobian))
    {
        log()->error("{} Unable to get the frame jacobian for the frame named '{}'.",
                     logPrefix,
                     m_pimpl->kinDyn.model().getFrameName(
                         supportFootPtr->discreteGeometryContact.index));
        return false;
    }
    baseVelocity.coeffs().noalias()
        = -m_pimpl->supportFootJacobian.leftCols<6>().colPivHouseholderQr().solve(
            m_pimpl->supportFootJacobian.rightCols(m_pimpl->kinDyn.getNrOfDegreesOfFreedom())
            * mannOutput.jointVelocities);

    // update the kindyn object with the kinematically feasible base
    if (!m_pimpl->kinDyn.setRobotState(I_H_base.transform(),
                                       mannOutput.jointPositions,
                                       baseVelocity.coeffs(),
                                       mannOutput.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to set the robot state in the kindyncomputations object.",
                     logPrefix);
        return false;
    }

    // From here we store the output of MANNAutoregressive
    const double leftFootHeight
        = m_pimpl->kinDyn.getWorldTransform(m_pimpl->leftFoot.discreteGeometryContact.index)
              .getPosition()[2];
    const double rightFootHeight
        = m_pimpl->kinDyn.getWorldTransform(m_pimpl->rightFoot.discreteGeometryContact.index)
              .getPosition()[2];

    // The updateContact function utilizes a Schmitt trigger to identify contact. The trigger's
    // output switches to True when the input exceeds a certain threshold for a duration of ΔT,
    // where ΔT represents a time threshold. It is crucial to note that in this case, we desire the
    // trigger to return True when the foot is in contact and False otherwise. To achieve this,
    // considering that the foot height is always positive, we provide the negative of the foot
    // height to the trigger. This ensures that the trigger returns True when the foot is close to
    // the ground.
    m_pimpl->updateContact(-leftFootHeight,
                           m_pimpl->leftFootSchmittTrigger,
                           m_pimpl->output.leftFootSchmittTriggerState,
                           m_pimpl->output.leftFoot);
    m_pimpl->updateContact(-rightFootHeight,
                           m_pimpl->rightFootSchmittTrigger,
                           m_pimpl->output.rightFootSchmittTriggerState,
                           m_pimpl->output.rightFoot);

    m_pimpl->output.jointsPosition = mannOutput.jointPositions;
    m_pimpl->output.basePose = I_H_base;
    m_pimpl->output.currentTime = m_pimpl->currentTime;
    m_pimpl->output.comPosition = iDynTree::toEigen(m_pimpl->kinDyn.getCenterOfMassPosition());
    m_pimpl->output.angularMomentum
        = iDynTree::toEigen(m_pimpl->kinDyn.getCentroidalTotalMomentum().getAngularVec3());

    // store the previous support foot and corner
    m_pimpl->supportFootPtr = supportFootPtr;

    // advance the internal time
    m_pimpl->currentTime += m_pimpl->dT;

    m_pimpl->isOutputValid = true;
    m_pimpl->fsmState = Impl::FSM::Running;

    return true;
}

bool MANNAutoregressive::isOutputValid() const
{
    // the output must be valid and the state machine must be in running phase
    return m_pimpl->isOutputValid && m_pimpl->fsmState == Impl::FSM::Running;
}

const MANNAutoregressive::Output& MANNAutoregressive::getOutput() const
{
    return m_pimpl->output;
}

const MANNInput& MANNAutoregressive::getMANNInput() const
{
    return m_pimpl->mannInput;
}

const MANNAutoregressive::AutoregressiveState& MANNAutoregressive::getAutoregressiveState() const
{
    return m_pimpl->state;
}
