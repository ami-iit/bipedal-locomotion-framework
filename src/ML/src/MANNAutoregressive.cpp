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

#include <iDynTree/Model.h>
#include <manif/SE3.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

MANNFootState MANNFootState::generateFootState(iDynTree::KinDynComputations& kinDyn,
                                               const std::vector<Eigen::Vector3d>& corners,
                                               const std::string& footName,
                                               int footIndex)
{
    using namespace BipedalLocomotion::Conversions;

    MANNFootState dummyState;
    // the schmitt trigger is initialized to true since we assume that the foot is in contact
    dummyState.schmittTriggerState.state = true;
    dummyState.contact.name = footName;
    dummyState.contact.index = footIndex;
    dummyState.contact.isActive = dummyState.schmittTriggerState.state;
    dummyState.contact.switchTime = std::chrono::nanoseconds::zero();
    dummyState.contact.pose = toManifPose(kinDyn.getWorldTransform(footIndex));

    // we need to force the height of the foot to be zero.
    dummyState.contact.pose.translation(Eigen::Vector3d{dummyState.contact.pose.translation()[0],
                                                        dummyState.contact.pose.translation()[1],
                                                        0.0});
    dummyState.corners = corners;
    return dummyState;
}

MANNAutoregressive::AutoregressiveState
MANNAutoregressive::AutoregressiveState::generateDummyAutoregressiveState(
    const MANNInput& input,
    const MANNOutput& output,
    const manif::SE3d& I_H_B,
    const MANNFootState& leftFootState,
    const MANNFootState& rightFootState,
    int mocapFrameRate,
    const std::chrono::nanoseconds& pastProjectedBaseHorizon)
{
    MANNAutoregressive::AutoregressiveState autoregressiveState;

    const int horizon
        = std::chrono::duration_cast<std::chrono::seconds>(pastProjectedBaseHorizon).count();
    const std::size_t lengthOfPresentPlusPastTrajectory = 1 + mocapFrameRate * horizon;
    autoregressiveState.pastProjectedBasePositions
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{0.0, 0.0}};
    autoregressiveState.pastProjectedBaseVelocity
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{0.0, 0.0}};
    autoregressiveState.pastFacingDirection
        = std::deque<Eigen::Vector2d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector2d{1.0, 0.0}};
    autoregressiveState.I_H_FD = manif::SE2d::Identity();

    autoregressiveState.previousMANNInput = input;
    autoregressiveState.previousMANNOutput = output;

    autoregressiveState.I_H_B = I_H_B;
    autoregressiveState.leftFootState = leftFootState;
    autoregressiveState.rightFootState = rightFootState;
    autoregressiveState.projectedContactPositionInWorldFrame.setZero();

    autoregressiveState.time = std::chrono::nanoseconds::zero();

    return autoregressiveState;
}

struct MANNAutoregressive::Impl
{
    MANN mann;
    MANNInput mannInput;
    int projectedBaseDatapoints;
    iDynTree::KinDynComputations kinDyn;
    MANNAutoregressiveOutput output;

    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};
    std::chrono::nanoseconds dT;
    std::chrono::nanoseconds pastProjectedBaseHorizon;
    int mocapFrameRate;

    std::shared_ptr<ContinuousDynamicalSystem::SO3Dynamics> baseOrientationDynamics;
    ContinuousDynamicalSystem::ForwardEuler<ContinuousDynamicalSystem::SO3Dynamics> integrator;

    Eigen::Vector3d gravity; // This is required by the kindyncomputations object

    AutoregressiveState state;
    Eigen::MatrixXd supportFootJacobian;

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

    bool updateContact(const double referenceHeight,
                       Math::SchmittTrigger& trigger,
                       Contacts::EstimatedContact& contact);
};

bool MANNAutoregressive::Impl::updateContact(const double referenceHeight,
                                             Math::SchmittTrigger& trigger,
                                             Contacts::EstimatedContact& contact)
{
    constexpr auto logPrefix = "[MANNAutoregressive::Impl::updateContact]";
    Math::SchmittTriggerInput footSchmittInput;
    footSchmittInput.rawValue = referenceHeight;
    footSchmittInput.time = this->currentTime;

    const bool wasContactActive = trigger.getState().state;
    if (!trigger.setInput(footSchmittInput))
    {
        log()->error("{} Unable to set the input to the Schmitt trigger.", logPrefix);
        return false;
    }

    if (!trigger.advance())
    {
        log()->error("{} Unable to advance the Schmitt trigger.", logPrefix);
        return false;
    }

    // if the contact was not active and the trigger is active, then we need to update the contact
    if (!wasContactActive && trigger.getState().state)
    {
        using namespace BipedalLocomotion::Conversions;
        contact.isActive = true;
        contact.switchTime = this->currentTime;
        contact.pose = toManifPose(this->kinDyn.getWorldTransform(contact.index));
    }
    // if the contact was active and the trigger is not active, then we need to update the contact
    else if (wasContactActive && !(trigger.getState().state))
    {
        contact.isActive = false;
    }

    contact.lastUpdateTime = this->currentTime;
    return true;
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
                                              std::vector<Eigen::Vector3d>& corners) -> bool {
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
        corners.resize(numberOfCorners);

        // for each corner we need to get its position in the frame attached to the foot
        for (std::size_t j = 0; j < numberOfCorners; j++)
        {
            if (!contactHandler->getParameter("corner_" + std::to_string(j), corners[j]))
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

    // the following lambda function is useful to initialize the Schmitt trigger
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

    // The dynamical system state is set in the reset function. Here we only need to set the
    // dynamical system in the integrator
    m_pimpl->baseOrientationDynamics = std::make_shared<ContinuousDynamicalSystem::SO3Dynamics>();
    if (!m_pimpl->integrator.setDynamicalSystem(m_pimpl->baseOrientationDynamics))
    {
        log()->error("{} Unable to set the dynamical system in the integrator.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("sampling_time", m_pimpl->dT))
    {
        log()->error("{} Unable to find the parameter named '{}'.", logPrefix, "sampling_time");
        return false;
    }

    if (!m_pimpl->integrator.setIntegrationStep(m_pimpl->dT))
    {
        log()->error("{} Unable to set the integration step.", logPrefix);
        return false;
    }

    std::string forwardDirection;
    if (!ptr->getParameter("forward_direction", forwardDirection) || forwardDirection != "x")
    {
        log()->error("{} Only forward_direction equal to 'x' is supported.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("mocap_frame_rate", m_pimpl->mocapFrameRate))
    {
        log()->error("{} Unable to find the parameter named '{}'.", logPrefix, "mocap_frame_rate");
        return false;
    }

    if (!ptr->getParameter("past_projected_base_horizon", m_pimpl->pastProjectedBaseHorizon))
    {
        log()->error("{} Unable to find the parameter named '{}'.",
                     logPrefix,
                     "past_projected_base_horizon");
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
    ok = ok && getFrameIndex("left_foot_frame_name", m_pimpl->state.leftFootState.contact.index);
    ok = ok && getFrameIndex("right_foot_frame_name", m_pimpl->state.rightFootState.contact.index);

    // Get the position of the corners in the foot frame
    ok = ok && getContactCorners("LEFT_FOOT", m_pimpl->state.leftFootState.corners);
    ok = ok && getContactCorners("RIGHT_FOOT", m_pimpl->state.rightFootState.corners);

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

    if (!cloneHandler->getParameter("projected_base_datapoints", m_pimpl->projectedBaseDatapoints))
    {
        log()->error("{} Unable to find the parameter named '{}'.",
                     logPrefix,
                     "projected_base_datapoints");
        return false;
    }

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

    m_pimpl->mannInput.basePositionTrajectory.resize(2, m_pimpl->projectedBaseDatapoints);
    m_pimpl->mannInput.baseVelocitiesTrajectory.resize(2, m_pimpl->projectedBaseDatapoints);
    m_pimpl->mannInput.facingDirectionTrajectory.resize(2, m_pimpl->projectedBaseDatapoints);
    m_pimpl->mannInput.jointPositions.resize(m_pimpl->kinDyn.getNrOfDegreesOfFreedom());
    m_pimpl->mannInput.jointVelocities.resize(m_pimpl->kinDyn.getNrOfDegreesOfFreedom());

    return ok;
}

bool MANNAutoregressive::populateInitialAutoregressiveState(
    Eigen::Ref<const Eigen::VectorXd> jointPositions,
    const manif::SE3d& basePosition,
    MANNAutoregressive::AutoregressiveState& state)
{
    constexpr auto logPrefix = "[MANNAutoregressive::populateInitialAutoregressiveState]";

    // set the base velocity to zero since we do not need to evaluate any quantity related to it
    Eigen::Matrix<double, 6, 1> baseVelocity;
    baseVelocity.setZero();

    // generate the input of the network we assume that the robot is stopped and the facing
    // direction is the x axis
    const auto input = MANNInput::generateDummyMANNInput(jointPositions, //
                                                         m_pimpl->projectedBaseDatapoints);
    const auto output = MANNOutput::generateDummyMANNOutput(jointPositions, //
                                                            m_pimpl->projectedBaseDatapoints / 2);

    if (!m_pimpl->kinDyn.setRobotState(basePosition.transform(),
                                       input.jointPositions,
                                       baseVelocity,
                                       input.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to reset the kindyncomputations object.", logPrefix);
        return false;
    }

    // we assume that the robot is in double support and the feet are on the ground
    state = MANNAutoregressive::AutoregressiveState::generateDummyAutoregressiveState(
        input,
        output,
        basePosition,
        MANNFootState::generateFootState(m_pimpl->kinDyn,
                                         m_pimpl->state.leftFootState.corners,
                                         "left_foot",
                                         m_pimpl->state.leftFootState.contact.index),
        MANNFootState::generateFootState(m_pimpl->kinDyn,
                                         m_pimpl->state.rightFootState.corners,
                                         "right_foot",
                                         m_pimpl->state.rightFootState.contact.index),
        m_pimpl->mocapFrameRate,
        m_pimpl->pastProjectedBaseHorizon);

    return true;
}

bool MANNAutoregressive::reset(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                               const manif::SE3d& basePose)
{
    AutoregressiveState autoregressiveState;
    if (!this->populateInitialAutoregressiveState(jointPositions, basePose, autoregressiveState))
    {
        log()->error("[MANNAutoregressive::reset] Unable to populate the initial autoregressive "
                     "state.");
        return false;
    }
    return this->reset(autoregressiveState);
}

bool MANNAutoregressive::reset(const AutoregressiveState& state)
{
    constexpr auto logPrefix = "[MANNAutoregressive::reset]";

    if (m_pimpl->fsmState != Impl::FSM::Initialized && m_pimpl->fsmState != Impl::FSM::Running
        && m_pimpl->fsmState != Impl::FSM::Reset)
    {
        log()->error("{} MANNAutoregressive has not been initialized.", logPrefix);
        return false;
    }

    // copy the autoregressive state
    m_pimpl->state = state;
    m_pimpl->currentTime = m_pimpl->state.time;

    // /the output is not valid since we are resetting the trajectory planner
    m_pimpl->isOutputValid = false;

    if (!m_pimpl->baseOrientationDynamics->setState({m_pimpl->state.I_H_B.quat()}))
    {
        log()->error("{} Unable to reset the base orientation dynamics.", logPrefix);
        return false;
    }

    if (!m_pimpl->leftFootSchmittTrigger.setState( //
            m_pimpl->state.leftFootState.schmittTriggerState))
    {
        log()->error("{} Unable to set the state of the left foot Schmitt trigger.", logPrefix);
        return false;
    }

    if (!m_pimpl->rightFootSchmittTrigger.setState( //
            m_pimpl->state.rightFootState.schmittTriggerState))
    {
        log()->error("{} Unable to set the state of the right foot Schmitt trigger.", logPrefix);
        return false;
    }

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

    // get the output of the network computed at the previous iteration
    const MANNOutput& previousMANNOutput = m_pimpl->state.previousMANNOutput;

    // the joint positions and velocities are considered as new input
    m_pimpl->mannInput.jointPositions = previousMANNOutput.jointPositions;
    m_pimpl->mannInput.jointVelocities = previousMANNOutput.jointVelocities;

    // we set the base velocity to zero since we do not need to evaluate any quantity related to it
    const Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    if (!m_pimpl->kinDyn.setRobotState(m_pimpl->state.I_H_B.transform(),
                                       m_pimpl->mannInput.jointPositions,
                                       baseVelocity,
                                       m_pimpl->mannInput.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to set the robot state in the kinDyn object.", logPrefix);
        return false;
    }

    // TODO here we assume that the x direction of the chest frame points forward. This may
    // cause issues on some robots (e.g., iCubV2.5)
    Eigen::Vector2d torsoDirection
        = iDynTree::toEigen(m_pimpl->kinDyn.getWorldTransform(m_pimpl->chestIndex).getRotation())
              .topLeftCorner<2, 1>();
    torsoDirection.normalize();

    // TODO here we assume that the x direction of the root frame points forward. This may cause
    // issues on some robots (e.g., iCubV2.5)
    Eigen::Vector2d rootDirection = m_pimpl->state.I_H_B.rotation().topLeftCorner<2, 1>();
    rootDirection.normalize();

    // we evaluate the forward direction frame. This 2D frame is centered in the ground projected
    // base link and the x axis is pointing to the average of the torso and root direction.
    const Eigen::Vector2d newFacingDirection = torsoDirection + rootDirection;
    const double theta = std::atan2(newFacingDirection[1], newFacingDirection[0]);
    const manif::SE2d I_H_current_FD = manif::SE2d(m_pimpl->state.I_H_B.translation()[0],
                                                   m_pimpl->state.I_H_B.translation()[1],
                                                   theta);

    // we compute the transformation between the current and the previous forward direction
    const manif::SE2d current_FD_H_previous_FD = I_H_current_FD.inverse() * m_pimpl->state.I_H_FD;

    // this lambda function takes the past trajectory. then it shifts it and append the new
    // datapoint.
    auto updatePreviousInputWithTransform = [](std::deque<Eigen::Vector2d>& pastTrajectory,
                                               const auto& transformation,
                                               const Eigen::Vector2d& newDataPoint) {
        // past trajectory is seen as first in first out queue
        pastTrajectory.pop_front();
        for (auto& pastPoint : pastTrajectory)
        {
            // we apply the transformation to the past trajectory
            // read it as point_{i+1} = transformation * point_{i}
            // where i is the time index
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

    // This part update update the state.
    const manif::SO2d current_FD_R_previous_FD = manif::SO2d(current_FD_H_previous_FD.angle());
    updatePreviousInputWithTransform(m_pimpl->state.pastProjectedBasePositions,
                                     current_FD_H_previous_FD,
                                     Eigen::Vector2d::Zero());
    updatePreviousInputWithTransform(m_pimpl->state.pastFacingDirection,
                                     current_FD_R_previous_FD,
                                     Eigen::Vector2d{1.0, 0.0});
    updatePreviousInputWithTransform(m_pimpl->state.pastProjectedBaseVelocity,
                                     current_FD_R_previous_FD,
                                     Eigen::Vector2d::Zero());

    // we subsample the past trajectory to get the input of the network

    const int halfProjectedBasedHorizon = m_pimpl->projectedBaseDatapoints / 2;
    performSubsampling(m_pimpl->state.pastProjectedBasePositions,
                       m_pimpl->mannInput.basePositionTrajectory.leftCols(
                           halfProjectedBasedHorizon));
    performSubsampling(m_pimpl->state.pastFacingDirection,
                       m_pimpl->mannInput.facingDirectionTrajectory.leftCols(
                           halfProjectedBasedHorizon));
    performSubsampling(m_pimpl->state.pastProjectedBaseVelocity,
                       m_pimpl->mannInput.baseVelocitiesTrajectory.leftCols(
                           halfProjectedBasedHorizon));

    constexpr double tauBasePosition = 1.5;
    m_pimpl->trajectoryBlending(previousMANNOutput.futureBasePositionTrajectory,
                                input.desiredFutureBaseTrajectory,
                                tauBasePosition,
                                m_pimpl->mannInput.basePositionTrajectory.rightCols(
                                    halfProjectedBasedHorizon));

    constexpr double tauFacingDirection = 1.3;
    m_pimpl->trajectoryBlending(previousMANNOutput.futureFacingDirectionTrajectory,
                                input.desiredFutureFacingDirections,
                                tauFacingDirection,
                                m_pimpl->mannInput.facingDirectionTrajectory.rightCols(
                                    halfProjectedBasedHorizon));

    constexpr double tauBaseVelocity = 1.3;
    m_pimpl->trajectoryBlending(previousMANNOutput.futureBaseVelocitiesTrajectory,
                                input.desiredFutureBaseVelocities,
                                tauBaseVelocity,
                                m_pimpl->mannInput.baseVelocitiesTrajectory.rightCols(
                                    halfProjectedBasedHorizon));

    if (!m_pimpl->mann.setInput(m_pimpl->mannInput))
    {
        log()->error("{} Unable to set the input to MANN network", logPrefix);
        return false;
    }

    // to check if the robot is stopped we need to compare the current input with the previous one.
    // Please notice that the previous input is now contained in m_pimpl->state.mannInput that will
    // be updated at the end of this function.
    constexpr double toleranceRobotStopped = 0.05;
    m_pimpl->isRobotStopped
        = m_pimpl->mannInput.basePositionTrajectory
              .isApprox(m_pimpl->state.previousMANNInput.basePositionTrajectory,
                        toleranceRobotStopped)
          && m_pimpl->mannInput.baseVelocitiesTrajectory
                 .isApprox(m_pimpl->state.previousMANNInput.baseVelocitiesTrajectory,
                           toleranceRobotStopped)
          && m_pimpl->mannInput.facingDirectionTrajectory
                 .isApprox(m_pimpl->state.previousMANNInput.facingDirectionTrajectory,
                           toleranceRobotStopped)
          && m_pimpl->mannInput.jointVelocities
                 .isApprox(m_pimpl->state.previousMANNInput.jointVelocities, //
                           toleranceRobotStopped)
          && m_pimpl->mannInput.jointPositions
                 .isApprox(m_pimpl->state.previousMANNInput.jointPositions, //
                           toleranceRobotStopped);

    // store the autoregressive state
    m_pimpl->state.I_H_FD = I_H_current_FD;
    m_pimpl->state.previousMANNInput = m_pimpl->mannInput;

    // the output is not valid anymore since we set a new input
    m_pimpl->isOutputValid = false;

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

    // Integrate the base orientation
    // if the robot is stopped (i.e, if the current mann input and the previous one are the same)
    // we set the yaw rate equal to zero
    const double yawRate = m_pimpl->isRobotStopped ? 0.0 : mannOutput.projectedBaseVelocity.angle();
    const manif::SO3Tangentd baseAngularVelocity(Eigen::Vector3d{0, 0, yawRate});
    if (!m_pimpl->baseOrientationDynamics->setControlInput({baseAngularVelocity}))
    {
        log()->error("{} Unable to set the control input to the base orientation dynamics.",
                     logPrefix);
        return false;
    }

    if (!m_pimpl->integrator.integrate(0s, m_pimpl->dT))
    {
        log()->error("{} Unable to integrate the base orientation dynamics.", logPrefix);
        return false;
    }

    // The following code is required to compute a kinematicaly feasible base position. This problem
    // is solved by applying legged odometry and considering the robot foot composed by n corners.
    // The number and the location of the corner are set in the parameter handler.
    manif::SE3d I_H_base
        = manif::SE3d(m_pimpl->state.I_H_B.translation(),
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
                 const std::vector<Eigen::Vector3d>& corners) -> std::vector<Eigen::Vector3d> {
        using namespace BipedalLocomotion::Conversions;

        std::vector<Eigen::Vector3d> footCorners;
        const manif::SE3d I_H_foot = toManifPose(m_pimpl->kinDyn.getWorldTransform(index));

        // for each corner we compute the position in the inertial frame
        for (const auto& corner : corners)
        {
            footCorners.emplace_back(I_H_foot.act(corner));
        }

        return footCorners;
    };

    // get the position of the corners of the left and right foot in the inertial frame
    const std::vector<Eigen::Vector3d> leftFootCorners
        = evaluateFootCornersPosition(m_pimpl->state.leftFootState.contact.index,
                                      m_pimpl->state.leftFootState.corners);
    const std::vector<Eigen::Vector3d> rightFootCorners
        = evaluateFootCornersPosition(m_pimpl->state.rightFootState.contact.index,
                                      m_pimpl->state.rightFootState.corners);

    // find the support vertex
    auto leftLowerCorner = std::min_element(leftFootCorners.begin(),
                                            leftFootCorners.end(),
                                            [](const Eigen::Vector3d& a, //
                                               const Eigen::Vector3d& b) { return a[2] < b[2]; });
    auto rightLowerCorner = std::min_element(rightFootCorners.begin(),
                                             rightFootCorners.end(),
                                             [](const Eigen::Vector3d& a, //
                                                const Eigen::Vector3d& b) { return a[2] < b[2]; });

    // find the support foot associated to the support vertex
    auto supportFootPtr = &m_pimpl->state.leftFootState;
    auto supportCorner = leftLowerCorner;
    // find the position of the support corner in the array
    int supportCornerIndex = std::distance(leftFootCorners.begin(), leftLowerCorner);
    SupportFoot currentSupportFoot = SupportFoot::Left;
    if ((*leftLowerCorner)[2] > (*rightLowerCorner)[2])
    {
        supportFootPtr = &m_pimpl->state.rightFootState;
        supportCorner = rightLowerCorner;
        // find the position of the support corner in the array
        supportCornerIndex = std::distance(rightFootCorners.begin(), rightLowerCorner);
        currentSupportFoot = SupportFoot::Right;
    }

    // we update the projected contact position in the world frame if and only if the support foot
    // changed
    // TODO(Giulio): In case you want to update the projected contact position if the support foot
    // did not change but the lower corner yes you should add this condition in the following if
    // "|| (supportFootPtr == m_pimpl->supportFootPtr && supportCornerIndex !=
    // m_pimpl->supportCornerIndex)"" In the original version of Adherent this behaviour was
    // enabled.
    if (currentSupportFoot != m_pimpl->state.supportFoot)
    {
        m_pimpl->state.supportCornerIndex = supportCornerIndex;
        m_pimpl->state.projectedContactPositionInWorldFrame[0] = (*supportCorner)[0];
        m_pimpl->state.projectedContactPositionInWorldFrame[1] = (*supportCorner)[1];
    }

    // compute the base position with legged odometry
    const iDynTree::Transform base_H_supportFoot
        = m_pimpl->kinDyn.getRelativeTransform(m_pimpl->rootIndex, supportFootPtr->contact.index);

    // read it as supportVertex_H_base = supportVertex_H_supportFoot * base_H_supportFoot.inverse()
    const iDynTree::Transform supportVertex_H_base
        = iDynTree::Transform(iDynTree::Rotation::Identity(),
                              iDynTree::Position(
                                  supportFootPtr->corners[m_pimpl->state.supportCornerIndex]))
              .inverse()
          * base_H_supportFoot.inverse();

    const iDynTree::Transform I_H_supportVertex
        = iDynTree::Transform(m_pimpl->kinDyn.getWorldTransform(supportFootPtr->contact.index)
                                  .getRotation(),
                              iDynTree::Position(
                                  m_pimpl->state.projectedContactPositionInWorldFrame));
    const iDynTree::Transform I_H_base_iDynTree = I_H_supportVertex * supportVertex_H_base;

    I_H_base.translation(iDynTree::toEigen(I_H_base_iDynTree.getPosition()));

    // compute the base velocity
    if (!m_pimpl->kinDyn.getFrameFreeFloatingJacobian(supportFootPtr->contact.index,
                                                      m_pimpl->supportFootJacobian))
    {
        log()->error("{} Unable to get the frame jacobian for the frame named '{}'.",
                     logPrefix,
                     m_pimpl->kinDyn.model().getFrameName(supportFootPtr->contact.index));
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
        = m_pimpl->kinDyn.getWorldTransform(m_pimpl->state.leftFootState.contact.index)
              .getPosition()[2];
    const double rightFootHeight
        = m_pimpl->kinDyn.getWorldTransform(m_pimpl->state.rightFootState.contact.index)
              .getPosition()[2];

    // The updateContact function utilizes a Schmitt trigger to identify contact. The trigger's
    // output switches to True when the input exceeds a certain threshold for a duration of ΔT,
    // where ΔT represents a time threshold. It is crucial to note that in this case, we desire the
    // trigger to return True when the foot is in contact and False otherwise. To achieve this,
    // considering that the foot height is always positive, we provide the negative of the foot
    // height to the trigger. This ensures that the trigger returns True when the foot is close to
    // the ground.
    if (!m_pimpl->updateContact(-leftFootHeight,
                                m_pimpl->leftFootSchmittTrigger,
                                m_pimpl->state.leftFootState.contact))
    {
        log()->error("{} Unable to update the contact information for the left foot.", logPrefix);
        return false;
    }
    m_pimpl->state.leftFootState.schmittTriggerState = m_pimpl->leftFootSchmittTrigger.getState();

    if (!m_pimpl->updateContact(-rightFootHeight,
                                m_pimpl->rightFootSchmittTrigger,
                                m_pimpl->state.rightFootState.contact))
    {
        log()->error("{} Unable to update the contact information for the right foot.", logPrefix);
        return false;
    }
    m_pimpl->state.rightFootState.schmittTriggerState = m_pimpl->rightFootSchmittTrigger.getState();

    m_pimpl->state.I_H_B = I_H_base;
    m_pimpl->state.previousMANNOutput = mannOutput;
    m_pimpl->state.supportFoot = currentSupportFoot;

    // populate the output
    m_pimpl->output.jointsPosition = m_pimpl->state.previousMANNOutput.jointPositions;
    m_pimpl->output.basePose = m_pimpl->state.I_H_B;
    m_pimpl->output.baseVelocity = baseVelocity;
    m_pimpl->output.currentTime = m_pimpl->state.time;
    m_pimpl->output.comPosition = iDynTree::toEigen(m_pimpl->kinDyn.getCenterOfMassPosition());
    m_pimpl->output.angularMomentum
        = iDynTree::toEigen(m_pimpl->kinDyn.getCentroidalTotalMomentum().getAngularVec3());
    m_pimpl->output.leftFoot = m_pimpl->state.leftFootState.contact;
    m_pimpl->output.rightFoot = m_pimpl->state.rightFootState.contact;

    m_pimpl->isOutputValid = true;
    m_pimpl->fsmState = Impl::FSM::Running;

    // advance the internal time
    m_pimpl->currentTime += m_pimpl->dT;

    // Since the advance function prepares the state for the next iteration, we need to update the
    // time in the state struct after incrementing it by dT
    m_pimpl->state.time = m_pimpl->currentTime;

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
