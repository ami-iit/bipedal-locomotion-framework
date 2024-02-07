/**
 * @file velMANNAutoregressive.cpp
 * @authors Paolo Maria Viceconte, Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <typeinfo>

#include <chrono>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/ForwardEuler.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/SO3Dynamics.h>
#include <BipedalLocomotion/Conversions/ManifConversions.h>
#include <BipedalLocomotion/ML/velMANN.h>
#include <BipedalLocomotion/ML/velMANNAutoregressive.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Model/Model.h>
#include <manif/SE3.h>

using namespace BipedalLocomotion::ML;
using namespace BipedalLocomotion;

velMANNFootState velMANNFootState::generateFootState(iDynTree::KinDynComputations& kinDyn,
                                               const std::vector<Eigen::Vector3d>& corners,
                                               const std::string& footName,
                                               int footIndex)
{
    using namespace BipedalLocomotion::Conversions;

    velMANNFootState dummyState;
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

velMANNAutoregressive::AutoregressiveState
velMANNAutoregressive::AutoregressiveState::generateDummyAutoregressiveState(
    const velMANNInput& input,
    const velMANNOutput& output,
    const manif::SE3d& I_H_B,
    const velMANNFootState& leftFootState,
    const velMANNFootState& rightFootState,
    int mocapFrameRate,
    const std::chrono::nanoseconds& pastProjectedBaseHorizon)
{
    velMANNAutoregressive::AutoregressiveState autoregressiveState;

    const int horizon
        = std::chrono::duration_cast<std::chrono::seconds>(pastProjectedBaseHorizon).count();
    const std::size_t lengthOfPresentPlusPastTrajectory = 1 + mocapFrameRate * horizon;
    autoregressiveState.pastProjectedBaseVelocity
        = std::deque<Eigen::Vector3d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector3d{0.0, 0.0, 0.0}};
    autoregressiveState.pastProjectedBaseAngVelocity
        = std::deque<Eigen::Vector3d>{lengthOfPresentPlusPastTrajectory, Eigen::Vector3d{0.0, 0.0, 0.0}};
    autoregressiveState.I_H_B_prev = manif::SE3d::Identity();

    autoregressiveState.previousVelMannInput = input;
    autoregressiveState.previousVelMannOutput = output;

    autoregressiveState.I_H_B = I_H_B;
    autoregressiveState.leftFootState = leftFootState;
    autoregressiveState.rightFootState = rightFootState;
    autoregressiveState.projectedContactPositionInWorldFrame.setZero();

    autoregressiveState.time = std::chrono::nanoseconds::zero();

    // for rotational PID
    autoregressiveState.I_H_ref = I_H_B;

    return autoregressiveState;
}

struct velMANNAutoregressive::Impl
{
    velMANN velMann;
    velMANNInput velMannInput;
    int projectedBaseDatapoints;
    iDynTree::KinDynComputations kinDyn;
    velMANNAutoregressiveOutput output;

    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};
    std::chrono::nanoseconds dT;
    std::chrono::nanoseconds pastProjectedBaseHorizon;
    int mocapFrameRate;

    std::shared_ptr<ContinuousDynamicalSystem::SO3Dynamics> baseOrientationDynamics;
    ContinuousDynamicalSystem::ForwardEuler<ContinuousDynamicalSystem::SO3Dynamics> integrator;

    Eigen::Vector3d gravity; // This is required by the kindyncomputations object

    AutoregressiveState state;
    Eigen::MatrixXd supportFootJacobian;

    // For rotational PID
    Eigen::RowVectorXd previousDesiredAngVel = Eigen::VectorXd::Zero(projectedBaseDatapoints);
    Eigen::Vector3d previousOmegaE = Eigen::Vector3d::Zero();

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

    bool updateContact(const double referenceHeight,
                       Math::SchmittTrigger& trigger,
                       Contacts::EstimatedContact& contact);
};

bool velMANNAutoregressive::Impl::updateContact(const double referenceHeight,
                                             Math::SchmittTrigger& trigger,
                                             Contacts::EstimatedContact& contact)
{
    constexpr auto logPrefix = "[velMANNAutoregressive::Impl::updateContact]";
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

velMANNAutoregressive::velMANNAutoregressive()
    : m_pimpl(std::make_unique<Impl>())
{
}

velMANNAutoregressive::~velMANNAutoregressive() = default;

bool velMANNAutoregressive::setRobotModel(const iDynTree::Model& model)
{
    constexpr std::size_t twistSize = 6;

    // resize the jacobian of the support foot that will be used in the class to evaluate the
    // velocity of the rootlink assuming that the stance foot has zero velocity
    m_pimpl->supportFootJacobian.resize(twistSize, twistSize + model.getNrOfDOFs());

    return m_pimpl->kinDyn.loadRobotModel(model);
}

bool velMANNAutoregressive::initialize(
    std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler)
{
    constexpr auto logPrefix = "[velMANNAutoregressive::initialize]";

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
    // information when the internal state is updated. For this reason we avoid allocating the
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

    auto velMANNParamHandler = ptr->getGroup("MANN").lock();
    if (velMANNParamHandler == nullptr)
    {
        log()->error("{} Unable to find the group named 'MANN'.", logPrefix);
        return false;
    }

    // add the joint size in the handler
    auto cloneHandler = velMANNParamHandler->clone();
    cloneHandler->setParameter("number_of_joints", int(m_pimpl->kinDyn.getNrOfDegreesOfFreedom()));
    ok = ok && m_pimpl->velMann.initialize(cloneHandler);

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

    m_pimpl->velMannInput.baseLinearVelocityTrajectory.resize(3, m_pimpl->projectedBaseDatapoints);
    m_pimpl->velMannInput.baseAngularVelocityTrajectory.resize(3, m_pimpl->projectedBaseDatapoints);
    m_pimpl->velMannInput.jointPositions.resize(m_pimpl->kinDyn.getNrOfDegreesOfFreedom());
    m_pimpl->velMannInput.jointVelocities.resize(m_pimpl->kinDyn.getNrOfDegreesOfFreedom());

    return ok;
}

bool velMANNAutoregressive::populateInitialAutoregressiveState(
    Eigen::Ref<const Eigen::VectorXd> jointPositions,
    const manif::SE3d& basePosition,
    velMANNAutoregressive::AutoregressiveState& state)
{
    constexpr auto logPrefix = "[velMANNAutoregressive::populateInitialAutoregressiveState]";

    // set the base velocity to zero since we do not need to evaluate any quantity related to it
    Eigen::Matrix<double, 6, 1> baseVelocity;
    baseVelocity.setZero();

    // generate the input of the network we assume that the robot is stopped and the facing
    // direction is the x axis
    const auto input = velMANNInput::generateDummyVelMANNInput(jointPositions, //
                                                         m_pimpl->projectedBaseDatapoints);
    const auto output = velMANNOutput::generateDummyVelMANNOutput(jointPositions, //
                                                            (m_pimpl->projectedBaseDatapoints / 2) + 1);

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
    state = velMANNAutoregressive::AutoregressiveState::generateDummyAutoregressiveState(
        input,
        output,
        basePosition,
        velMANNFootState::generateFootState(m_pimpl->kinDyn,
                                         m_pimpl->state.leftFootState.corners,
                                         "left_foot",
                                         m_pimpl->state.leftFootState.contact.index),
        velMANNFootState::generateFootState(m_pimpl->kinDyn,
                                         m_pimpl->state.rightFootState.corners,
                                         "right_foot",
                                         m_pimpl->state.rightFootState.contact.index),
        m_pimpl->mocapFrameRate,
        m_pimpl->pastProjectedBaseHorizon);

    return true;
}

bool velMANNAutoregressive::reset(Eigen::Ref<const Eigen::VectorXd> jointPositions,
                               const manif::SE3d& basePose)
{
    AutoregressiveState autoregressiveState;
    if (!this->populateInitialAutoregressiveState(jointPositions, basePose, autoregressiveState))
    {
        log()->error("[velMANNAutoregressive::reset] Unable to populate the initial autoregressive "
                     "state.");
        return false;
    }
    return this->reset(autoregressiveState);
}

bool velMANNAutoregressive::reset(const AutoregressiveState& state)
{
    constexpr auto logPrefix = "[velMANNAutoregressive::reset]";

    if (m_pimpl->fsmState != Impl::FSM::Initialized && m_pimpl->fsmState != Impl::FSM::Running
        && m_pimpl->fsmState != Impl::FSM::Reset)
    {
        log()->error("{} velMANNAutoregressive has not been initialized.", logPrefix);
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

bool velMANNAutoregressive::setInput(const Input& input)
{
    // If the output of the network is valid, then the network has been called at least once
    constexpr auto logPrefix = "[velMANNAutoregressive::setInput]";

    if (m_pimpl->fsmState != Impl::FSM::Reset && !this->isOutputValid())
    {
        log()->error("{} The network has not been reset. Please call reset().", logPrefix);
        return false;
    }

    // get the output of the network computed at the previous iteration
    const velMANNOutput& previousVelMannOutput = m_pimpl->state.previousVelMannOutput;

    // the joint positions and velocities are considered as new input
    m_pimpl->velMannInput.jointPositions = previousVelMannOutput.jointPositions;
    m_pimpl->velMannInput.jointVelocities = previousVelMannOutput.jointVelocities;

    // we set the base velocity to zero since we do not need to evaluate any quantity related to it
    const Eigen::Matrix<double, 6, 1> baseVelocity = Eigen::Matrix<double, 6, 1>::Zero();
    if (!m_pimpl->kinDyn.setRobotState(m_pimpl->state.I_H_B.transform(),
                                       m_pimpl->velMannInput.jointPositions,
                                       baseVelocity,
                                       m_pimpl->velMannInput.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to set the robot state in the kinDyn object.", logPrefix);
        return false;
    }

    // TODO here we assume that the x direction of the base frame points forward. This may
    // cause issues on some robots (e.g., iCubV2.5)

    // we compute the transformation between the current and the previous base direction
    const manif::SE3d current_B_H_previous_B = m_pimpl->state.I_H_B.inverse().compose(m_pimpl->state.I_H_B_prev);
    const manif::SO3d current_B_R_previous_B = manif::SO3d(current_B_H_previous_B.quat());

    // this lambda function takes the past trajectory. then it shifts it and append the new
    // datapoint.
    auto updatePreviousInputWithTransform = [](std::deque<Eigen::Vector3d>& pastTrajectory,
                                               const auto& transformation,
                                               const Eigen::Vector3d& newDataPoint) {
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

    auto performSubsampling = [](const std::deque<Eigen::Vector3d>& pastTrajectory,
                                 Eigen::Ref<Eigen::Matrix3Xd> velMannInput) -> void {
        const int sampleDelta = pastTrajectory.size() / (velMannInput.cols() - 1);

        for (int i = 0; i < velMannInput.cols(); i++)
        {
            velMannInput.col(i) = pastTrajectory[i * sampleDelta];
        }
    };

    // This part updates the state.
    updatePreviousInputWithTransform(m_pimpl->state.pastProjectedBaseVelocity,
                                     current_B_R_previous_B,
                                     previousVelMannOutput.futureBaseLinearVelocityTrajectory.col(0));
    updatePreviousInputWithTransform(m_pimpl->state.pastProjectedBaseAngVelocity,
                                     current_B_R_previous_B,
                                     previousVelMannOutput.futureBaseAngularVelocityTrajectory.col(0));

    // we subsample the past trajectory to get the input of the network
    const int halfProjectedBasedHorizon = m_pimpl->projectedBaseDatapoints / 2;
    performSubsampling(m_pimpl->state.pastProjectedBaseVelocity,
                       m_pimpl->velMannInput.baseLinearVelocityTrajectory.leftCols(
                           halfProjectedBasedHorizon));
    performSubsampling(m_pimpl->state.pastProjectedBaseAngVelocity,
                       m_pimpl->velMannInput.baseAngularVelocityTrajectory.leftCols(
                           halfProjectedBasedHorizon));

    m_pimpl->velMannInput.baseLinearVelocityTrajectory.rightCols(halfProjectedBasedHorizon) << input.desiredFutureBaseVelocities.rightCols(halfProjectedBasedHorizon), previousVelMannOutput.futureBaseLinearVelocityTrajectory.bottomRows(1).rightCols(halfProjectedBasedHorizon);

    // If the user input changed between the previous timestep and now, update the reference frame
    const double newInputThresh = 1e-5;
    m_pimpl->previousDesiredAngVel.resize(input.desiredFutureBaseAngVelocities.cols());

    if ((m_pimpl->previousDesiredAngVel - input.desiredFutureBaseAngVelocities).norm() >= newInputThresh)
    {
        m_pimpl->state.I_H_ref = m_pimpl->state.I_H_B;
    }

    const double refYaw = (iDynTree::Rotation(m_pimpl->state.I_H_ref.quat().toRotationMatrix()).asRPY())(2);

    Eigen::Matrix3Xd desiredFutureBaseDirections3d = (Eigen::Matrix3Xd(input.desiredFutureBaseDirections.rows() + 1, input.desiredFutureBaseDirections.cols()) << input.desiredFutureBaseDirections, Eigen::RowVectorXd::Zero(input.desiredFutureBaseDirections.cols())).finished();
    Eigen::Vector3d forwardDir(1, 0, 0);
    const double desYaw =
        std::atan2((forwardDir.cross(desiredFutureBaseDirections3d.col(0)))(2),
        forwardDir.dot(desiredFutureBaseDirections3d.col(0)));

    // Construct desired angle term of rotational PID equation
    manif::SE3d R_d = manif::SE3d(0, 0, 0, //xyz translation is unimportant
                      0, -0.09, desYaw + refYaw);

    Eigen::Matrix3d R_mult = (R_d.inverse().compose(m_pimpl->state.I_H_B).quat().toRotationMatrix());

    Eigen::Matrix3d Sk = ((R_mult - R_mult.inverse())/2);
    Eigen::Vector3d Skv(Sk(2,1), Sk(0,2), Sk(1,0));

    // Apply rotational PID
    const double c0 = 2.0;
    Eigen::Matrix3Xd omega_E(3, input.desiredFutureBaseDirections.cols());
    for (int i = 0; i < input.desiredFutureBaseDirections.cols(); i++)
    {
        omega_E.col(i) = previousVelMannOutput.futureBaseAngularVelocityTrajectory.col(i) - c0 * Skv;
    }

    // assign the rotational PID angular velocity output to be the future portion of the next MANN input
    m_pimpl->velMannInput.baseAngularVelocityTrajectory.rightCols(halfProjectedBasedHorizon) = omega_E.rightCols(halfProjectedBasedHorizon);

    // Save PID controller output for use in base orientation update
    m_pimpl->previousOmegaE = omega_E.col(0);
    m_pimpl->previousDesiredAngVel = input.desiredFutureBaseAngVelocities;

    if (!m_pimpl->velMann.setInput(m_pimpl->velMannInput))
    {
        log()->error("{} Unable to set the input to velMANN network", logPrefix);
        return false;
    }

    // to check if the robot is stopped we need to compare the current input with the previous one.
    // Please notice that the previous input is now contained in m_pimpl->state.velMannInput that will
    // be updated at the end of this function.
    constexpr double toleranceRobotStopped = 0.05;
    m_pimpl->isRobotStopped
        = m_pimpl->velMannInput.baseLinearVelocityTrajectory
                 .isApprox(m_pimpl->state.previousVelMannInput.baseLinearVelocityTrajectory,
                           toleranceRobotStopped)
          && m_pimpl->velMannInput.baseAngularVelocityTrajectory
                 .isApprox(m_pimpl->state.previousVelMannInput.baseAngularVelocityTrajectory,
                           toleranceRobotStopped)
          && m_pimpl->velMannInput.jointVelocities
                 .isApprox(m_pimpl->state.previousVelMannInput.jointVelocities, //
                           toleranceRobotStopped)
          && m_pimpl->velMannInput.jointPositions
                 .isApprox(m_pimpl->state.previousVelMannInput.jointPositions, //
                           toleranceRobotStopped);

    // store the autoregressive state
    m_pimpl->state.I_H_B_prev = m_pimpl->state.I_H_B;
    m_pimpl->state.previousVelMannInput = m_pimpl->velMannInput;

    // the output is not valid anymore since we set a new input
    m_pimpl->isOutputValid = false;

    return true;
}

bool velMANNAutoregressive::advance()
{
    using namespace std::chrono_literals;
    using namespace BipedalLocomotion::GenericContainer::literals;

    constexpr auto logPrefix = "[velMANNAutoregressive::advance]";

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

    // perform one iteration of velMANN
    if (!m_pimpl->velMann.advance() || !m_pimpl->velMann.isOutputValid())
    {
        log()->error("{} Unable to compute the output of the network.", logPrefix);
        return false;
    }

    // get the output of the network
    const auto& velMannOutput = m_pimpl->velMann.getOutput();

    // Integrate the base orientation
    // if the robot is stopped (i.e, if the current velMANN input and the previous one are the same)
    // we set the yaw rate equal to zero
    const manif::SO3Tangentd baseAngularVelocity = m_pimpl->isRobotStopped ? Eigen::Vector3d{0, 0, 0} : manif::SO3d(m_pimpl->state.I_H_B.quat()).act(m_pimpl->previousOmegaE);
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
                                       velMannOutput.jointPositions,
                                       baseVelocity.coeffs(),
                                       velMannOutput.jointVelocities,
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
            * velMannOutput.jointVelocities);

    // update the kindyn object with the kinematically feasible base
    if (!m_pimpl->kinDyn.setRobotState(I_H_base.transform(),
                                       velMannOutput.jointPositions,
                                       baseVelocity.coeffs(),
                                       velMannOutput.jointVelocities,
                                       m_pimpl->gravity))
    {
        log()->error("{} Unable to set the robot state in the kindyncomputations object.",
                     logPrefix);
        return false;
    }

    // From here we store the output of velMANNAutoregressive
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
    m_pimpl->state.previousVelMannOutput = velMannOutput;
    m_pimpl->state.supportFoot = currentSupportFoot;

    // populate the output
    m_pimpl->output.jointsPosition = m_pimpl->state.previousVelMannOutput.jointPositions;
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

bool velMANNAutoregressive::isOutputValid() const
{
    // the output must be valid and the state machine must be in running phase
    return m_pimpl->isOutputValid && m_pimpl->fsmState == Impl::FSM::Running;
}

const velMANNAutoregressive::Output& velMANNAutoregressive::getOutput() const
{
    return m_pimpl->output;
}

const velMANNInput& velMANNAutoregressive::getVelMANNInput() const
{
    return m_pimpl->velMannInput;
}

const velMANNAutoregressive::AutoregressiveState& velMANNAutoregressive::getAutoregressiveState() const
{
    return m_pimpl->state;
}
