/**
 * @file YarpRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <cmath>
#include <future>
#include <thread>
#include <unordered_map>

#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlLimits.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/ICurrentControl.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPWMControl.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::RobotInterface;

struct YarpRobotControl::Impl
{
    /**
     * JointsControlValuesAndMode contains the information regarding the desired joint values and
     * the control mode for each joint.
     */
    struct JointsControlValuesAndMode
    {
        /** Set containing the indices related to each control mode*/
        std::unordered_map<IRobotControl::ControlMode, std::vector<int>> index;

        /** Set containing the desired joint values related to each control mode. In case of
         * Position, PositionDirect and Velocity the joint values are expressed in deg or deg/s */
        std::unordered_map<IRobotControl::ControlMode, Eigen::VectorXd> value;
    };

    std::shared_ptr<yarp::dev::PolyDriver> robotDevice; /**< PolyDriver */

    // YARP Interfaces exposed by the RemoteControlBoardRemapper
    yarp::dev::IEncodersTimed* encodersInterface{nullptr}; /**< Encorders interface. */
    yarp::dev::IPositionDirect* positionDirectInterface{nullptr}; /**< Direct position control
                                                                     interface. */
    yarp::dev::IPositionControl* positionInterface{nullptr}; /**< Position control interface. */
    yarp::dev::IVelocityControl* velocityInterface{nullptr}; /**< Velocity control interface. */
    yarp::dev::ITorqueControl* torqueInterface{nullptr}; /**< Torque control interface. */
    yarp::dev::IPWMControl* pwmInterface{nullptr}; /**< PWM control interface. */
    yarp::dev::ICurrentControl* currentInterface{nullptr}; /**< Current control interface. */
    yarp::dev::IControlMode* controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IAxisInfo* axisInfoInterface{nullptr}; /**< Axis info interface. */
    yarp::dev::IControlLimits* controlLimitsInterface{nullptr}; /**< Control limits interface. */

    std::size_t actuatedDOFs; /**< Number of the actuated DoFs. */

    Eigen::VectorXd positionFeedback; /**< Current joint position [rad]. */

    std::vector<IRobotControl::ControlMode> controlModes; /**< Vector containing the map between the
                                                             joint and the current control mode */
    std::vector<yarp::conf::vocab32_t> controlModesYarp; /**< Vector containing the map between the
                                                             joint and the current yarp control mode
                                                          */

    std::vector<std::string> axesName; /**< List containing the mapping between the joints index and
                                          the their name */

    JointsControlValuesAndMode desiredJointValuesAndMode; /**< Struct containing the information
                                                             regarding the desired joint value and
                                                             the control mode */
    std::vector<double> positionControlRefSpeeds; /**< Vector containing the ref speed in
                                                     deg/seconds for the position control joints. */

    std::chrono::nanoseconds positioningDuration{0}; /**< Duration of the trajectory generated when
                                                        the joint is controlled in position mode */
    std::chrono::nanoseconds startPositionControlInstant{0}; /**< Initial time instant of the
                                                                  trajectory generated when the
                                                                  joint is controlled in position
                                                                  mode */
    double positioningTolerance{0.0}; /**< Max Admissible error for position control joint [rad] */
    double positionDirectMaxAdmissibleError{0.0}; /**< Max admissible error for position direct
                                                     control joint [rad] */

    std::size_t maxReadingAttempts{100}; /**< Max number of attempts used for reading from the yarp
                                            interfaces. */
    std::size_t readingTimeout{500}; /**< Timeout used while reading from the yarp interfaces in
                                        microseconds. */

    static IRobotControl::ControlMode
    YarpControlModeToControlMode(const yarp::conf::vocab32_t& controlModeYarp)
    {
        switch (controlModeYarp)
        {
        case VOCAB_CM_POSITION:
            return IRobotControl::ControlMode::Position;

        case VOCAB_CM_POSITION_DIRECT:
            return IRobotControl::ControlMode::PositionDirect;

        case VOCAB_CM_VELOCITY:
            return IRobotControl::ControlMode::Velocity;

        case VOCAB_CM_TORQUE:
            return IRobotControl::ControlMode::Torque;

        case VOCAB_CM_PWM:
            return IRobotControl::ControlMode::PWM;

        case VOCAB_CM_CURRENT:
            return IRobotControl::ControlMode::Current;

        case VOCAB_CM_IDLE:
            return IRobotControl::ControlMode::Idle;

        default:
            return IRobotControl::ControlMode::Unknown;
        }
    }

    static yarp::conf::vocab32_t
    ControlModeToYarpControlMode(const IRobotControl::ControlMode& controlMode)
    {
        switch (controlMode)
        {
        case IRobotControl::ControlMode::Position:
            return VOCAB_CM_POSITION;

        case IRobotControl::ControlMode::PositionDirect:
            return VOCAB_CM_POSITION_DIRECT;

        case IRobotControl::ControlMode::Velocity:
            return VOCAB_CM_VELOCITY;

        case IRobotControl::ControlMode::Torque:
            return VOCAB_CM_TORQUE;

        case IRobotControl::ControlMode::PWM:
            return VOCAB_CM_PWM;

        case IRobotControl::ControlMode::Current:
            return VOCAB_CM_CURRENT;

        case IRobotControl::ControlMode::Idle:
            return VOCAB_CM_IDLE;

        default:
            return VOCAB_CM_UNKNOWN;
        }
    }

    bool getControlModes()
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::getControlModes]";

        if (this->controlModeInterface == nullptr)
        {
            log()->error("{} The control mode I/F is not ready.", errorPrefix);
            return false;
        }

        // try to read the control mode
        std::size_t counter = 0;
        while (!this->controlModeInterface->getControlModes(this->controlModesYarp.data()))
        {
            if (++counter == this->maxReadingAttempts)
            {
                log()->error("{} Error while reading the control mode.", errorPrefix);
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::microseconds(this->readingTimeout));
        }

        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            this->controlModes[i] = YarpControlModeToControlMode(this->controlModesYarp[i]);
        }

        return true;
    }

    bool setControlModes(const std::vector<IRobotControl::ControlMode>& controlModes)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setControlModes]";

        if (this->controlModeInterface == nullptr)
        {
            log()->error("{} The control mode I/F is not ready.", errorPrefix);
            return false;
        }

        // clear all the stored control modes
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Position].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::PositionDirect].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Velocity].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Torque].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::PWM].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Current].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Idle].clear();

        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            // convert the control mode into Yarp control mode
            this->controlModesYarp[i] = ControlModeToYarpControlMode(controlModes[i]);

            // store the joint associated to a specific control mode
            this->desiredJointValuesAndMode.index[controlModes[i]].push_back(i);
        }

        // set the control mode
        if (!this->controlModeInterface->setControlModes(this->controlModesYarp.data()))
        {
            log()->error("{} Error settings the control mode.", errorPrefix);
            return false;
        }

        // resize the desired joint value vector associated to each control mode
        for (const auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            this->desiredJointValuesAndMode.value[mode].resize(indices.size());
        }

        // resize the position control reference speed vector
        this->positionControlRefSpeeds.resize(
            this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Position].size());

        return true;
    }

    bool getJointPos()
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::getJointPos]";

        if (this->encodersInterface == nullptr)
        {
            log()->error("{} The control mode I/F is not ready.", errorPrefix);
            return false;
        }

        // try to read the joint encoders
        std::size_t counter = 0;
        while (!this->encodersInterface->getEncoders(this->positionFeedback.data()))
        {
            if (++counter == this->maxReadingAttempts)
            {
                log()->error("{} Error while reading the encoders.", errorPrefix);
                return false;
            }

            // Sleep for some while
            std::this_thread::sleep_for(std::chrono::microseconds(this->readingTimeout));
        }

        // convert the joint position in radians
        this->positionFeedback *= M_PI / 180.0;

        return true;
    }

    bool setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setDriver]";

        if (robotDevice == nullptr)
        {
            log()->error("{} The robotDevice is pointing to an non initialized memory.",
                         errorPrefix);
            return false;
        }

        // obtain the interfaces
        if (!robotDevice->view(encodersInterface) || encodersInterface == nullptr)
        {
            log()->error("{} Cannot load the IEncodersTimed interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(positionInterface) || positionInterface == nullptr)
        {
            log()->error("{} Cannot load the IPositionControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(positionDirectInterface) || positionDirectInterface == nullptr)
        {
            log()->error("{} Cannot load the IPositionDirect interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(velocityInterface) || velocityInterface == nullptr)
        {
            log()->error("{} Cannot load the IVelocityControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(torqueInterface) || torqueInterface == nullptr)
        {
            log()->error("{} Cannot load the ITorqueControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(pwmInterface) || pwmInterface == nullptr)
        {
            log()->error("{} Cannot load the IPWMControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(currentInterface) || currentInterface == nullptr)
        {
            log()->error("{} Cannot load the ICurrentControl interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(controlModeInterface) || controlModeInterface == nullptr)
        {
            log()->error("{} Cannot load the IControlMode interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(axisInfoInterface) || axisInfoInterface == nullptr)
        {
            log()->error("{} Cannot load the IAxisInfo interface.", errorPrefix);
            return false;
        }

        if (!robotDevice->view(controlLimitsInterface) || controlLimitsInterface == nullptr)
        {
            log()->error("{} Cannot load the IControlMode interface.", errorPrefix);
            return false;
        }

        // get the number of degree of freedom
        int dofs = 0;
        if (!encodersInterface->getAxes(&dofs))
        {
            log()->error("{} Cannot get the actuated DoFs.", errorPrefix);
            return false;
        }
        this->actuatedDOFs = dofs;

        // resize vector
        this->positionFeedback.resize(this->actuatedDOFs);
        this->controlModes.resize(this->actuatedDOFs);
        this->controlModesYarp.resize(this->actuatedDOFs);
        this->axesName.resize(this->actuatedDOFs);

        // populate the axesName vector
        for (int i = 0; i < this->actuatedDOFs; i++)
        {
            this->axisInfoInterface->getAxisName(i, this->axesName[i]);
        }

        // store the polydriver
        this->robotDevice = robotDevice;

        if (!this->getControlModes())
        {
            log()->error("{} Unable to get the control modes.", errorPrefix);
            return false;
        }

        // clear all the stored control modes
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Position].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::PositionDirect].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Velocity].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Torque].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::PWM].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Current].clear();

        // store the joint associated to a specific control mode
        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            this->desiredJointValuesAndMode.index[this->controlModes[i]].push_back(i);
        }

        // resize the desired joint value vector associated to each control mode
        for (const auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            this->desiredJointValuesAndMode.value[mode].resize(indices.size());
        }

        // resize the reference speed for the position control mode
        // The size of the vector is equal to the size of the joints in position control
        this->positionControlRefSpeeds.resize(
            this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Position].size());

        return true;
    }

    /**
     * Return the the worst position error for the joint controlled in position direct.
     * The first value is the index while the second is the error in radians.
     */
    std::pair<int, double>
    getWorstPositionDirectError(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                                Eigen::Ref<const Eigen::VectorXd> jointPositions) const
    {
        // clear the std::pair
        std::pair<int, double> worstError{0, 0.0};

        for (int i = 0; i < this->actuatedDOFs; i++)
        {
            // we are interested only if the joint is in PositionDirect
            if (this->controlModes[i] == IRobotControl::ControlMode::PositionDirect)
            {
                const double jointError = std::abs(jointPositions[i] - desiredJointValues[i]);

                if (jointError > worstError.second)
                {
                    worstError.first = i;
                    worstError.second = jointError;
                }
            }
        }

        return worstError;
    }

    std::function<bool(const int, const int*, const double*)>
    control(const IRobotControl::ControlMode& mode)
    {
        assert(mode != IRobotControl::ControlMode::Unknown);
        assert(mode != IRobotControl::ControlMode::Idle);

        switch (mode)
        {
        case IRobotControl::ControlMode::Position:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->positionInterface->positionMove(nJoints, joints, refs);
            };

        case IRobotControl::ControlMode::PositionDirect:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->positionDirectInterface->setPositions(nJoints, joints, refs);
            };

        case IRobotControl::ControlMode::Velocity:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->velocityInterface->velocityMove(nJoints, joints, refs);
            };

        case IRobotControl::ControlMode::Torque:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->torqueInterface->setRefTorques(nJoints, joints, refs);
            };

        case IRobotControl::ControlMode::Current:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                return this->currentInterface->setRefCurrents(nJoints, joints, refs);
            };

        case IRobotControl::ControlMode::PWM:
            return [&](const int nJoints, const int* joints, const double* refs) -> bool {
                bool ok = true;
                for (int i = 0; i < nJoints; i++)
                {
                    ok = ok && this->pwmInterface->setRefDutyCycle(joints[i], refs[i]);
                }
                return ok;
            };

        default:
            return nullptr;
        }

        return nullptr;
    }

    bool setReferences(Eigen::Ref<const Eigen::VectorXd> jointValues,
                       std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
    {
        constexpr auto errorPrefix = "[YarpRobotControl::Impl::setReferences]";

        // the following checks are performed only if the robot is controlled in position direct or
        // in position mode
        if (!this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Position].empty()
            || !this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::PositionDirect]
                    .empty())
        {

            if (jointValues.size() != this->actuatedDOFs)
            {
                log()->error("{} The size of the joint values is different from the number of "
                             "actuated DoFs. Expected size: {}. Received size: {}.",
                             errorPrefix,
                             this->actuatedDOFs,
                             jointValues.size());
                return false;
            }

            if (currentJointValues.has_value())
            {
                if (currentJointValues->size() != this->actuatedDOFs)
                {
                    log()->error("{} The size of the current joint values is different from the "
                                 "number of actuated DoFs. Expected size: {}. Received size: {}.",
                                 errorPrefix,
                                 this->actuatedDOFs,
                                 currentJointValues->size());
                    return false;
                }
                this->positionFeedback = currentJointValues.value();
            } else
            {
                if (!this->getJointPos())
                {
                    log()->error("{} Unable to get the joint position.", errorPrefix);
                    return false;
                }
            }

            const auto worstError = this->getWorstPositionDirectError(jointValues, //
                                                                      this->positionFeedback);

            if (worstError.second > this->positionDirectMaxAdmissibleError)
            {
                log()->error("{} The worst error between the current and the desired position of "
                             "the "
                             "joint named '{}' is greater than {} deg. Error = {} deg.",
                             errorPrefix,
                             this->axesName[worstError.first],
                             180 / M_PI * this->positionDirectMaxAdmissibleError,
                             180 / M_PI * worstError.second);
                return false;
            }
        }

        for (const auto& [mode, indices] : this->desiredJointValuesAndMode.index)
        {
            // if indices vector is empty no joint is controlled with this specific control mode
            if (indices.empty())
            {
                continue;
            }

            if (mode == IRobotControl::ControlMode::Idle)
            {
                continue;
            }

            else if (mode == IRobotControl::ControlMode::Unknown)
            {
                std::string joints = "";
                for (const auto& index : indices)
                {
                    joints += " '" + this->axesName[index] + "'";
                }

                log()->error("{} The following joints does not have a specified control "
                             "mode:{}. Please set a feasible control mode.",
                             errorPrefix,
                             joints);
                return false;

            } else if (mode == IRobotControl::ControlMode::Position)
            {
                const double positioningDurationSeconds
                    = std::chrono::duration<double>(this->positioningDuration).count();
                for (int i = 0; i < indices.size(); i++)
                {
                    const auto jointError
                        = std::abs(jointValues[indices[i]] - this->positionFeedback[indices[i]]);

                    constexpr double scaling = 180 / M_PI;
                    constexpr double minVelocityInDegPerSeconds = 3.0;
                    this->positionControlRefSpeeds[i]
                        = std::max(minVelocityInDegPerSeconds,
                                   scaling * (jointError / positioningDurationSeconds));
                }

                if (!this->positionInterface->setRefSpeeds(indices.size(),
                                                           indices.data(),
                                                           this->positionControlRefSpeeds.data()))
                {
                    log()->error("{} Unable to set the reference speed for the position control "
                                 "joints.",
                                 errorPrefix);
                    return false;
                }

                this->startPositionControlInstant = BipedalLocomotion::clock().now();
            }

            // Yarp wants the quantities in degrees
            double scaling = 180 / M_PI;
            // if the control mode is torque or PWM current it is not required to change the unit of
            // measurement
            if (mode == ControlMode::Torque //
                || mode == ControlMode::PWM //
                || mode == ControlMode::Current)
            {
                scaling = 1;
            }

            for (int i = 0; i < indices.size(); i++)
            {
                this->desiredJointValuesAndMode.value[mode][i] = scaling * jointValues[indices[i]];
            }

            if (!this->control(mode)(indices.size(),
                                     indices.data(),
                                     this->desiredJointValuesAndMode.value[mode].data()))

            {
                log()->error("{} Unable to set the desired joint values.", errorPrefix);
                return false;
            }
        }
        return true;
    }

    bool checkControlMode(const std::vector<IRobotControl::ControlMode>& controlModes) const
    {
        return controlModes == this->controlModes;
    }

    bool checkControlMode(const IRobotControl::ControlMode& mode) const
    {
        return std::all_of(this->controlModes.begin(),
                           this->controlModes.end(),
                           [&mode](const auto& m) { return m == mode; });
    }
};

YarpRobotControl::YarpRobotControl()
    : m_pimpl(std::make_unique<Impl>())
{
}

YarpRobotControl::~YarpRobotControl() = default;

bool YarpRobotControl::setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice)
{
    return m_pimpl->setDriver(robotDevice);
}

bool YarpRobotControl::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[YarpRobotControl::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} The handler is not pointing to an already initialized memory.",
                     errorPrefix);
        return false;
    }

    // optional parameters
    int temp = 0;
    if (ptr->getParameter("reading_timeout", temp))
    {
        // the reading timeout has to be a positive number
        if (temp < 0)
        {
            log()->error("{} 'reading_timeout' parameter has to be a positive number.",
                         errorPrefix);
            return false;
        }
        m_pimpl->readingTimeout = temp;
    }

    if (ptr->getParameter("max_reading_attempts", temp))
    {
        // the max_reading_attempts has to be a strictly positive number
        if (temp <= 0)
        {
            log()->error("{} 'max_reading_attempts' parameter has to be a strictly positive "
                         "number.",
                         errorPrefix);

            return false;
        }
        m_pimpl->maxReadingAttempts = temp;
    }

    // mandatory parameters
    using namespace std::chrono_literals;
    bool ok = ptr->getParameter("positioning_duration", m_pimpl->positioningDuration)
              && (m_pimpl->positioningDuration > 0s);
    ok = ok && ptr->getParameter("positioning_tolerance", m_pimpl->positioningTolerance)
         && (m_pimpl->positioningTolerance > 0);
    ok = ok
         && ptr->getParameter("position_direct_max_admissible_error",
                              m_pimpl->positionDirectMaxAdmissibleError)
         && (m_pimpl->positionDirectMaxAdmissibleError > 0);

    return ok;
}

bool YarpRobotControl::setControlMode(const std::vector<IRobotControl::ControlMode>& controlModes)
{
    if (!m_pimpl->setControlModes(controlModes))
    {
        log()->error("[YarpRobotControl::setControlMode] Unable to set the control modes.");
        return false;
    }

    // if the control mode is set for all the joints, we can store it
    m_pimpl->controlModes = controlModes;

    return true;
}

bool YarpRobotControl::setControlMode(const IRobotControl::ControlMode& mode)
{
    // create a vector containing the same control mode for all the joints
    const std::vector<IRobotControl::ControlMode> controlModes(m_pimpl->actuatedDOFs, mode);
    return this->setControlMode(controlModes);
}

std::future<bool>
YarpRobotControl::setControlModeAsync(const std::vector<IRobotControl::ControlMode>& controlModes)
{
    // lambda function to set the control mode
    auto setControlMode
        = [this, controlModes]() -> bool { return this->setControlMode(controlModes); };

    return std::async(std::launch::async, setControlMode);
}

std::future<bool> YarpRobotControl::setControlModeAsync(const IRobotControl::ControlMode& mode)
{
    // lambda function to set the control mode
    auto setControlMode = [this, mode]() -> bool { return this->setControlMode(mode); };

    return std::async(std::launch::async, setControlMode);
}

bool YarpRobotControl::setReferences(
    Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
    const std::vector<IRobotControl::ControlMode>& controlModes,
    std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
{
    if (!m_pimpl->checkControlMode(controlModes))
    {
        log()->error("[YarpRobotControl::setReferences] Control modes are not the expected one. "
                     "Please call setControlMode before calling this function.");
        return false;
    }

    return m_pimpl->setReferences(desiredJointValues, currentJointValues);
}

bool YarpRobotControl::setReferences(
    Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
    const IRobotControl::ControlMode& mode,
    std::optional<Eigen::Ref<const Eigen::VectorXd>> currentJointValues)
{
    if (!m_pimpl->checkControlMode(mode))
    {
        log()->error("[YarpRobotControl::setReferences] Control mode is not the expected one. "
                     "Please call setControlMode before calling this function.");
        return false;
    }

    return m_pimpl->setReferences(desiredJointValues, currentJointValues);
}

bool YarpRobotControl::checkMotionDone(bool& motionDone,
                                       bool& isTimeExpired,
                                       std::vector<std::pair<std::string, double>>& info)
{
    constexpr auto errorPrefix = "[YarpRobotControl::checkMotionDone]";

    if (!m_pimpl->positionInterface->checkMotionDone(&motionDone))
    {
        log()->error("{} Unable to check if the motion is terminated from the Yarp interface.",
                     errorPrefix);
        return false;
    }

    if (!m_pimpl->getJointPos())
    {
        log()->error("{} Unable to get the joint position.", errorPrefix);
        return false;
    }

    info.clear();

    const auto& jointPositionControlIndex
        = m_pimpl->desiredJointValuesAndMode.index[ControlMode::Position];

    const auto& jointPositionControlDesiredValue
        = m_pimpl->desiredJointValuesAndMode.value[ControlMode::Position];

    for (int i = 0; i < jointPositionControlIndex.size(); i++)
    {
        const double desiredJointPosRad = jointPositionControlDesiredValue[i] * M_PI / 180;
        const double error = std::abs(desiredJointPosRad
                                      - m_pimpl->positionFeedback[jointPositionControlIndex[i]]);

        if (error > m_pimpl->positioningTolerance)
        {
            info.push_back({m_pimpl->axesName[jointPositionControlIndex[i]], error});
        }
    }

    using namespace std::chrono_literals;
    const std::chrono::nanoseconds now = BipedalLocomotion::clock().now();
    constexpr std::chrono::nanoseconds timeTolerance{1s};
    if (now - m_pimpl->startPositionControlInstant > m_pimpl->positioningDuration + timeTolerance)
    {
        isTimeExpired = true;
    } else
    {
        isTimeExpired = false;
    }
    motionDone = motionDone && info.empty();

    return true;
}

std::vector<std::string> YarpRobotControl::getJointList() const
{
    return m_pimpl->axesName;
}

bool YarpRobotControl::isValid() const
{
    return m_pimpl->robotDevice != nullptr;
}

bool YarpRobotControl::getJointLimits(Eigen::Ref<Eigen::VectorXd> lowerLimits,
                                      Eigen::Ref<Eigen::VectorXd> upperLimits) const
{

    constexpr auto errorPrefix = "[YarpRobotControl::getJointLimits]";

    if (lowerLimits.size() != m_pimpl->actuatedDOFs)
    {
        log()->error("{} The size of the first input vector is not "
                     "correct. Expected size: {}. Received size: {}.",
                     errorPrefix,
                     m_pimpl->actuatedDOFs,
                     lowerLimits.size());
        return false;
    }

    if (upperLimits.size() != m_pimpl->actuatedDOFs)
    {
        log()->error("{} The size of the second input vector is not "
                     "correct. Expected size: {}. Received size: {}.",
                     errorPrefix,
                     m_pimpl->actuatedDOFs,
                     upperLimits.size());
        return false;
    }

    for (int i = 0; i < m_pimpl->actuatedDOFs; i++)
    {
        m_pimpl->controlLimitsInterface->getLimits(i, &lowerLimits[i], &upperLimits[i]);
    }
    return true;
}
