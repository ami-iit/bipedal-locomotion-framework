/**
 * @file YarpRobotControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>
#include <unordered_map>

#include <yarp/dev/IAxisInfo.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/dev/IEncodersTimed.h>
#include <yarp/dev/IPositionControl.h>
#include <yarp/dev/IPositionDirect.h>
#include <yarp/dev/ITorqueControl.h>
#include <yarp/dev/IVelocityControl.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/os/Time.h>

#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>

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
    yarp::dev::IControlMode* controlModeInterface{nullptr}; /**< Control mode interface. */
    yarp::dev::IAxisInfo* axisInfoInterface{nullptr}; /**< Axis info interface. */

    std::size_t actuatedDOFs; /**< Number of the actuated DoFs. */

    Eigen::VectorXd positionFeedback; /**< Current joint position [rad]. */

    std::vector<IRobotControl::ControlMode> controlModes; /**< Vector containing the map between the
                                                             joint and the current control mode */
    std::vector<yarp::conf::vocab32_t> controlModesYarp; /**< Vector containing the map between the
                                                             joint and the current yarp control mode */

    std::vector<std::string> axesName; /**< List containing the mapping between the joints index and
                                          the their name */

    JointsControlValuesAndMode desiredJointValuesAndMode; /**< Struct containing the information
                                                             regarding the desired joint value and
                                                             the control mode */

    double positioningDuration{0.0}; /**< Duration of the trajectory generated when the joint is
                                        controlled in position mode */
    double startPositionControlInstant{0.0}; /**< Initial time instant of the trajectory generated
                                                when the joint is controlled in position mode */
    double positioningTolerance{0.0}; /**< Max Admissible error for position control joint [rad] */
    double positionDirectMaxAdmissibleError{0.0}; /**< Max admissible error for position direct
                                                     control joint [rad] */

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

        default:
            return VOCAB_CM_UNKNOWN;
        }
    }

    bool getControlModes()
    {
        constexpr std::string_view errorPrefix = "[YarpRobotControl::Impl::getControlModes] ";

        if (this->controlModeInterface == nullptr)
        {
            std::cerr << errorPrefix << "The control mode I/F is not ready." << std::endl;
            return false;
        }

        if (!this->controlModeInterface->getControlModes(this->controlModesYarp.data()))
        {
            std::cerr << errorPrefix << "Error reading the control mode." << std::endl;
            return false;
        }

        for (std::size_t i = 0; i < this->actuatedDOFs; i++)
        {
            this->controlModes[i] = YarpControlModeToControlMode(this->controlModesYarp[i]);
        }

        return true;
    }

    bool setControlModes(const std::vector<IRobotControl::ControlMode>& controlModes)
    {
        constexpr std::string_view errorPrefix = "[YarpRobotControl::Impl::setControlModes] ";

        if (this->controlModeInterface == nullptr)
        {
            std::cerr << errorPrefix << "The control mode I/F is not ready." << std::endl;
            return false;
        }

        // clear all the stored control modes
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Position].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::PositionDirect].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Velocity].clear();
        this->desiredJointValuesAndMode.index[IRobotControl::ControlMode::Torque].clear();

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
            std::cerr << errorPrefix << "Error settings the control mode." << std::endl;
            return false;
        }

        // resize the desired joint value vector associated to each control mode
        for (const auto& [mode, indeces] : this->desiredJointValuesAndMode.index)
        {
            this->desiredJointValuesAndMode.value[mode].resize(indeces.size());
        }

        return true;
    }

    bool getJointPos()
    {
        constexpr std::string_view errorPrefix = "[YarpRobotControl::Impl::getJointPos] ";

        if (this->encodersInterface == nullptr)
        {
            std::cerr << errorPrefix << "The encoder I/F is not ready." << std::endl;
            return false;
        }

        if (!this->encodersInterface->getEncoders(this->positionFeedback.data()))
        {
            std::cerr << errorPrefix << "Error reading encoders." << std::endl;
            return false;
        }

        // convert the joint position in radians
        this->positionFeedback *= M_PI / 180.0;

        return true;
    }

    bool setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice)
    {
        constexpr std::string_view errorPrefix = "[YarpRobotControl::Impl::setDriver] ";

        if (robotDevice == nullptr)
        {
            std::cerr << errorPrefix
                      << "The robotDevice is pointing to an non initialized memory."
                      << std::endl;
            return false;
        }

        // obtain the interfaces
        if (!robotDevice->view(encodersInterface) || encodersInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the IEncodersTimed interface." << std::endl;
            return false;
        }

        if (!robotDevice->view(positionInterface) || positionInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the IPositionControl interface." << std::endl;
            return false;
        }

        if (!robotDevice->view(positionDirectInterface) || positionDirectInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the IPositionDirect interface." << std::endl;
            return false;
        }

        if (!robotDevice->view(velocityInterface) || velocityInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the IVelocityInterface interface."
                      << std::endl;
            return false;
        }

        if (!robotDevice->view(torqueInterface) || torqueInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the ITorqueInterface interface." << std::endl;
            return false;
        }

        if (!robotDevice->view(controlModeInterface) || controlModeInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the IControlMode interface." << std::endl;
            return false;
        }

        if (!robotDevice->view(axisInfoInterface) || axisInfoInterface == nullptr)
        {
            std::cerr << errorPrefix << "Cannot load the IAxisInfo interface." << std::endl;
            return false;
        }

        // get the number of degree of freedom
        int dofs = 0;
        if (!encodersInterface->getAxes(&dofs))
        {
            std::cerr << errorPrefix << "Cannot get the actuated DoFs." << std::endl;
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

        return this->getControlModes();
    }

    /**
     * Return the the worst position error for the joint controlled in position direct.
     * The first value is the index while the second is the error in radians.
     */
    std::pair<int, double> getWorstPositionDirectError(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
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

        default:
            return nullptr;
        }

        return nullptr;
    }

    bool setReferences(Eigen::Ref<const Eigen::VectorXd> jointValues)
    {
        constexpr std::string_view errorPrefix = "[YarpRobotControl::Impl::setReferences] ";

        if(!this->getJointPos())
        {
            std::cerr << errorPrefix << "Unable to get the joint position." << std::endl;
            return  false;
        }

        const auto worstError = this->getWorstPositionDirectError(jointValues,
                                                                  this->positionFeedback);

        if (worstError.second > this->positionDirectMaxAdmissibleError)
        {
            std::cerr << errorPrefix << "The worst error between the current and the "
                      << "desired position of the joint named '" << this->axesName[worstError.first]
                      << "' is greater than " << this->positionDirectMaxAdmissibleError
                      << " rad. Error = " << worstError.second << " rad." << std::endl;
            return false;
        }

        for (const auto& [mode, indeces] : this->desiredJointValuesAndMode.index)
        {
            // if indeces vector is empty no joint is controlled with this specific control mode
            if (indeces.empty())
                continue;

            if (mode == IRobotControl::ControlMode::Unknown)
            {
                std::string error = " The following joints does not have a specified control "
                                    "mode: ";

                for (const auto& index : indeces)
                    error += "'" + this->axesName[index] + "' ";

                std::cerr << errorPrefix << error << ". Please set a feasible control mode."
                          << std::endl;

                return false;

            } else if (mode == IRobotControl::ControlMode::Position)
            {
                std::vector<double> refSpeeds(indeces.size());
                for (int i = 0; i < indeces.size(); i++)
                {
                    const auto jointError = std::abs(jointValues[indeces[i]]
                                                     - this->positionFeedback[indeces[i]]);

                    constexpr double scaling = 180 / M_PI;
                    constexpr double maxVelocityInDegPerSeconds = 3.0;
                    refSpeeds[i] = std::max(maxVelocityInDegPerSeconds,
                                            scaling * (jointError / this->positioningDuration));

                    this->positionInterface->setRefSpeeds(indeces.size(),
                                                          indeces.data(),
                                                          refSpeeds.data());
                }

                this->startPositionControlInstant = yarp::os::Time::now();
            }

            // Yarp wants the quantities in degrees
            double scaling = 180 / M_PI;
            if (mode == ControlMode::Torque)
                scaling = 1;

            for (int i = 0; i < indeces.size(); i++)
                this->desiredJointValuesAndMode.value[mode][i] = scaling * jointValues[indeces[i]];

            if (!this->control(mode)(indeces.size(),
                                     indeces.data(),
                                     this->desiredJointValuesAndMode.value[mode].data()))

            {
                std::cerr << errorPrefix << "Unable to set the desired joint values." << std::endl;
                return false;
            }
        }
        return true;
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
    constexpr std::string_view errorPrefix = "[YarpRobotControl::initialize] ";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "The handler is not pointing to an already initialized memory."
                  << std ::endl;
        return false;
    }

    bool ok = ptr->getParameter("positioning_duration", m_pimpl->positioningDuration);
    ok = ok && ptr->getParameter("positioning_tolerance", m_pimpl->positioningTolerance);
    ok = ok && ptr->getParameter("position_direct_max_admissible_error",
                                 m_pimpl->positionDirectMaxAdmissibleError);

    return ok;
}

bool YarpRobotControl::setReferences(Eigen::Ref<const Eigen::VectorXd> jointValues,
                                       const std::vector<IRobotControl::ControlMode>& controlModes)
{
    if (controlModes != m_pimpl->controlModes)
    {
        m_pimpl->controlModes = controlModes;
        if (!m_pimpl->setControlModes(m_pimpl->controlModes))
        {
            std::cerr << "[YarpRobotControl::setReferences] Unable to switch in "
                         "position-direct control mode."
                      << std::endl;
            return false;
        }
    }

    return m_pimpl->setReferences(jointValues);
}

bool YarpRobotControl::setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                                       const IRobotControl::ControlMode& mode)
{

    // check if all the joints are controlled in the desired control mode
    if (!std::all_of(m_pimpl->controlModes.begin(),
                     m_pimpl->controlModes.end(),
                     [&mode](const auto& m) { return m == mode; }))
    {
        std::fill(m_pimpl->controlModes.begin(), m_pimpl->controlModes.end(), mode);
        if (!m_pimpl->setControlModes(m_pimpl->controlModes))
        {
            std::cerr << "[YarpRobotControl::setReferences] Unable to the desired control mode."
                      << std::endl;
            return false;
        }
    }

    return m_pimpl->setReferences(desiredJointValues);
}

bool YarpRobotControl::checkMotionDone(bool& motionDone,
                                         bool& isTimeExpired,
                                         std::vector<std::pair<std::string, double>>& info)
{
    constexpr std::string_view errorPrefix = "[YarpRobotControl::checkMotionDone] ";

    if (!m_pimpl->positionInterface->checkMotionDone(&motionDone))
    {
        std::cerr << errorPrefix
                  << "Unable to check if the motion is terminated from the Yarp interface."
                  << std::endl;
        return false;
    }

    if (!m_pimpl->getJointPos())
    {
        std::cerr << errorPrefix << "Unable to get the joint position." << std::endl;
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

    const double now = yarp::os::Time::now();
    constexpr double timeTolerance = 1.0;
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
