/**
 * @file YarpRobotControl.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_ROBOT_CONTROL_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_ROBOT_CONTROL_H

// std
#include <memory>

// Eigen
#include <Eigen/Dense>

// YARP
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/RobotInterface/IRobotControl.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * YarpRobotControl Yarp implementation of the IRobotControl interface
 * @warning At the current stage only revolute joints are supported.
 */
class YarpRobotControl : public IRobotControl
{
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:

    /**
     * Constructor
     */
    YarpRobotControl();

    /**
     * Initialize the Interface
     * @param handler pointer to a parameter handler interface
     * @return True/False in case of success/failure.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler) final;

    /**
     * Set the driver required to control the robot.
     * @param robotDevice device used to control the robot.
     * @return True/False in case of success/failure.
     */
    bool setDriver(std::shared_ptr<yarp::dev::PolyDriver> robotDevice);

    /**
     * Check if the motion set through the position control mode ended.
     * @param[out] motionDone true if the motion ended.
     * @param[out] isTimerExpired true if the internal timer is expired or not.
     * @param[out] info vector containing the list of the joint whose motion did not finish yet.
     * @return True/False in case of success/failure.
     * @note If the timer is expired and the motion did not finish yet, there may be a problem
     * with the robot.
     */
    bool checkMotionDone(bool& motionDone,
                         bool& isTimeExpired,
                         std::vector<std::pair<std::string, double>>& info) final;

    /**
     * Set the desired reference.
     * @param jointValues desired joint values.
     * @param controlModes vector containing the control mode for each joint.
     * @return True/False in case of success/failure.
     * @note In case of position control the values has to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm.
     * @warning At the current stage only revolute joints are supported.
     */
    bool setReferences(Eigen::Ref<const Eigen::VectorXd> jointValues,
                       const std::vector<IRobotControl::ControlMode>& controlModes) final;

    /**
     * Set the desired reference.
     * @param jointValues desired joint values.
     * @param controlMode a control mode for all the joints.
     * @return True/False in case of success/failure.
     * @note In case of position control the values has to be expressed in rad, in case of velocity
     * control in rad/s. If the robot is controlled in torques, the desired joint values are
     * expressed in Nm.
     * @warning Call this function if you want to control all the joint with the same control mode.
     * Otherwise call setReferences(Eigen::Ref<const Eigen::VectorXd>, const
     * std::vector<IRobotControl::ControlMode>&).
     */
    bool setReferences(Eigen::Ref<const Eigen::VectorXd> desiredJointValues,
                       const IRobotControl::ControlMode& mode) final;

    /**
     * Destructor
     */
    ~YarpRobotControl();
};
} // namespace ParametersHandler
} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_ROBOT_CONTROL_H
