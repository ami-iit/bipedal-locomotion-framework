/**
 * @file Module.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_UTILITIES_CARTESIAN_TRAJECTORY_PLAYER_MODULE_H
#define BIPEDAL_LOCOMOTION_UTILITIES_CARTESIAN_TRAJECTORY_PLAYER_MODULE_H

// std
#include <deque>
#include <memory>
#include <string>
#include <vector>

// YARP
#include <yarp/os/RFModule.h>

#include <matioCpp/matioCpp.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>

#include <BipedalLocomotion/Planners/SwingFootPlanner.h>

#include <BipedalLocomotion/CartesianTrajectoryPlayer/Homing.h>

#include <thrifts/CartesianTrajectoryPlayerCommands.h>

namespace BipedalLocomotion
{
namespace CartesianTrajectoryPlayer
{

class Module : public yarp::os::RFModule, public CartesianTrajectoryPlayerCommands
{
    /* WalkingControllers::WalkingIK m_ik; */

    double m_dT; /**< RFModule period. */
    std::string m_robot; /**< Robot name. */

    Eigen::VectorXd m_currentJointPos; /**< Current joint positions. */
    Eigen::VectorXd m_currentJointVel; /**< Current joint positions. */
    std::vector<Eigen::Matrix<double, 6, 1>> m_contactForces;


    std::vector<BipedalLocomotion::RobotInterface::PolyDriverDescriptor> m_poly;


    RobotInterface::YarpRobotControl m_robotControl; /**< Robot control object. */
    RobotInterface::YarpSensorBridge m_sensorBridge; /**< Sensor bridge object. */

    Homing m_homing;

    Planners::SwingFootPlanner m_rightFootPlanner;
    Planners::SwingFootPlanner m_leftFootPlanner;


    int m_numOfJoints; /**< Number of joints to control. */

    std::unordered_map<std::string, std::vector<double>> m_log ; /**< Measured joint
                                                                           positions. */


    std::vector<std::string> m_axisList; /**< Axis name list. */

    unsigned int m_idxTraj{0}; /**< Index to iterate the trajectory. */
    std::mutex m_mutex;

    enum class State
    {
        idle,
        positioning,
        ready,
        running
    };
    State m_state{State::idle}; /** State machine */


    yarp::os::Port m_rpcPort;

    bool createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool createContactWrenchInterface(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler);

    bool readStateFromFile(const std::string& filename, const std::size_t numFields);

public:
    /**
     * Get the period of the RFModule.
     * @return the period of the module.
     */
    double getPeriod() override;

    /**
     * Main function of the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool updateModule() override;

    /**
     * Configure the RFModule.
     * @param rf is the reference to a resource finder object
     * @return true in case of success and false otherwise.
     */
    bool configure(yarp::os::ResourceFinder& rf) override;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    bool homing() final;

    bool start() final;
};
} // namespace JointTrajectoryPlayer
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_UTILITIES_JOINT_TRAJECTORY_PLAYER_MODULE_H
