/**
 * @file Module.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_UTILITIES_JOINT_POSITION_TRACKING_MODULE_H
#define BIPEDAL_LOCOMOTION_UTILITIES_JOINT_POSITION_TRACKING_MODULE_H

// std
#include <chrono>
#include <memory>
#include <string>
#include <vector>

// YARP
#include <yarp/os/RFModule.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

namespace BipedalLocomotion
{
namespace JointPositionTracking
{

class Module : public yarp::os::RFModule
{
    std::chrono::nanoseconds m_dT; /**< RFModule period. */
    std::string m_robot; /**< Robot name. */

    Eigen::VectorXd m_currentJointPos;

    BipedalLocomotion::RobotInterface::PolyDriverDescriptor m_controlBoard;

    BipedalLocomotion::RobotInterface::YarpRobotControl m_robotControl;
    BipedalLocomotion::RobotInterface::YarpSensorBridge m_sensorBridge;

    std::vector<double> m_setPoints;
    std::vector<double>::const_iterator m_currentSetPoint;

    BipedalLocomotion::Planners::QuinticSpline m_spline;
    std::vector<std::chrono::nanoseconds> m_timeKnots;
    std::vector<Eigen::VectorXd> m_trajectoryKnots;

    double m_initTrajectoryTime;

    bool generateNewTrajectory();

    bool createPolydriver(
        std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool initializeRobotControl(
        std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    bool instantiateSensorBridge(
        std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    std::vector<double> m_logJointPos;
    std::vector<double> m_logDesiredJointPos;

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
};
} // namespace JointPositionTracking
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_UTILITIES_JOINT_POSITION_TRACKING_MODULE_H
