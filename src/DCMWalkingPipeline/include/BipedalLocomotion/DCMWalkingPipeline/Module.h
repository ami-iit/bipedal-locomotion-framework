/**
 * @file Module.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_DCM_WALKING_MODULE_H
#define BIPEDAL_LOCOMOTION_DCM_WALKING_MODULE_H

#include <mutex>

#include <yarp/os/RFModule.h>
#include <yarp/sig/Vector.h>

#include <BipedalLocomotion/DCMWalkingPipeline/Homing.h>
#include <BipedalLocomotion/DCMWalkingPipeline/Pipeline.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <thrifts/BipedalLocomotion/DCMWalkingPipeline/Commands.h>

namespace BipedalLocomotion
{
namespace DCMWalkingPipeline
{

class Module : public yarp::os::RFModule, public Commands
{

    enum class State
    {
        Idle,
        Configured,
        Prepared,
        Walking
    };

    State m_state{State::Idle};

    double m_dT;

    std::mutex m_mutex;


    BipedalLocomotion::RobotInterface::YarpRobotControl m_robotControl;
    BipedalLocomotion::RobotInterface::YarpSensorBridge m_sensorBridge;

    std::shared_ptr<yarp::dev::PolyDriver> m_robotDevice;

    BipedalLocomotion::DCMWalkingPipeline::Homing m_homing;
    BipedalLocomotion::DCMWalkingPipeline::Pipeline m_pipeline;

    yarp::os::Port m_rpcPort;
    yarp::os::BufferedPort<yarp::sig::Vector> m_robotBasePort;

    manif::SE3d m_baseTransform;
    manif::SE3d::Tangent m_baseVelocity;

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
    bool configure(yarp::os::ResourceFinder& rf) final;

    /**
     * Close the RFModule.
     * @return true in case of success and false otherwise.
     */
    bool close() override;

    /**
     * This allows you to put the robot in a home position for walking.
     * @return true in case of success and false otherwise.
     */
    bool homing() final;

    bool startWalking() final;
};

} // namespace DCMWalkingPipeline
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_DCM_WALKING_MODULE_H
