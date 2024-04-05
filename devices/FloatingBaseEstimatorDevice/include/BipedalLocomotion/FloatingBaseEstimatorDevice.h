/**
 * @file FloatingBaseEstimatorDevice.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_BASE_ESTIMATOR_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_BASE_ESTIMATOR_DEVICE_H

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimator.h>

#include <iDynTree/Estimation/ContactStateMachine.h>
#include <iDynTree/ModelLoader.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IFrameTransform.h>

#include <mutex>
#include <memory>

namespace BipedalLocomotion
{
    class FloatingBaseEstimatorDevice;
}

class BipedalLocomotion::FloatingBaseEstimatorDevice : public yarp::dev::DeviceDriver,
                                                       public yarp::dev::IMultipleWrapper,
                                                       public yarp::os::PeriodicThread
{
public:
    FloatingBaseEstimatorDevice(double period,
                                yarp::os::ShouldUseSystemClock useSystemClock
                                = yarp::os::ShouldUseSystemClock::No);
    FloatingBaseEstimatorDevice();
    ~FloatingBaseEstimatorDevice();

    virtual bool open(yarp::os::Searchable& config) final;
    virtual bool close() final;
    virtual bool attachAll(const yarp::dev::PolyDriverList & poly) final;
    virtual bool detachAll() final;
    virtual void run() final;

private:
    bool setupRobotModel(yarp::os::Searchable& config);
    bool setupRobotSensorBridge(yarp::os::Searchable& config);
    bool setupFeetContactStateMachines(yarp::os::Searchable& config);
    bool parseFootSchmittParams(yarp::os::Searchable& config, iDynTree::SchmittParams& params);
    bool setupBaseEstimator(yarp::os::Searchable& config);
    bool loadTransformBroadcaster();
    bool updateMeasurements();
    bool updateInertialBuffers();
    bool updateContactStates();
    bool updateKinematics();

    void publish();
    void publishBaseLinkState(const BipedalLocomotion::Estimators::FloatingBaseEstimators::Output& out);
    void publishInternalStateAndStdDev(const BipedalLocomotion::Estimators::FloatingBaseEstimators::Output& out);
    void publishFootContactStatesAndNormalForces();
    bool openCommunications();
    void closeCommunications();
    bool openBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port,
                             const std::string& portPrefix,
                             const std::string& address);
    void closeBufferedSigPort(yarp::os::BufferedPort<yarp::sig::Vector>& port);

    struct
    {
        yarp::os::BufferedPort<yarp::sig::Vector> floatingBaseStatePort;
        yarp::os::BufferedPort<yarp::sig::Vector> internalStateAndStdDevPort;
        yarp::os::BufferedPort<yarp::sig::Vector> contactStatePort;
    } m_comms;

    iDynTree::Model m_model;
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn;
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge> m_robotSensorBridge;
    std::unique_ptr<BipedalLocomotion::Estimators::FloatingBaseEstimator> m_estimator;
    std::unique_ptr<iDynTree::ContactStateMachine> m_lFootCSM, m_rFootCSM;
    bool m_currentlFootState{false}, m_currentrFootState{false};
    double m_currentlContactNormal{0.0}, m_currentrContactNormal{0.0};
    std::mutex m_deviceMutex;
    std::string m_portPrefix{"/base-estimator"};
    std::string m_robot{"icubSim"};
    std::string m_estimatorType{"InvEKF"};
    std::string m_baseLinkImuName{"root_link_imu_acc"};
    std::string m_leftFootWrenchName{"left_foot_cartesian_wrench"};
    std::string m_rightFootWrenchName{"right_foot_cartesian_wrench"};

    yarp::dev::PolyDriver m_transformBroadcaster;
    yarp::dev::IFrameTransform* m_transformInterface{nullptr};
    bool m_publishROSTF{false};

    Eigen::Vector3d m_basePos;
    Eigen::Vector3d m_baseRPY; // rpy euler angles in xyz convention
    Eigen::Vector3d m_baseLinearVel;
    Eigen::Vector3d m_baseAngularVel;
};



#endif //BIPEDAL_LOCOMOTION_FRAMEWORK_BASE_ESTIMATOR_DEVICE_H
