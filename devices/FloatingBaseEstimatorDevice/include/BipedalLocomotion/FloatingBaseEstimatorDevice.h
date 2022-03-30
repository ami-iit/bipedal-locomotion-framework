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

#include <BipedalLocomotion/FloatingBaseEstimators/KinematicInertialFilterWrapper.h>
#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>
#include <BipedalLocomotion/FloatingBaseEstimators/LeggedOdometry.h>
#include <BipedalLocomotion/ContactDetectors/SchmittTriggerDetector.h>

#include <iDynTree/ModelIO/ModelLoader.h>

#include <yarp/os/PeriodicThread.h>
#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/Wrapper.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/dev/IFrameTransform.h>

#include <mutex>
#include <memory>

namespace BipedalLocomotion
{

using LOSchmittWrapper = Estimators::KinematicInertialFilterWrapper<Estimators::LeggedOdometry,
                                                                    Contacts::SchmittTriggerDetector>;
using InvEKFSchmittWrapper = Estimators::KinematicInertialFilterWrapper<Estimators::InvariantEKFBaseEstimator,
                                                                        Contacts::SchmittTriggerDetector>;

enum class BaseEstimatorType
{
    LeggedOdom,
    InvEKF
};

enum class ContactDetectorType
{
    SchmittTrigger
};

class FloatingBaseEstimatorDevice : public yarp::dev::DeviceDriver,
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
    bool setupBaseEstimator(yarp::os::Searchable& config);
    bool loadTransformBroadcaster();

    bool updateMeasurements();
    bool updateInertialBuffers();
    bool updateContactWrenches();
    bool updateKinematics();
    bool updateEstimator();

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

    struct Communication
    {
        yarp::os::BufferedPort<yarp::sig::Vector> floatingBaseStatePort;
        yarp::os::BufferedPort<yarp::sig::Vector> internalStateAndStdDevPort;
        yarp::os::BufferedPort<yarp::sig::Vector> contactStatePort;
    };

    Communication m_comms;

    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn;
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge> m_robotSensorBridge;

    // Base Estimators and Contact Detectors
    std::unordered_map<std::string, BaseEstimatorType> m_supportedEstimatorLookup{
        {"LeggedOdometry", BaseEstimatorType::LeggedOdom},
        {"InvEKF", BaseEstimatorType::InvEKF}};
    BaseEstimatorType m_estimatorType{BaseEstimatorType::LeggedOdom};
    std::unordered_map<std::string, ContactDetectorType> m_supportedContactDetectorLookup{
        {"SchmittTrigger", ContactDetectorType::SchmittTrigger}};
    ContactDetectorType m_contactDetectorType{ContactDetectorType::SchmittTrigger};
    std::unique_ptr<InvEKFSchmittWrapper> m_invEKFSchmitt{nullptr};
    std::unique_ptr<LOSchmittWrapper> m_leggedOdomSchmitt{nullptr};

    Estimators::ProprioceptiveInput m_input;
    Estimators::KinematicInertialFilterOutput m_output;

    // meta data
    bool m_currentlFootState{false}, m_currentrFootState{false}; // for publishing
    double m_currentlContactNormal{0.0}, m_currentrContactNormal{0.0}; // for publishing
    std::mutex m_deviceMutex;
    std::string m_portPrefix{"/base-estimator"};
    std::string m_robot{"icubSim"};

    bool m_robotModelUsesFrontRearFootFTs{true};  // true for iCubGenova09, false for iCubGenova04

    // Top Level Parameters - iCubGenova09 // iCubGenova04
    std::string m_baseLinkImuName{"chest_imu_acc_1x1"}; // root_link_imu_acc
    std::vector<std::string> m_leftFootWrenchNames{"left_foot_front_cartesian_wrench", "left_foot_rear_cartesian_wrench"}; // left_foot_cartesian_wrench
    std::vector<std::string> m_rightFootWrenchNames{"right_foot_front_cartesian_wrench", "right_foot_rear_cartesian_wrench"}; // right_foot_cartesian_wrench

    const std::string m_printPrefix{"[BipedalLocomotion::FloatingBaseEstimatorDevice]"};
    yarp::dev::PolyDriver m_transformBroadcaster;
    yarp::dev::IFrameTransform* m_transformInterface{nullptr};
    bool m_publishToTFServer{false};

    Eigen::Vector3d m_basePos;
    Eigen::Vector3d m_baseRPY; // rpy euler angles in xyz convention
    Eigen::Vector3d m_baseLinearVel;
    Eigen::Vector3d m_baseAngularVel;
};

} // namespace BipedalLocomotion

#endif //BIPEDAL_LOCOMOTION_FRAMEWORK_BASE_ESTIMATOR_DEVICE_H
