/**
 * @file RobotDynamicsEstimatorDevice.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_FRAMEWORK_ROBOT_DYNAMICS_ESTIMATOR_DEVICE_H
#define BIPEDAL_LOCOMOTION_FRAMEWORK_ROBOT_DYNAMICS_ESTIMATOR_DEVICE_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/RobotDynamicsEstimator.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>
#include <BipedalLocomotion/YarpUtilities/VectorsCollection.h>

#include <iDynTree/Estimation/ContactStateMachine.h>
#include <iDynTree/Estimation/ExtWrenchesAndJointTorquesEstimator.h>
#include <iDynTree/ModelLoader.h>

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/dev/IMultipleWrapper.h>
#include <yarp/dev/IVirtualAnalogSensor.h>
#include <yarp/os/PeriodicThread.h>
#include <yarp/os/ResourceFinder.h>

#include <memory>
#include <mutex>
#include <thread>
#include <unordered_map>

namespace BipedalLocomotion
{
class RobotDynamicsEstimatorDevice;
}

/**
 * RobotDynamicsEstimatorDevice is a concrete class and implements yarp device.
 * The device uses the RobotDynamicsEstimator library to estimate joint torques and
 * external contact wrenches.
 */
class BipedalLocomotion::RobotDynamicsEstimatorDevice : public yarp::dev::DeviceDriver,
                                                        public yarp::dev::IMultipleWrapper,
                                                        public yarp::os::PeriodicThread
{
public:
    /**
     * Constructor.
     */
    RobotDynamicsEstimatorDevice(double period,
                                 yarp::os::ShouldUseSystemClock useSystemClock
                                 = yarp::os::ShouldUseSystemClock::No);
    /**
     * Constructor.
     */
    RobotDynamicsEstimatorDevice();

    /**
     * Destructor.
     */
    ~RobotDynamicsEstimatorDevice();

    /**
     * Open the DeviceDriver.
     * @param config is a list of parameters for the device.
     * @return true/false upon success/failure
     */
    virtual bool open(yarp::os::Searchable& config) final;

    /**
     * Close the DeviceDriver.
     * @return true/false on success/failure.
     */
    virtual bool close() final;

    /**
     * Attach drivers to the device.
     * @param poly is a yarp::dev::PolyDriverList object containing the list of drivers to attach.
     * @return true/false on success/failure.
     */
    virtual bool attachAll(const yarp::dev::PolyDriverList& poly) final;

    /**
     * Detach drivers from the device.
     * @return true/false on success/failure.
     */
    virtual bool detachAll() final;

    /**
     * Loop function. This is the thread itself.
     * The thread calls the run() function every <period> ms.
     */
    virtual void run() final;

private:
    // class members
    std::string m_portPrefix{"/rde"}; /**< Default port prefix. */
    std::string m_robot{"ergocubSim"}; /**< Robot name. Default is ergocubSim. */
    std::string m_baseLink{"root_link"}; /**< Base link name. Default is root_link. */
    std::vector<std::string> m_jointNameList{}; /**< Joint name list. */
    Eigen::VectorXd m_gearboxRatio; /**< Gearbox ratio list. */
    Eigen::VectorXd m_torqueConstant; /**< Torque constant list. */
    std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< KinDynComputations object. */
    std::unique_ptr<BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimator>
        m_estimator; /**< RobotDynamicsEstimator object. */
    std::unique_ptr<BipedalLocomotion::RobotInterface::YarpSensorBridge>
        m_robotSensorBridge; /**<
                             YarpSensorBridge
                             object.
                           */
    yarp::os::BufferedPort<BipedalLocomotion::YarpUtilities::VectorsCollection>
        m_loggerPort; /**<
                         Logger
                         port.
                       */
    std::unordered_map<std::string, Eigen::VectorXd> m_ftOffset; /**<
                                                                    Map containing the offset for
                                                                    the force torque sensors.
                                                                  */
    bool m_isFirstRun{true}; /**< Flag to check if it is the first run. */
    iDynTree::ExtWrenchesAndJointTorquesEstimator
        m_iDynEstimator; /**<
                           iDynTree
                           ExtWrenchesAndJointTorquesEstimator
                           object.
                         */
    std::thread m_publishEstimationThread; /**< Thread to publish the estimation. */
    struct EstimatorInput
    {
        std::mutex mutex;
        BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimatorInput input;
    } m_estimatorInput; /**< Estimator input. */
    struct EstimatorOutput
    {
        std::mutex mutex;
        BipedalLocomotion::Estimators::RobotDynamicsEstimator::RobotDynamicsEstimatorOutput output;
    } m_estimatorOutput; /**< Estimator output. */
    bool m_estimatorIsRunning; /**< Flag to check if the estimator is running. */
    Eigen::VectorXd m_estimatedTauj; /**< Estimated joint torques. */
    Eigen::VectorXd m_measuredTauj; /**< Measured joint torques. */
    yarp::dev::PolyDriver m_remappedVirtualAnalogSensors; /**< Remapped virtual analog sensor
                                                           containg the axes for which the joint
                                                           torques estimates are published */
    struct
    {
        yarp::dev::IVirtualAnalogSensor* ivirtsens;
        yarp::dev::IMultipleWrapper* multwrap;
    } m_remappedVirtualAnalogSensorsInterfaces; /**< Remapped virtual analog sensor interfaces. */
    yarp::sig::Vector m_estimatedJointTorquesYARP; /**< Estimated joint torques in YARP format. */
    Eigen::Vector3d m_temp3DMeasurement; /**< Temporary 3D measurement. */

    // class methods
    /**
     * Setup the robot model from parameter handler.
     * @param paramHandler is a pointer to the parameter handler.
     * @param mdlLdr is a reference to the model loader.
     * @return true/false on success/failure.
     */
    bool setupRobotModel(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler,
                         iDynTree::ModelLoader& mdlLdr);

    /**
     * Create the SubModel list and the KinDynWrapper list for the robot dynamics estimator.
     * @param paramHandler is a pointer to the parameter handler.
     * @param modelLoader is a reference to the model loader.
     * @param subModelList is a reference to the SubModel list.
     * @param kinDynWrapperList is a reference to the KinDynWrapper list.
     * @return true/false on success/failure.
     */
    bool
    createSubModels(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler,
                    iDynTree::ModelLoader& modelLoader,
                    std::vector<Estimators::RobotDynamicsEstimator::SubModel>& subModelList,
                    std::vector<std::shared_ptr<Estimators::RobotDynamicsEstimator::KinDynWrapper>>&
                        kinDynWrapperList);

    /**
     * Setup the robot dynamics estimator.
     * @param paramHandler is a pointer to the parameter handler.
     * @return true/false on success/failure.
     */
    bool setupRobotDynamicsEstimator(
        std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler);

    /**
     * Setup the robot sensor bridge.
     * @param paramHandler is a pointer to the parameter handler.
     * @return true/false on success/failure.
     */
    bool
    setupRobotSensorBridge(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler);

    /**
     * Open the communication ports.
     * @return true/false on success/failure.
     */
    bool openCommunications();

    /**
     * Update the measurements used by the estimator.
     * @return true/false on success/failure.
     */
    bool updateMeasurements();

    /**
     * Resize the estimator initial state based on configuration.
     * @param modelHandler is a pointer to the parameter handler.
     * @return true/false on success/failure.
     */
    bool resizeEstimatorInitialState(
        BipedalLocomotion::ParametersHandler::IParametersHandler::weak_ptr modelHandler);

    /**
     * Set the estimator initial state based on configuration.
     * @return true/false on success/failure.
     */
    bool setEstimatorInitialState();

    /**
     * Resize the estimator measurement based on configuration.
     * @param modelHandler is a pointer to the parameter handler.
     * @return true/false on success/failure.
     */
    bool resizeEstimatorMeasurement(
        BipedalLocomotion::ParametersHandler::IParametersHandler::weak_ptr modelHandler);

    /**
     * Published the estimator output.
     * This is a separate thread.
     */
    void publishEstimatorOutput();

    /**
     * Open the remapper virtual sensors.
     * @return true/false on success/failure.
     */
    bool openRemapperVirtualSensors();
};

#endif // BIPEDAL_LOCOMOTION_FRAMEWORK_ROBOT_DYNAMICS_ESTIMATOR_DEVICE_H
