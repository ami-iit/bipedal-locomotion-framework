/**
 * @file ISensorBridge.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ISENSOR_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ISENSOR_BRIDGE_H

#include <memory>
#include <string>
#include <vector>
#include <unordered_map>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

using Vector12d = Eigen::Matrix<double, 12, 1>;
using Vector6d = Eigen::Matrix<double, 6, 1>;

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * Sensor bridge options
 */
struct SensorBridgeOptions
{
    bool isKinematicsEnabled{false}; /**< flag to connect kinematics measurement sources */
    bool isIMUEnabled{false}; /**< flag to connect IMU measurement sources */
    bool isLinearAccelerometerEnabled{false}; /**< flag to connect linear accelerometer measurement sources */
    bool isGyroscopeEnabled{false}; /**< flag to connect gyroscope measurement sources */
    bool isOrientationSensorEnabled{false}; /**< flag to connect gyroscope measurement sources */
    bool isMagnetometerEnabled{false}; /**< flag to connect magnetometer measurement sources */
    bool isSixAxisForceTorqueSensorEnabled{false}; /**< flag to connect six axis force torque measurement sources */
    bool isThreeAxisForceTorqueSensorEnabled{false}; /**< flag to connect six axis force torque measurement sources */
    bool isCartesianWrenchEnabled{false}; /**< flag to connect cartesian wrench measurement sources */

    size_t nrJoints{0}; /**< number of joints available through Kinematics stream, to be configured at initialization */
};

/**
 *  Sensor lists
 */
struct SensorLists
{
    std::vector<std::string> jointsList; /**< list of joints attached to the bridge */
    std::vector<std::string> IMUsList;   /**< list of IMUs attached to the bridge */
    std::vector<std::string> linearAccelerometersList; /**< list of linear accelerometers attached to the bridge */
    std::vector<std::string> gyroscopesList; /**< list of gyroscopes attached to the bridge */
    std::vector<std::string> orientationSensorsList; /**< list of orientation sensors attached to the bridge */
    std::vector<std::string> magnetometersList; /**< list of magnetometers attached to the bridge */
    std::vector<std::string> sixAxisForceTorqueSensorsList; /**< list of six axis force torque sensors attached to the bridge */
    std::vector<std::string> threeAxisForceTorqueSensorsList; /**< list of three axis force torque sensors attached to the bridge */
    std::vector<std::string> cartesianWrenchesList; /**< list of cartesian wrench streams attached to the bridge */
};


/**
 * Meta data struct to hold list of sensors and configured options
 * available from the Sensor bridge interface
 */
struct SensorBridgeMetaData
{
  SensorLists sensorsList;
  SensorBridgeOptions bridgeOptions;
};

/**
 * Sensor bridge interface.
 */
class ISensorBridge
{
public:
    using unique_ptr = std::unique_ptr<ISensorBridge>;

    using shared_ptr = std::shared_ptr<ISensorBridge>;

    using weak_ptr = std::weak_ptr<ISensorBridge>;

    /**
     * Initialize estimator
     * @param[in] handler Parameters handler
     */
    virtual bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) = 0;

    /**
     * Get joints list
     * @param[out] jointsList list of joints attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getJointsList(std::vector<std::string>& jointsList) { return false; };

    /**
     * Get imu sensors
     * @param[out] IMUsList list of IMUs attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getIMUsList(std::vector<std::string>& IMUsList) { return false; };

    /**
     * Get linear accelerometers
     * @param[out] linearAccelerometersList list of linear accelerometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getLinearAccelerometersList(std::vector<std::string>& linearAccelerometersList) { return false; };

    /**
     * Get gyroscopes
     * @param[out] gyroscopesList list of gyroscopes attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getGyroscopesList(std::vector<std::string>& gyroscopesList) { return false; };

    /**
     * Get orientation sensors
     * @param[out] orientationSensorsList list of orientation sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getOrientationSensorsList(std::vector<std::string>& orientationSensorsList) { return false; };

    /**
     * Get magnetometers sensors
     * @param[out] magnetometersList list of magnetometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getMagnetometersList(std::vector<std::string>& magnetometersList) { return false; };

    /**
     * Get 6 axis FT sensors
     * @param[out] sixAxisForceTorqueSensorsList list of 6 axis force torque sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getSixAxisForceTorqueSensorsList(std::vector<std::string>& sixAxisForceTorqueSensorsList) { return false; };

    /**
     * Get 6 axis FT sensors
     * @param[out] threeAxisForceTorqueSensorsList list of 3 axis force torque sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getThreeAxisForceTorqueSensorsList(std::vector<std::string>& threeAxisForceTorqueSensorsList) { return false; };

   /**
     * Get cartesian wrenches
     * @param[out] cartesianWrenchesList list of cartesian wrenches attached to the bridge
     * @return  true/false in case of success/failure
     */
    virtual bool getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList) { return false; };

    /**
     * Get joint position  in radians
     * @param[in] jointName name of the joint
     * @param[out] jointPosition joint position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getJointPosition(const std::string& jointName,
                                  double& jointPosition,
                                  double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get all joints' positions in radians
     * @param[out] parameter all joints' position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointPositions" to this method
     *
     * @return true/false in case of success/failure
     */
    virtual bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                                   double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get joint velocity in rad/s
     * @param[in] jointName name of the joint
     * @param[out] jointVelocity joint velocity in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getJointVelocity(const std::string& jointName,
                                  double& jointVelocity,
                                  double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get all joints' velocities in rad/s
     * @param[out] parameter all joints' velocities in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "jointVelocties" to this method
     *
     * @return true/false in case of success/failure
     */
    virtual bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                                    double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get IMU measurement
     * The serialization of measurments is as follows,
     *   (rpy acc omega mag)
     *  - rpy in radians Roll-Pitch-Yaw Euler angles
     *  - acc in m/s^2 linear accelerometer measurements
     *  - omega in rad/s gyroscope measurements
     *  - mag in tesla magnetometer measurements
     * @param[in] imuName name of the IMU
     * @param[out] imuMeasurement imu measurement of size 12
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getIMUMeasurement(const std::string& imuName,
                                   Eigen::Ref<Vector12d> imuMeasurement,
                                   double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get linear accelerometer measurement in m/s^2
     * @param[in] accName name of the linear accelerometer
     * @param[out] accMeasurement linear accelerometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getLinearAccelerometerMeasurement(const std::string& accName,
                                                   Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                                   double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get gyroscope measurement in rad/s
     * @param[in] gyroName name of the gyroscope
     * @param[out] gyroMeasurement gyroscope measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getGyroscopeMeasure(const std::string& gyroName,
                                     Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                                     double* receiveTimeInSeconds = nullptr) { return false; };

   /**
     * Get orientation sensor measurement in radians as roll pitch yaw Euler angles
     * @param[in] rpyName name of the orientation sensor
     * @param[out] rpyMeasurement rpy measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getOrientationSensorMeasurement(const std::string& rpyName,
                                                 Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                                 double* receiveTimeInSeconds = nullptr) { return false; };

   /**
     * Get magentometer measurement in tesla
     * @param[in] magName name of the magnetometer
     * @param[out] magMeasurement magnetometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getMagnetometerMeasurement(const std::string& magName,
                                            Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                            double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get six axis force torque measurement
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 6 containing 3d forces and 3d torques
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                                  Eigen::Ref<Vector6d> ftMeasurement,
                                                  double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Get three axis force-torque measurement containing normal force (N) and tangential moments (Nm)
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 3 containing tau_x tau_y and fz
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                                    Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                                    double* receiveTimeInSeconds = nullptr) { return false; };

   /**
     * Get 6D end effector wrenches in N and Nm for forces and torques respectively
     * @param[in] cartesianWrenchName name of the end effector wrench
     * @param[out] cartesianWrenchMeasurement end effector wrench measurement of size 6
     * @param[out] rreceiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    virtual bool getCartesianWrench(const std::string& cartesianWrenchName,
                                    Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                                    double* receiveTimeInSeconds = nullptr) { return false; };

    /**
     * Destructor
     */
    virtual ~ISensorBridge() = default;

protected:
    /**
     * Helper method to maintain SensorBridgeOptions struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeOptions SensorBridgeOptions to hold the bridge options for streaming sensor measurements
     */
    virtual bool populateSensorBridgeOptionsFromConfig(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                                      SensorBridgeOptions& sensorBridgeOptions) { return true; };

    /**
     * Helper method to maintain SensorLists struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeOptions configured object of SensorBridgeOptions
     * @param[in] sensorLists SensorLists object holding list of connected sensor devices
     */
    virtual bool populateSensorListsFromConfig(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                               const SensorBridgeOptions& sensorBridgeOptions,
                                               SensorLists& sensorLists) { return true; };

    /**
     * Helper method to maintain SensorBridgeMetaData struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeMetaData configured object of SensorBridgeMetadata
     * @param[in] sensorLists SensorLists object holding list of connected sensor devices
     */
    virtual bool populateSensorBridgeMetaDataFromConfig(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                                        SensorBridgeMetaData& sensorBridgeMetaData) { return true; };


};
} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_ISENSOR_BRIDGE_H
