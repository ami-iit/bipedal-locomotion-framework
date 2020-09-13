/**
 * @file YarpSensorBridge.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H

// std
#include <memory>

// YARP
#include <yarp/dev/PolyDriverList.h>

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/RobotInterface/ISensorBridge.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{


/**
 * YarpSensorBridge Yarp implementation of the ISensorBridge interface
 * Currently available interfaces
 * - Remapped Remote Control Board for joint states
 * - Inertial Measurement Units through generic sensor interface and remapped multiple analog sensor interface
 * - Whole Body Dynamics Estimated end effector wrenches through a generic sensor interface
 * - Force Torque Sensors through analog sensor interface and remapped multiple analog sensor interface
 * - Depth Cameras through RGBD sensor interface
 * - Camera images through OpenCV Grabber interface
 *
 */
class YarpSensorBridge : public ISensorBridge,
                         public BipedalLocomotion::System::Advanceable<SensorBridgeMetaData>
{
public:
    /**
     * Constructor
     */
    YarpSensorBridge();

    /**
     * Destructor
     */
    ~YarpSensorBridge();

    /**
     * Initialize estimator
     * @param[in] handler Parameters handler
     */
    bool initialize(std::weak_ptr<IParametersHandler> handler) final;

    /**
     * Set the list of device drivers from which the sensor measurements need to be streamed
     * @param deviceDriversList device drivers holding the pointer to sensor interfaces
     * @return True/False in case of success/failure.
     */
    bool setDriversList(const yarp::dev::PolyDriverList& deviceDriversList);

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successful.
     */
    bool advance() final;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
    bool isValid() const final;

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    const SensorBridgeMetaData& get() const final;

    /**
     * Get joints list
     * @param[out] jointsList list of joints attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getJointsList(std::vector<std::string>& jointsList) final;

    /**
     * Get imu sensors
     * @param[out] IMUsList list of IMUs attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getIMUsList(std::vector<std::string>& IMUsList) final;

    /**
     * Get linear accelerometers
     * @param[out] linearAccelerometersList list of linear accelerometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getLinearAccelerometersList(std::vector<std::string>& linearAccelerometersList) final;

    /**
     * Get gyroscopes
     * @param[out] gyroscopesList list of gyroscopes attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getGyroscopesList(std::vector<std::string>& gyroscopesList) final;

    /**
     * Get orientation sensors
     * @param[out] orientationSensorsList list of orientation sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getOrientationSensorsList(std::vector<std::string>& orientationSensorsList) final;

    /**
     * Get magnetometers sensors
     * @param[out] magnetometersList list of magnetometers attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getMagnetometersList(std::vector<std::string>& magnetometersList) final;

    /**
     * Get 6 axis FT sensors
     * @param[out] sixAxisForceTorqueSensorsList list of 6 axis force torque sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getSixAxisForceTorqueSensorsList(std::vector<std::string>& sixAxisForceTorqueSensorsList) final;

    /**
     * Get 6 axis FT sensors
     * @param[out] threeAxisForceTorqueSensorsList list of 3 axis force torque sensors attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getThreeAxisForceTorqueSensorsList(std::vector<std::string>& threeAxisForceTorqueSensorsList) final;

   /**
     * Get cartesian wrenches
     * @param[out] cartesianWrenchesList list of cartesian wrenches attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getCartesianWrenchesList(std::vector<std::string>& cartesianWrenchesList) final;

    /**
     * Get rgb cameras
     * @param[out] rgbCamerasList list of rgb cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getRGBCamerasList(std::vector<std::string>& rgbCamerasList) final;

    /**
     * Get depth cameras
     * @param[out] depthCamerasList list of depth cameras attached to the bridge
     * @return  true/false in case of success/failure
     */
    bool getDepthCamerasList(std::vector<std::string>& depthCamerasList) final;

    /**
     * Get joint position  in radians
     * @param[in] jointName name of the joint
     * @param[out] jointPosition joint position in radians
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getJointPosition(const std::string& jointName,
                          double& jointPosition,
                          double* receiveTimeInSeconds = nullptr) final;

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
    bool getJointPositions(Eigen::Ref<Eigen::VectorXd> jointPositions,
                           double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get joint velocity in rad/s
     * @param[in] jointName name of the joint
     * @param[out] jointVelocity joint velocity in radians per second
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getJointVelocity(const std::string& jointName,
                          double& jointVelocity,
                          double* receiveTimeInSeconds = nullptr) final;

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
    bool getJointVelocities(Eigen::Ref<Eigen::VectorXd> jointVelocties,
                            double* receiveTimeInSeconds = nullptr) final;

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
    bool getIMUMeasurement(const std::string& imuName,
                           Eigen::Ref<Vector12d> imuMeasurement,
                           double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get linear accelerometer measurement in m/s^2
     * @param[in] accName name of the linear accelerometer
     * @param[out] accMeasurement linear accelerometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getLinearAccelerometerMeasurement(const std::string& accName,
                                           Eigen::Ref<Eigen::Vector3d> accMeasurement,
                                           double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get gyroscope measurement in rad/s
     * @param[in] gyroName name of the gyroscope
     * @param[out] gyroMeasurement gyroscope measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getGyroscopeMeasure(const std::string& gyroName,
                             Eigen::Ref<Eigen::Vector3d> gyroMeasurement,
                             double* receiveTimeInSeconds = nullptr) final;

   /**
     * Get orientation sensor measurement in radians as roll pitch yaw Euler angles
     * @param[in] rpyName name of the orientation sensor
     * @param[out] rpyMeasurement rpy measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getOrientationSensorMeasurement(const std::string& rpyName,
                                         Eigen::Ref<Eigen::Vector3d> rpyMeasurement,
                                         double* receiveTimeInSeconds = nullptr) final;

   /**
     * Get magentometer measurement in tesla
     * @param[in] magName name of the magnetometer
     * @param[out] magMeasurement magnetometer measurements of size 3
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getMagnetometerMeasurement(const std::string& magName,
                                    Eigen::Ref<Eigen::Vector3d> magMeasurement,
                                    double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get six axis force torque measurement
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 6 containing 3d forces and 3d torques
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getSixAxisForceTorqueMeasurement(const std::string& ftName,
                                          Eigen::Ref<Vector6d> ftMeasurement,
                                          double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get three axis force-torque measurement containing normal force (N) and tangential moments (Nm)
     * @param[in] ftName name of the FT sensor
     * @param[out] ftMeasurement FT measurements of size 3 containing tau_x tau_y and fz
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getThreeAxisForceTorqueMeasurement(const std::string& ftName,
                                            Eigen::Ref<Eigen::Vector3d> ftMeasurement,
                                            double* receiveTimeInSeconds = nullptr) final;

   /**
     * Get 6D end effector wrenches in N and Nm for forces and torques respectively
     * @param[in] cartesianWrenchName name of the end effector wrench
     * @param[out] cartesianWrenchMeasurement end effector wrench measurement of size 6
     * @param[out] rreceiveTimeInSeconds time at which the measurement was received
     * @return true/false in case of success/failure
     */
    bool getCartesianWrench(const std::string& cartesianWrenchName,
                            Eigen::Ref<Vector6d> cartesianWrenchMeasurement,
                            double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get color image from the camera
     * @param[in] camName name of the camera
     * @param[out] colorImg image as Eigen matrix object
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "colorImg" to this method
     *
     * @return true/false in case of success/failure
     */
    bool getColorImage(const std::string& camName,
                       Eigen::Ref<Eigen::MatrixXd> colorImg,
                       double* receiveTimeInSeconds = nullptr) final;

    /**
     * Get depth image
     * @param[in] camName name of the gyroscope
     * @param[out] depthImg depth image as a Eigen matrix object
     * @param[out] receiveTimeInSeconds time at which the measurement was received
     *
     * @warning the size is decided at the configuration and remains fixed,
     * and internal checks must be done at the implementation level by the Derived class.
     * This means that the user must pass a resized argument "depthImg" to this method
     *
     * @return true/false in case of success/failure
     */
    bool getDepthImage(const std::string& camName,
                       Eigen::Ref<Eigen::MatrixXd> depthImg,
                       double* receiveTimeInSeconds = nullptr) final;

protected:
    /**
     * Helper method to maintain SensorBridgeOptions struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeOptions SensorBridgeOptions to hold the bridge options for streaming sensor measurements
     */
    bool populateSensorBridgeOptionsFromConfig(std::weak_ptr<IParametersHandler> handler,
                                               SensorBridgeOptions& sensorBridgeOptions) final;

    /**
     * Helper method to maintain SensorLists struct by populating it from the configuration parameters
     * @note the user may choose to use/not use this method depending on their requirements for the implementation
     * if the user chooses to not use the method, the implementation must simply contain "return true;"
     *
     * @param[in] handler  Parameters handler
     * @param[in] sensorBridgeOptions configured object of SensorBridgeOptions
     * @param[in] sensorLists SensorLists object holding list of connected sensor devices
     */
     bool populateSensorListsFromConfig(std::weak_ptr<IParametersHandler> handler,
                                        const SensorBridgeOptions& sensorBridgeOptions,
                                        SensorLists& sensorLists) final;
private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotInterface
} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H
