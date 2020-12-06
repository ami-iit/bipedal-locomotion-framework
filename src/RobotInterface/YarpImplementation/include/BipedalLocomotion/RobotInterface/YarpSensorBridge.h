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
 * The YarpSensorBridge expects a list of device drivers through the yarp::dev::PolyDriverList object.
 * Each PolyDriver object in the list is compared with the configured sensor names and the assumptions listed below
 * to infer the sensor types and relevant interfaces in order to to read the relevant data.
 *
 * MAJOR ASSUMPTIONS
 * - Every sensor unit(device driver) attached to this Bridge is identified by a unique name
 * - A single instance of a remote control board remapper and a multiple analog sensor remapper is expected if the suer wants to use the control board interfaces and multiple analog sensor interfaces
 * - Any generic sensor interface with channel dimensions of 6 is considered to be a cartesian wrench interface
 * - Any generic sensor interface with channel dimensions of 12 is considered as a IMU interface (server inertial)
 * - Any analog sensor interface with channel dimensions of 6 is considered as a force torque sensor interface
 * - The images are available through a FrameGrabber interface (RGB only) and a RGBD interface (RGB and Depth).
 * - The current internal design (read all sensors in a serial fashion) may not be suitable for a heavy measurement set
 *
 * The parameters for writing the configuration file for this class is given as,
 * |     Group                  |         Parameter               | Type              |                   Description                   |
 * |:--------------------------:|:-------------------------------:|:-----------------:|:---------------------------------------------- :|
 * |                            |check_for_nan                    | boolean           |flag to activate checking for NANs in the incoming measurement buffers, not applicable for images|
 * |                            |stream_joint_states              | boolean           |Flag to activate the attachment to IMU sensor devices       |
 * |                            |stream_inertials                 | boolean           |Flag to activate the attachment to IMU sensor devices       |
 * |                            |stream_cartesian_wrenches        | boolean           |Flag to activate the attachment to Cartesian wrench related devices       |
 * |                            |stream_forcetorque_sensors       | boolean           |Flag to activate the attachment to six axis FT sensor devices       |
 * |                            |stream_cameras                   | boolean           |Flag to activate the attachment to Cameras devices       |
 * |RemoteControlBoardRemapper  |                                 |                   |Expects only one remapped remotecontrolboard device attached to it, if there multiple remote control boards, then  use a remapper to create a single remotecontrolboard |
 * |                            |joints_list                      | vector of strings |This parameter is **optional**. The joints list used to open the remote control board remapper. If the list is not passed, the order of the joint stored in the PolyDriver is used       |
 * |InertialSensors             |                                 |                   |Expects IMU to be opened as genericSensorClient devices communicating through the inertial server and other inertials as a part multiple analog sensors remapper ("multipleanalogsensorsremapper") |
 * |                            |imu_list                         | vector of strings |list of the names of devices opened as genericSensorClient device and having a channel dimension of 12      |
 * |                            |gyroscopes_list                  | vector of strings |list of the names of devices opened with ThreeAxisGyroscope interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3  |
 * |                            |accelerometers_list              | vector of strings |list of the names of devices opened with ThreeAxisLinearAccelerometers interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3 |
 * |                            |orientation_sensors_list         | vector of strings |list of the names of devices opened with OrientationSensors interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3 |
 * |                            |magnetometers_list               | vector of strings |list of the names of devices opened with ThreeAxisMagnetometer interface remapped through the "multipleanalogsensorsremapper" interfaces and having a channel dimension of 3 |
 * |CartesianWrenches           |                                 |                   |Expects the devices wrapping the cartesian wrenches ports to be opened as "genericSensorClient" device and have a channel dimension of 6                      |
 * |                            |cartesian_wrenches_list          | vector of strings |list of the names of devices opened as genericSensorClient device and having a channel dimension of 6      |
 * |SixAxisForceTorqueSensors   |                                 |                   |Expects the Six axis FT sensors to be opened with SixAxisForceTorqueSensors interface remapped through multiple analog sensors remapper ("multipleanalogsensorsremapper") or to be opened as analog sensor ("analogsensorclient") device having channel dimensions as 6|
 * |                            |sixaxis_forcetorque_sensors_list | vector of strings |list of six axis FT sensors (the difference between a MAS FT and an analog FT is done internally assuming that the names are distinct form each other)|
 * |Cameras                     |                                 |                   |Expects cameras to be opened either as remote frame grabber ("RemoteFrameGrabber") with IFrameGrabber interface or rgbd sensor ("RGBDSensorClient") with IRGBDSensor interface|
 * |                            |rgbd_list                        | vector of strings |list containing the devices opened as RGBDSensorClients containing the IRGBD sensor interface      |
 * |                            |rgbd_image_width                 | vector of strings |list containing the image width dimensions of RGBD cameras. Required parameter if cameras are enabled. The list must be the same size and order as rgbd_list |
 * |                            |rgbd_image_height                | vector of strings |list containing the image height dimensions of RGBD cameras. Required parameter if cameras are enabled. The list must be the same size and order as rgbd_list |
 * |                            |rgb_cameras_list                 | vector of strings |list containing the devices opened as RemoteFrameGrabber devices containing the IFrameGrabber interface|
 * |                            |rgb_image_width                  | vector of strings |list containing the image width dimensions of RGB cameras. Required parameter if cameras are enabled. The list must be the same size and order as rgb_list |
 * |                            |rgb_image_height                 | vector of strings |list containing the image height dimensions of RGB cameras. Required parameter if cameras are enabled. The list must be the same size and order as rgb_list |
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
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) final;

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
     * @brief list of sensors that was failed to be read in the current advance() step
     * @return list of sensors as a vector of strings
     */
    std::vector<std::string> getFailedSensorReads();

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

private:
    /** Private implementation */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;
};

} // namespace RobotInterface
} // namespace BipedalLocomotion


#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_SENSOR_BRIDGE_H
