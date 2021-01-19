/**
 * @file FloatingBaseEstimator.h
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_H

#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimatorParams.h>
#include <BipedalLocomotion/FloatingBaseEstimators/FloatingBaseEstimatorIO.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iostream>
#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{

/**
*  FloatingBaseEstimator class contains the bare-bones implementation for
*  a LeggedOdometry based or Extended Kalman Filter based floating base estimation
*  algorithms for bipedal robots.
*  @note it is assumed that if an IMU (primary) is specified in the inherited implementation,
*  then this IMU is rigidly attached to the specified base link in the implementation
*/
class FloatingBaseEstimator : public BipedalLocomotion::System::Advanceable<FloatingBaseEstimators::Output>
{
public:
    virtual ~FloatingBaseEstimator() { };
    /**
    *  iDynTree based model-specific computations class
    *  This is class is used in a required configuration step for the estimator
    *  All the model related kinematics and dynamics computations specific to
    *  the floating base estimator are contained within this class.
    */
    class ModelComputations
    {
    public:
        /**
        * Set the reduced model
        * @param[in] model reduced iDynTree model
        * @return True in case of success, false otherwise.
        */
        bool setModel(const iDynTree::Model& model);
        
        /**
        * Set the shared kindyn object
        * @param[in] kinDyn shared pointer of the common KinDynComputations resource        
        * @return True in case of success, false otherwise.
        * 
        * @note Expects only a valid pointer to the object, need not be loaded with the robot model.
        */
        bool setKinDynObject(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

        /**
        * Set base link name and name of the IMU rigidly attached to the IMU
        * @param[in] baseLinkFrame name of the base link frame
        * @param[in] imuFrame name of the IMU frame
        * @return True in case of success, false otherwise.
        */
        bool setBaseLinkAndIMU(const std::string& baseLinkFrame, const std::string& imuFrame);

        /**
        * Set the feet contact frames, expected to be in contact with the environment
        * @param[in] lFootContactFrame left foot contact frame
        * @param[in] rFootContactFrame right foot contact frame
        * @return True in case of success, false otherwise.
        */
        bool setFeetContactFrames(const std::string& lFootContactFrame, const std::string& rFootContactFrame);

        /**
        * Check if model is configured with the required information
        * @return True in case of success, false otherwise.
        */
        bool isModelInfoLoaded();

        /**
        * Get relative pose between IMU and the feet
        * @param[in] encoders joint positions through encoder measurements
        * @param[out] IMU_H_l_foot pose of the left foot contact frame with respect to the IMU frame
        * @param[out] IMU_H_r_foot pose of the right foot contact frame with respect to the IMU frame
        * @return True in case of success, false otherwise.
        */
        bool getIMU_H_feet(const iDynTree::JointPosDoubleArray& encoders,
                           iDynTree::Transform& IMU_H_l_foot,
                           iDynTree::Transform& IMU_H_r_foot);

        /**
        * Get relative pose between IMU and the feet
        * @param[in] encoders joint positions through encoder measurements
        * @param[out] IMU_H_l_foot pose of the left foot contact frame with respect to the IMU frame
        * @param[out] IMU_H_r_foot pose of the right foot contact frame with respect to the IMU frame
        * @param[out] J_IMULF Jacobian of left foot frame with respect to IMU frame in mixed-velocity trivialization
        * @param[out] J_IMURF Jacobian of right foot frame with respect to IMU frame in mixed-velocity trivialization
        * @return True in case of success, false otherwise.
        */
        bool getIMU_H_feet(const iDynTree::JointPosDoubleArray& encoders,
                           iDynTree::Transform& IMU_H_l_foot,
                           iDynTree::Transform& IMU_H_r_foot,
                           iDynTree::MatrixDynSize& J_IMULF,
                           iDynTree::MatrixDynSize& J_IMURF);

        /**
        * Get the base link pose and velocity from the estimated IMU pose and velocity
        * @note the input and output velocities are both specified in mixed-velocity representation
        * @param[in] A_H_IMU pose of the IMU in the world
        * @param[in] v_IMU mixed-trivialized velocity of the IMU in the world
        * @param[out] A_H_B pose of the base link in the world
        * @param[out] v_B mixed-trivialized velocity of the base link in the world
        * @return True in case of success, false otherwise.
        */
        bool getBaseStateFromIMUState(const iDynTree::Transform& A_H_IMU, const iDynTree::Twist& v_IMU,
                                      iDynTree::Transform& A_H_B, iDynTree::Twist& v_B);

        /**
         * Getters
         */
        const int& nrJoints() const { return m_nrJoints; }
        const std::string& baseLink() const { return m_baseLink; }
        const iDynTree::FrameIndex& baseLinkIdx() const { return m_baseLinkIdx; }
        const iDynTree::FrameIndex& baseIMUIdx() const { return m_baseImuIdx; }
        const std::string& baseLinkIMU() const { return m_baseImuFrame; }
        const std::string& leftFootContactFrame() const { return m_lFootContactFrame; }
        const std::string& rightFootContactFrame() const { return m_rFootContactFrame; }
        const iDynTree::Transform& base_H_IMU() const { return m_base_H_imu; }
        const bool& isModelSet() const { return m_modelSet; }
        const bool& isKinDynValid() const { return m_validKinDyn; }
        std::shared_ptr<iDynTree::KinDynComputations> kinDyn() const { return m_kindyn; }
        
    private:
        std::string m_baseLink{""}; /**< name of the floating base link*/
        std::string m_baseImuFrame{""}; /**< name of the IMU frame rigidly attached to the floating base link*/
        std::string m_lFootContactFrame{""}; /**< name of the left foot contact frame expected to be in contact with the environment*/
        std::string m_rFootContactFrame{""}; /**< name of the right foot contact frame expected to be in contact with the environment*/
        iDynTree::FrameIndex m_baseLinkIdx{iDynTree::FRAME_INVALID_INDEX}; /**< base link's frame index in the loaded model*/
        iDynTree::FrameIndex m_baseImuIdx{iDynTree::FRAME_INVALID_INDEX}; /**< IMU index in the loaded model*/
        iDynTree::FrameIndex m_lFootContactIdx{iDynTree::FRAME_INVALID_INDEX}; /**< Left foot contact frame index in the loaded model*/
        iDynTree::FrameIndex m_rFootContactIdx{iDynTree::FRAME_INVALID_INDEX}; /**< Right foot contact freame index in the loaded model*/

        std::shared_ptr<iDynTree::KinDynComputations> m_kindyn{nullptr}; /**< KinDynComputations object to do the model specific computations */
        iDynTree::Transform m_base_H_imu; /**< Rigid body transform of IMU frame with respect to the base link frame */
        int m_nrJoints{0}; /**< number of joints in the loaded reduced model */
        bool m_modelSet{false};
        bool m_validKinDyn{false};
    };


    /**
    * Configure generic parameters and the model. The generic parameters include,
    *  - sampling_period_in_s [PARAMETER|REQUIRED|Sampling period of the estimator in seconds]
    *  - ModelInfo [GROUP|REQUIRED|URDF Model specific parameters used by the estimator]
    *      - base_link [PARAMETER|REQUIRED|base link frame from the URDF model| Exists in "ModelInfo" GROUP]
    *      - base_link_imu [PARAMETER|REQUIRED|IMU frame rigidly attached to thebase link from the URDF model| Exists in "ModelInfo" GROUP]
    *      - left_foot_contact_frame [PARAMETER|REQUIRED|left foot contact frame from the URDF model| Exists in "ModelInfo" GROUP]
    *      - right_foot_contact_frame [PARAMETER|REQUIRED|right foot contact frame from the URDF model| Exists in "ModelInfo" GROUP]
    * @param[in] handler configure the generic parameters for the estimator
    * @param[in] kindyn shared pointer of iDynTree kindyncomputations object (model will be loaded internally)
    * @param[in] model reduced iDynTree model required by the estimator
    * @note any custom initialization of parameters or the algorithm implementation is not done here,
    *       it must be done in customInitialization() by the child class implementing the algorithm
    * @return True in case of success, false otherwise.
    */
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler, 
                    std::shared_ptr<iDynTree::KinDynComputations> kindyn,
                    const iDynTree::Model& model);
    

    /**
    * Set the polled IMU measurement
    * @param[in] accMeas linear accelerometer measurement expressed in the local IMU frame (m/s^2)
    * @param[in] gyroMeas gyroscope measurement expressed in the local IMU frame (rad/s)
    * @return True in case of success, false otherwise.
    */
    bool setIMUMeasurement(const Eigen::Vector3d& accMeas,
                           const Eigen::Vector3d& gyroMeas);

    /**
    * Set feet contact states
    * @param[in] lfInContact left foot contact state [0, 1]
    * @param[in] rfInContact right foot contact state [0, 1]
    * @return True in case of success, false otherwise.
    */
    bool setContacts(const bool& lfInContact, const bool& rfInContact);

    /**
    * Set contact status
    * 
    * @param[in] name contact frame name
    * @param[in] contactStatus flag to check active contact
    * @param[in] timeNow  time of measurement update
    */
    bool setContactStatus(const std::string& name, 
                          const bool& contactStatus, 
                          const double& timeNow);

    /**
    * Set kinematic measurements
    * @note it is assumed that the order of the joints loaded in the model and the order of the measurements in these vectors match
    * @param[in] encoders joint positions measured through encoders
    * @param[in] encoderSpeeds joint velocities measured through encoders
    * @return True in case of success, false otherwise.
    */
    bool setKinematics(const Eigen::VectorXd& encoders,
                       const Eigen::VectorXd& encoderSpeeds);

    /**
    * Compute one step of the estimator
    * @return True in case of success, false otherwise.
    */
    virtual bool advance() final;

    /**
     * Reset the internal state of the estimator
     * @param[in] newState Internal state of the estimator
     * @return True in case of success, false otherwise.
     *
     * @note reset and advance estimator to get updated estimator output
     */
    virtual bool resetEstimator(const FloatingBaseEstimators::InternalState& newState);

    /**
     * Reset the base pose estimate and consequently the internal state of the estimator
     * @param[in] newBaseOrientation base link orientation as a Eigen quaternion
     * @param[in] newBasePosition base link position
     * @return True in case of success, false otherwise.
     *
     * * @note reset and advance estimator to get updated estimator output
     */
    virtual bool resetEstimator(const Eigen::Quaterniond& newBaseOrientation,
                                const Eigen::Vector3d& newBasePosition);

    /**
    * Get estimator outputs
    * @return A struct containing he estimated internal states of the estiamtor and the associated covariance matrix
    */
    virtual const FloatingBaseEstimators::Output& get() const final;

    /**
    * Determines the validity of the object retrieved with get()
    * @return True in case of success, false otherwise.
    */
    virtual bool isValid() const final { return (m_estimatorState == State::Running); };

    /**
    * Get ModelComputations object by reference
    * @return ModelComputations object providing information between considered model related quantities in the estimator
    * like the base link, IMU, feet contact frames.
    */
    ModelComputations& modelComputations();

protected:
    /**
    * Configure generic parameters
    * @param[in] handler configure the generic parameters for the estimator
    * @return True in case of success, false otherwise.
    */
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * These custom parameter specifications should be specified by the derived class.
    * - If setupOptions() is called from within customInitialization(), ensure the group "Options" exist.
    *   Please check the documentation of setupOptions() for relevant parameters
    * - If setupSensorDevs() is called from within customInitialization(), ensure the group "SensorsStdDev" exist.
    *   Please check the documentation of setupSensorDevs() for relevant parameters
    * - If setupInitialStates() is called from within customInitialization(), ensure the group "InitialStates" exist.
    *   Please check the documentation of setupInitialStates() for relevant parameters
    * - If setupPriorDevs() is called from within customInitialization(), ensure the group "PriorsStdDev" exist.
    *   Please check the documentation of setupPriorDevs() for relevant parameters
    * @param[in] handler configure the custom parameters for the estimator
    * @return bool
    */
    virtual bool customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler) { return true; };

    /**
    * Propagate the states through the prediction model, if there exists any (eg. a strap-down IMU model)
    * @param[in] meas measurements passed as exogeneous inputs to the prediction model
    * @param[in] dt sampling period in seconds
    * @param[out] state previous state estimate
    * @param[out] P previous state covariance matrix
    * @return True in case of success, false otherwise.
    */
    virtual bool predictState(const FloatingBaseEstimators::Measurements& meas,
                              const double& dt) { return true; };

    /**
    * Update the predicted state estimates using kinematics and contact measurements
    * @param[in] meas measurements to update the predicted states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateKinematics(FloatingBaseEstimators::Measurements& meas,
                                  const double& dt) { return true; };

    /**
    * Setup estimator options. The parameters in the Options group are,
    * - enable_imu_bias_estimation [PARAMETER|-|Enable/disable IMU bias estimation]
    * - enable_static_imu_bias_initialization [PARAMETER|-|Enable/disable IMU bias initialization assuming static configuration]
    * - nr_samples_for_imu_bias_initialization [PARAMETER|REQUIRED, if staticImuBiasInitializationEnabled is set to true|Number of samples for static bias initialization]
    * - enable_ekf_update [PARAMETER|-|Enable/disable update step of EKF (not recommended to set to false)]
    * - acceleration_due_to_gravity [PARAMETER|-|Acceleration due to gravity, 3d vector]
    * @note this is a recipe method that can be called by the derived class from within customInitialization()
    * @param[in] handler parameter handler
    * @return True in case of success, false otherwise.
    */
    bool setupOptions(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * Setup measurement noise standard deviations The parameters in the SensorsStdDev group are,
    * - accelerometer_measurement_noise_std_dev [PARAMETER|REQUIRED|Accelerometer measurement white noise deviation]
    * - gyroscope_measurement_noise_std_dev [PARAMETER|REQUIRED|Gyroscope measurement white noise deviation]
    * - accelerometer_measurement_bias_noise_std_dev [PARAMETER|REQUIRED, if imuBiasEstimationEnabled is set to true|Accelerometer measurement bias noise deviation]
    * - gyroscope_measurement_bias_noise_std_dev [PARAMETER|REQUIRED, if imuBiasEstimationEnabled is set to true|Gyroscope measurement bias noise deviation]
    * - contact_foot_linear_velocity_noise_std_dev [PARAMETER|REQUIRED|Linear velocity white noise deviation when foot is in contact]
    * - contact_foot_angular_velocity_noise_std_dev [PARAMETER|REQUIRED|Angular velocity white noise deviation when foot is in contact]
    * - swing_foot_linear_velocity_noise_std_dev [PARAMETER|REQUIRED|Linear velocity white noise deviation when foot is swinging off contact]
    * - swing_foot_angular_velocity_noise_std_dev [PARAMETER|REQUIRED|Angular velocity white noise deviation when foot is swinging off contact]
    * - forward_kinematic_measurement_noise_std_dev [PARAMETER|REQUIRED|White noise deviation in relative poses computed through forward kinematics]
    * - encoders_measurement_noise_std_dev [PARAMETER|REQUIRED|Encoder measurement white noise deviation]
    * @note this is a recipe method that can be called by the derived class from within customInitialization()
    * @note ensure to call setupOptions() before calling setupSensorDevs() to handle bias related parameters to be configured properly
    * @param[in] handler parameter handler
    * @return True in case of success, false otherwise.
    */
    bool setupSensorDevs(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * Setup initial states. The parameters in the InitialStates group are,
    * - imu_orientation_quaternion_wxyz [PARAMETER|REQUIRED|Initial IMU orientation  wrt inertial frame, as a quaternion]
    * - imu_position_xyz [PARAMETER|REQUIRED|Initial IMU position wrt inertial frame]
    * - imu_linear_velocity_xyz [PARAMETER|REQUIRED|Initial IMU linear velocity wrt inertial frame, in a mixed-velocity representation]
    * - l_contact_frame_orientation_quaternion_wxyz [PARAMETER|REQUIRED|Initial left foot contact frame orientation wrt inertial frame, as a quaternion]
    * - l_contact_frame_position_xyz [PARAMETER|REQUIRED|Initial left foot contact frame position wrt inertial frame]
    * - r_contact_frame_orientation_quaternion_wxyz [PARAMETER|REQUIRED|Initial right foot contact frame orientation wrt inertial frame, as a quaternion]
    * - r_contact_frame_position_xyz [PARAMETER|REQUIRED|Initial right foot contact frame position wrt inertial frame]
    * - accelerometer_bias [PARAMETER|REQUIRED, if imuBiasEstimationEnabled is set to true|Initial accelerometer bias expressed in IMU frame]
    * - gyroscope_bias [PARAMETER|REQUIRED, if imuBiasEstimationEnabled is set to true|Initial gyroscope bias expressed in IMU frame]
    * @note this is a recipe method that can be called by the derived class from within customInitialization()
    * @note ensure to call setupOptions() before calling setupInitialStates() to handle bias related parameters to be configured properly
    * @param[in] handler parameter handler
    * @return True in case of success, false otherwise.
    */
    bool setupInitialStates(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * Setup initial state standard deviations. The parameters in the PriorsStdDev group are,
    * - imu_orientation_quaternion_wxyz [PARAMETER|REQUIRED|Initial IMU orientation as wrt inertial frame]
    * - imu_position [PARAMETER|REQUIRED|Initial IMU position deviation wrt inertial frame]
    * - imu_linear_velocity [PARAMETER|REQUIRED|Initial IMU linear velocity deviation wrt inertial frame, in a mixed-velocity representation]
    * - l_contact_frame_orientation_quaternion [PARAMETER|REQUIRED|Initial left foot contact frame orientation deviation wrt inertial frame]
    * - l_contact_frame_position [PARAMETER|REQUIRED|Initial left foot contact frame position deviation wrt inertial frame]
    * - r_contact_frame_orientation_quaternion [PARAMETER|REQUIRED|Initial right foot contact frame orientation deviation wrt inertial frame]
    * - r_contact_frame_position [PARAMETER|REQUIRED|Initial right foot contact frame position deviation wrt inertial frame]
    * - accelerometer_bias [PARAMETER|REQUIRED, if imuBiasEstimationEnabled is set to true|Initial accelerometer bias devitaion expressed in IMU frame]
    * - gyroscope_bias [PARAMETER|REQUIRED, if imuBiasEstimationEnabled is set to true|Initial gyroscope bias deviation expressed in IMU frame]
    * @note this is a recipe method that can be called by the derived class from within customInitialization()
    * @note ensure to call setupOptions() before calling setupPriorDevs() to handle bias related parameters to be configured properly
    * @param[in] handler parameter handler
    * @return True in case of success, false otherwise.
    */
    bool setupPriorDevs(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    ModelComputations m_modelComp; /**< Model computations object */
    FloatingBaseEstimators::Options m_options; /**< Struct holding estimator options */
    FloatingBaseEstimators::Measurements m_meas, m_measPrev; /**< Struct holding the latest measurements that were set to the estimator */
    FloatingBaseEstimators::InternalState m_state, m_statePrev; /**< Structs holding the curent and previous internal states of the estimation algorithm */
    FloatingBaseEstimators::Output m_estimatorOut; /**< Struct holding outputs of the estimator */
    FloatingBaseEstimators::StateStdDev m_priors, m_stateStdDev; /**< Struct holding standard deviations associated to prior state estimates */
    FloatingBaseEstimators::SensorsStdDev m_sensorsDev; /**< Struct holding standard deviations associated to sensor measurements */

    /**
    * Enumerator used to determine the running state of the estimator
    */
    enum class State
    {
        NotInitialized, /**< The estimator is not initialized yet call FloatingBaseEstimator::initialze
                           method to initialize it*/
        Initialized,    /**< The estimator is initialized and ready to be used */
        Running         /**< The estimator is running */
    };

    State m_estimatorState{State::NotInitialized}; /**< State of the estimator */

    double m_dt{0.01}; /**< Fixed time step of the estimator, in seconds */
    bool m_useIMUForAngVelEstimate{true}; /**< by default set to true for strap down IMU based EKF implementations, if IMU measurements not used, corresponding impl can set to false */
    bool m_useIMUVelForBaseVelComputation{true};
private:
    /**
    * Setup model related parameters
    *
    * @param[in] handler parameter handler
    * @return True in case of success, false otherwise.
    */
    bool setupModelParams(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);


    /**
     * Setup parameter vector
     * @param[in] param parameter name
     * @param[in] prefix print prefix
     * @param[in] handler parameter handler
     * @param[out] vec parameter vector
     * @return True in case of success, false otherwise.
     */
    bool setupFixedVectorParamPrivate(const std::string& param, const std::string& prefix,
                                      std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                                      std::vector<double>& vec);

    /**
     * Wrapper method for getting base state from internal IMU state
     * @param[in] state internal state of the estimator
     * @param[in] meas previous measurement to the estimator
     * @param[in] basePose base pose as an iDynTree Transform object
     * @param[in] baseTwist mixe- trivialized base velocity as an iDynTree Twist object
     */
    bool updateBaseStateFromIMUState(const FloatingBaseEstimators::InternalState& state,
                                     const FloatingBaseEstimators::Measurements& meas,
                                     iDynTree::Transform& basePose,
                                     iDynTree::Twist& baseTwist);
};


} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_H

