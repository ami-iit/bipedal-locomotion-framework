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
#include <BipedalLocomotion/Estimators/FloatingBaseEstimatorParams.h>
#include <BipedalLocomotion/Estimators/FloatingBaseEstimatorIO.h>

#include <iDynTree/Model/Model.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/JointState.h>
#include <iostream>

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
class FloatingBaseEstimator : public BipedalLocomotion::System::Advanceable<FBEOutput>
{
public:
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
        * Set base link name and name of the IMU rigidly attached to the IMU
        * @param[in] base_link_frame name of the base link frame
        * @param[in] imu_frame name of the IMU frame
        * @return True in case of success, false otherwise.
        */
        bool setBaseLinkAndIMU(const std::string& base_link_frame, const std::string& imu_frame);

        /**
        * Set the feet contact frames, expected to be in contact with the environment
        * @param[in] l_foot_contact_frame left foot contact frame
        * @param[in] r_foot_contact_frame right foot contact frame
        * @return True in case of success, false otherwise.
        */
        bool setFeetContactFrames(const std::string& l_foot_contact_frame, const std::string& r_foot_contact_frame);

        /**
        * Check if model is configured with the required information
        * @note this is a required step to initialize the estimator
        * @return True in case of success, false otherwise.
        */
        bool checkModelInfoLoaded();

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
        const auto& nrJoints() const { return m_nr_joints; }
        const auto& baseLink() const { return m_base_link; }
        const auto& baseLinkIMU() const { return m_base_imu_frame; }
        const auto& leftFootContact() const { return m_l_foot_contact_frame; }
        const auto& rightFootContact() const { return m_r_foot_contact_frame; }
        const auto& base_H_IMU() const { return m_base_H_imu; }
        const auto& modelSet() const { return m_model_set; }

    private:
        std::string m_base_link{""}; /**< name of the floating base link*/
        std::string m_base_imu_frame{""}; /**< name of the IMU frame rigidly attached to the floating base link*/
        std::string m_l_foot_contact_frame{""}; /**< name of the left foot contact frame expected to be in contact with the environment*/
        std::string m_r_foot_contact_frame{""}; /**< name of the right foot contact frame expected to be in contact with the environment*/
        iDynTree::FrameIndex m_base_link_idx{iDynTree::FRAME_INVALID_INDEX}; /**< base link's frame index in the loaded model*/
        iDynTree::FrameIndex m_base_imu_idx{iDynTree::FRAME_INVALID_INDEX}; /**< IMU index in the loaded model*/
        iDynTree::FrameIndex m_l_foot_contact_idx{iDynTree::FRAME_INVALID_INDEX}; /**< Left foot contact frame index in the loaded model*/
        iDynTree::FrameIndex m_r_foot_contact_idx{iDynTree::FRAME_INVALID_INDEX}; /**< Right foot contact freame index in the loaded model*/

        iDynTree::KinDynComputations m_kindyn; /**< KinDynComputations object to do the model specific computations */
        iDynTree::Model m_model; /**< Loaded reduced model */
        iDynTree::Transform m_base_H_imu; /**< Rigid body transform of IMU frame with respect to the base link frame */
        int m_nr_joints{0}; /**< number of joints in the loaded reduced model */
        bool m_model_set{false};
    };


    /**
    * Configure generic parameters. The generic parameters include,
    *  - sampling_period_in_s [PARAMETER|REQUIRED|Sampling period of the estimator in seconds]
    *  - ModelInfo [GROUP|REQUIRED|URDF Model specific parameters used by the estimator]
    *      - base_link [PARAMETER|REQUIRED|base link frame from the URDF model| Exists in "ModelInfo" GROUP]
    *      - base_link_imu [PARAMETER|REQUIRED|IMU frame rigidly attached to thebase link from the URDF model| Exists in "ModelInfo" GROUP]
    *      - left_foot_contact_frame [PARAMETER|REQUIRED|left foot contact frame from the URDF model| Exists in "ModelInfo" GROUP]
    *      - right_foot_contact_frame [PARAMETER|REQUIRED|right foot contact frame from the URDF model| Exists in "ModelInfo" GROUP]
    * @param handler configure the generic parameters for the estimator
    * @note call modelComputations().setModel() before calling initialize
    * @note any custom initialization of parameters or the algorithm implementation is not done here,
    *       it must be done in customInitialization() by the child class implementing the algorithm
    * @return bool
    */
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
    * Set the polled IMU measurement
    * @param[in] acc_meas linear accelerometer measurement expressed in the local IMU frame (m/s^2)
    * @param[in] gyro_meas gyroscope measurement expressed in the local IMU frame (rad/s)
    * @return True in case of success, false otherwise.
    */
    bool setIMUMeasurement(const Eigen::Vector3d& acc_meas,
                           const Eigen::Vector3d& gyro_meas);

    /**
    * Set feet contact states
    * @param[in] lf_contact left foot contact state [0, 1]
    * @param[in] rf_contact right foot contact state [0, 1]
    * @return True in case of success, false otherwise.
    */
    bool setContacts(const bool& lf_contact, const bool& rf_contact);

    /**
    * Set kinematic measurements
    * @note it is assumed that the order of the joints loaded in the model and the order of the measurements in these vectors match
    * @param[in] encoders joint positions measured through encoders
    * @param[in] encoder_speeds joint velocities measured through encoders
    * @return True in case of success, false otherwise.
    */
    bool setKinematics(const Eigen::VectorXd& encoders,
                       const Eigen::VectorXd& encoder_speeds);

    /**
    * Compute one step of the estimator
    * @return True in case of success, false otherwise.
    */
    virtual bool advance() final;

    /**
    * Get estimator outputs
    * @return A struct containing he estimated internal states of the estiamtor and the associated covariance matrix
    */
    virtual const FBEOutput& get() const final;

    /**
    * Determines the validity of the object retrieved with get()
    * @return True in case of success, false otherwise.
    */
    virtual bool isValid() const final { return (m_estimator_state == State::Running); };

    /**
    * Get ModelComputations object by reference
    * @return ModelComputations object providing information between considered model related quantities in the estimator
    * like the base link, IMU, feet contact frames.
    */
    ModelComputations& modelComputations();

protected:
    /**
    *
    * @param[in] handler p_handler:...
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
    virtual bool predictState(const FBEMeasurements& meas,
                              const double& dt) { return true; };

    /**
    * Update the predicted state estimates using kinematics and contact measurements
    * @param[in] meas measurements to update the predicted states
    * @param[in] dt sampling period in seconds
    * @return True in case of success, false otherwise.
    */
    virtual bool updateKinematics(const FBEMeasurements& meas,
                                  const double& dt) { return true; };

   /**
   * Update the states and the associated covariance using the Kalman gain and the innovations (in an EKF based implementation)
   * @note this function should be called from within updateKinematics in an EKF base implementation, otherwise left unused
   * @param[in] deltaY innovation vector
   * @param[in] H discrete-time measurement model Jacobian
   * @param[in] R discrete-time measurement noise covariance matrix
   * @return bool
   */
   virtual bool updateStates(const Eigen::VectorXd& deltaY,
                             const Eigen::MatrixXd& H,
                             const Eigen::MatrixXd& R) { return true; };

    ModelComputations m_model_comp; /**< Model computations object */
    FBEOptions m_options; /**< Struct holding estimator options */
    FBEMeasurements m_meas; /**< Struct holding the latest measurements that were set to the estimator */
    FBEInternalState m_state, m_state_prev; /**< Structs holding the curent and previous internal states of the estimation algorithm */
    FBEOutput m_estimator_out; /**< Struct holding outputs of the estimator */
    FBEPriorsStdDev m_priors; /**< Struct holding standard deviations associated to prior state estimates */
    FBESensorsStdDev m_sensors_dev; /**< Struct holding standard deviations associated to sensor measurements */

    /**
    * Enumerator used to determine the running state of the estimator
    */
    enum class State
    {
        NotInitialized, /**< The estimator is not initialized yet call RecursiveLeastSquare::initialze
                           method to initialize it*/
        Initialized,    /**< The estimator is initialized and ready to be used */
        Running         /**< The estimator is running */
    };

    State m_estimator_state{State::NotInitialized}; /**< State of the estimator */

    double m_dt{0.01}; /**< Fixed time step of the estimator, in seconds */

private:
    /**
    * Setup model related parameters
    *
    * @param[in] handler parameter handler
    * @return True in case of success, false otherwise.
    */
    bool setupModelParams(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

};


} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_FBE_H

