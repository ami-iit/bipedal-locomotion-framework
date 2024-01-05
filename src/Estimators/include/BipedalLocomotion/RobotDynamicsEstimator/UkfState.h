/**
 * @file UkfState.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_STATE_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_STATE_H

#include <memory>
#include <string>
#include <Eigen/Dense>

// BayesFilters
#include <BayesFilters/AdditiveStateModel.h>

// BLF
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * UkfState is a concrete class that represents the State of the estimator.
 * The user should build the dynamic model of the state setting a variable handler
 * describing the variables composing the state, the list of the dynamic model associated
 * to each variable in the variable handler, and the matrix of covariances
 * associated to the state. The user should set also a ukf input provider
 * which provides the inputs needed to update the ukf process dynamics.
 */
class UkfState : public bfl::AdditiveStateModel, UkfModel
{
    Eigen::MatrixXd m_covarianceQ; /**< Covariance matrix. */
    Eigen::MatrixXd  m_initialCovariance; /**< Initial covariance matrix. */
    std::size_t m_stateSize; /**< Length of the state vector. */
    Eigen::VectorXd m_nextState; /**< Vector containing all the updated states. */

public:
    /**
     * Build the ukf state model
     * @param kinDyn a pointer to an iDynTree::KinDynComputations object that will be shared among
     * all the dynamics.
     * @param subModelList a vector of pairs (SubModel, KinDynWrapper) that will be shared among all the dynamics.
     * @param handler pointer to the IParametersHandler interface.
     * @note the following parameters are required by the class
     * |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |              `dT`              |     `double`    |                                      Sampling time.                                            |    Yes    |
     * |         `dynamics_list`        |`vector<string>` |                          List of dynamics composing the state model.                           |    Yes    |
     * For **each** dynamics listed in the parameter `dynamics_list` the user must specified all the parameters
     * required by the dynamics itself but `dT` since is already specified in the parent group.
     * Moreover the following parameters are required for each dynamics.
     * |     Group     |         Parameter Name         |         Type         |                                           Description                                          | Mandatory |
     * |:-------------:|:------------------------------:|:--------------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |`DYNAMICS_NAME`|             `name`             |        `string`      |                               String representing the name of the dynamics.                    |    Yes    |
     * |`DYNAMICS_NAME`|           `elements`           |`std::vector<string>` |  Vector of strings representing the elements composing the specific dynamics.                  |    No     |
     * |`DYNAMICS_NAME`|          `covariance`          |`std::vector<double>` |             Vector of double containing the covariance associated to each element.             |    Yes    |
     * |`DYNAMICS_NAME`|         `dynamic_model`        |        `string`      |  String representing the type of dynamics. The string should match the name of the C++ class.  |    Yes    |
     * |`DYNAMICS_NAME`|          `friction_k0`         |`std::vector<double>` | Vector of double containing the coefficient k0 of the friction model of each element.          |    No     |
     * |`DYNAMICS_NAME`|          `friction_k1`         |`std::vector<double>` | Vector of double containing the coefficient k1 of the friction model of each element.          |    No     |
     * |`DYNAMICS_NAME`|          `friction_k2`         |`std::vector<double>` | Vector of double containing the coefficient k2 of the friction model of each element.          |    No     |
     * `DYNAMICS_NAME` is a placeholder for the name of the dynamics contained in the `dynamics_list` list.
     * `name` can contain only the following values ("ds", "tau_m", "tau_F", "*_ft_sensor", "*_ft_sensor_bias", "*_ft_acc_bias", "*_ftgyro_bias").
     * @note The following `ini` file presents an example of the configuration that can be used to
     * build the UkfState.
     *
     * \code{.ini}
     * # UkfState.ini
     *
     * dynamics_list                   ("JOINT_VELOCITY", "FRICTION_TORQUE", "RIGHT_LEG_FT", "RIGHT_FOOT_REAR_GYRO_BIAS")
     *
     * [JOINT_VELOCITY]
     * name                            "ds"
     * elements                        ("r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll")
     * covariance                      (1e-3, 1e-3, 1e-3, 1e-3, 1e-3, 1e-3)
     * dynamic_model                   "JointVelocityDynamics"
     *
     * [FRICTION_TORQUE]
     * name                            "tau_F"
     * elements                        ("r_hip_pitch", "r_hip_roll", "r_hip_yaw", "r_knee", "r_ankle_pitch", "r_ankle_roll")
     * covariance                      (1e-2, 1e-2, 1e-2, 1e-2, 1e-2, 1e-1)
     * dynamic_model                   "FrictionTorqueDynamics"
     * friction_k0                     (9.106, 5.03, 4.93, 12.88, 14.34, 1.12)
     * friction_k1                     (200.0, 6.9, 200.0, 59.87, 200.0, 200.0)
     * friction_k2                     (1.767, 5.64, 0.27, 2.0, 3.0, 0.0)
     *
     * [RIGHT_LEG_FT]
     * name                            "r_leg_ft_sensor"
     * elements                        ("fx", "fy", "fz", "mx", "my", "mz")
     * covariance                      (1e-3, 1e-3, 1e-3, 1e-4, 1e-4, 1e-4)
     * dynamic_model                   "ZeroVelocityStateDynamics"
     *
     * [RIGHT_FOOT_REAR_GYRO_BIAS]
     * name                            "r_foot_rear_ft_gyro_bias"
     * elements                        ("x", "y", "z")
     * covariance                      (8.2e-8, 1e-2, 9.3e-3)
     * dynamic_model                   "ZeroVelocityStateDynamics"
     *
     * \endcode
     * @return a std::unique_ptr to the UkfState.
     * In case of issues, an empty BipedalLocomotion::System::VariablesHandler
     * and an invalid pointer will be returned.
     */
    static std::unique_ptr<UkfState> build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                           std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                                           const std::vector<SubModel>& subModelList,
                                           const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList);

    /**
     * Initialize the ukf state model.
     * @param handler pointer to the IParametersHandler interface.
     * @note the following parameters are required by the class
     * |         Parameter Name         |              Type                 |                             Description                                       | Mandatory |
     * |:------------------------------:|:---------------------------------:|:-----------------------------------------------------------------------------:|:---------:|
     * |       `sampling_time`          |     `std::chrono::nanoseconds`    |                            Sampling time.                                     |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Finalize the UkfState.
     * @param handler variable handler.
     * @note You should call this method after you initialize the UkfState.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& handler);

    /**
     * @brief setUkfInputProvider set the provider for the ukf input.
     * @param ukfInputProvider a pointer to a structure containing the joint positions and the robot base pose, velocity and acceleration.
     */
    void setUkfInputProvider(std::shared_ptr<const UkfInputProvider> ukfInputProvider);

    /**
     * @brief getStateVariableHandler access the `System::VariablesHandler` instance created during the initialization phase.
     * @return the state variable handler containing all the state variables and their sizes and offsets.
     */
    const System::VariablesHandler& getStateVariableHandler() const;

    /**
     * @brief propagate implements the prediction phase of the ukf
     * @param cur_states is the state computed at the previous step
     * @param prop_states is the predicted state
     */
    void propagate(const Eigen::Ref<const Eigen::MatrixXd>& currentStates, Eigen::Ref<Eigen::MatrixXd> propagatedStates) override;

    /**
     * @brief getNoiseCovarianceMatrix access the `Eigen::MatrixXd` representing the process covariance matrix.
     * @return the process noise covariance matrix.
     */
    Eigen::MatrixXd getNoiseCovarianceMatrix() override;

    /**
     * @brief setProperty is not implemented.
     * @param property is a string.
     * @return false as it is not implemented.
     */
    bool setProperty(const std::string& property) override;

    /**
     * @brief getStateDescription access the `bfl::VectorDescription`.
     * @return the state vector description.
     */
    bfl::VectorDescription getStateDescription() override;

    /**
     * @brief getStateSize access the length of the state vector.
     * @return the length of state vector.
     */
    std::size_t getStateSize();

    /**
     * @brief getInitialStateCovarianceMatrix access the `Eigen::MatrixXd` representing the initial state covariance matrix.
     * @return a Eigen reference to the Eigen Matrix covariance.
     */
    Eigen::Ref<const Eigen::MatrixXd> getInitialStateCovarianceMatrix() const;
}; // class UKFModel

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_STATE_H
