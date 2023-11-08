/**
 * @file UkfMeasurement.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_MEASUREMENT_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_MEASUREMENT_H

#include <memory>
#include <string>
#include <Eigen/Dense>

// BayesFilters
#include <BayesFilters/AdditiveMeasurementModel.h>

// BLF
#include <BipedalLocomotion/RobotDynamicsEstimator/UkfModel.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/System/Source.h>
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
 * UkfMeasurement is a concrete class that represents the Measurement of the estimator.
 * The user should build the dynamic model of the measurement, setting a variable handler
 * describing the variables composing the measurement vector, the list of the dynamic models
 * associated to each variable, and the matrix of covariances associated to the variable in the
 * measurement vector. The user should set also a ukf input provider
 * which provides the inputs needed to update the ukf measurement dynamics.
 */

class UkfMeasurement : public bfl::AdditiveMeasurementModel, UkfModel
{
    /**
     * Private implementation
     */
    bfl::VectorDescription m_measurementDescription;
    bfl::VectorDescription m_inputDescription;
    Eigen::MatrixXd m_covarianceR; /**< Covariance matrix. */
    int m_measurementSize{0}; /**< Length of the measurement vector. */
    System::VariablesHandler m_measurementVariableHandler; /**< Variable handler describing the
                                                            measurement vector. */
    Eigen::VectorXd m_tempPredictedMeas;
    Eigen::MatrixXd m_predictedMeasurement; /**< Vector containing the updated measurement. */

public:
    /**
     * Build the ukf measurement model
     * @param kinDyn a pointer to an iDynTree::KinDynComputations object that will be shared among
     * all the dynamics.
     * @param subModelList a vector of SubModel objects.
     * @param kinDynWrapperList a vector of pointers to KinDynWrapper objects
     * @param handler pointer to the IParametersHandler interface.
     * @param stateVariableHandler a variable handler describing the variables in the state vector of the ukf.
     * @note the following parameters are required by the class
     * |         Parameter Name         |       Type      |                                           Description                                          | Mandatory |
     * |:------------------------------:|:---------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |         `sampling_time`        |     `double`    |                                      Sampling time.                                            |    Yes    |
     * |         `dynamics_list`        |`vector<string>` |                          List of dynamics composing the measurement model.                           |    Yes    |
     * For **each** dynamics listed in the parameter `dynamics_list` the user must specified all the parameters
     * required by the dynamics itself but `dT` since is already specified in the parent group.
     * Moreover the following parameters are required for each dynamics.
     * |     Group     |         Parameter Name         |         Type         |                                           Description                                          | Mandatory |
     * |:-------------:|:------------------------------:|:--------------------:|:----------------------------------------------------------------------------------------------:|:---------:|
     * |`DYNAMICS_NAME`|             `name`             |        `string`      |                               String representing the name of the dynamics.                    |    Yes    |
     * |`DYNAMICS_NAME`|           `elements`           |`std::vector<string>` |  Vector of strings representing the elements composing the specific dynamics.                  |    No     |
     * |`DYNAMICS_NAME`|          `covariance`          |`std::vector<double>` |             Vector of double containing the covariance associated to each element.             |    Yes    |
     * |`DYNAMICS_NAME`|         `dynamic_model`        |        `string`      |  String representing the type of dynamics. The string should match the name of the C++ class.  |    Yes    |
     * |`DYNAMICS_NAME`|            `use_bias`          |       `boolean`      |               Boolean saying if an additive bias must be used in the dynamic model.            |    No     |
     * |`DYNAMICS_NAME`|            `use_bias`          |       `boolean`      |               Boolean saying if an additive bias must be used in the dynamic model.            |    No     |
     * `DYNAMICS_NAME` is a placeholder for the name of the dynamics contained in the `dynamics_list` list.
     * `name` can contain only the following values ("ds", "i_m", "*_ft_sensor", "*_ft_acc", "*_ft_gyro").
     * @note The following `ini` file presents an example of the configuration that can be used to
     * build the UkfMeasurement.
     *
     * \code{.ini}
     * # UkfMeasurement.ini
     *
     * dynamics_list                   ("JOINT_VELOCITY", "MOTOR_CURRENT", "RIGHT_LEG_FT", "RIGHT_FOOT_REAR_GYRO")
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
     * dynamic_model                   "ConstantMeasurementModel"
     *
     * [RIGHT_FOOT_REAR_GYRO_BIAS]
     * name                            "r_foot_rear_ft_gyro_bias"
     * elements                        ("x", "y", "z")
     * covariance                      (8.2e-8, 1e-2, 9.3e-3)
     * dynamic_model                   "ConstantMeasurementModel"
     *
     * \endcode
     * @return a std::unique_ptr to the UkfMeasurement.
     * In case of issues, an empty BipedalLocomotion::System::VariablesHandler
     * and an invalid pointer will be returned.
     */
    static std::unique_ptr<UkfMeasurement> build(std::weak_ptr<const ParametersHandler::IParametersHandler> handler,
                                                 System::VariablesHandler& stateVariableHandler,
                                                 std::shared_ptr<iDynTree::KinDynComputations> kinDynFullModel,
                                                 const std::vector<SubModel>& subModelList,
                                                 const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList);

    /**
     * Initialize the ukf measurement model.
     * @param handler pointer to the IParametersHandler interface.
     * @note the following parameters are required by the class
     * |         Parameter Name         |              Type                 |                             Description                                       | Mandatory |
     * |:------------------------------:|:---------------------------------:|:-----------------------------------------------------------------------------:|:---------:|
     * |       `sampling_time`          |     `std::chrono::nanoseconds`    |                            Sampling time.                                     |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Finalize the UkfMeasurement.
     * @param handler variable handler.
     * @note You should call this method after you initialize the UkfMeasurement.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& handler);

    /**
     * @brief setUkfInputProvider set the provider for the ukf input.
     * @param ukfInputProvider a pointer to a structure containing the joint positions and the robot base pose, velocity and acceleration.
     */
    void setUkfInputProvider(std::shared_ptr<const UkfInputProvider> ukfInputProvider);

    /**
     * @brief getMeasurementVariableHandler access the `System::VariablesHandler` instance created during the initialization phase.
     * @return the measurement variable handler containing all the measurement variables and their sizes and offsets.
     */
    const System::VariablesHandler& getMeasurementVariableHandler() const;

    /**
     * @brief predictedMeasure predict the new measurement depending on the state computed by the predict step.
     * @param cur_states is the state computed by the prediction phase.
     * @return a std::pair<bool, bfl::Data> where the bool value says if the measurement
     * prediciton is done correctly and the bfl::Data is the predicted measure.
     */
    std::pair<bool, bfl::Data> predictedMeasure(const Eigen::Ref<const Eigen::MatrixXd>& currentState) const override;

    /**
     * @brief getNoiseCovarianceMatrix access the `Eigen::MatrixXd` representing the process covariance matrix.
     * @return a boolean value and the measurement noise covariance matrix.
     */
    std::pair<bool, Eigen::MatrixXd> getNoiseCovarianceMatrix() const override;

    /**
     * @brief setProperty is not implemented.
     * @param property is a string.
     * @return false as it is not implemented.
     */
    bool setProperty(const std::string& property) override;

    /**
     * @brief getMeasurementDescription access the `bfl::VectorDescription`.
     * @return the measurement vector description.
     */
    bfl::VectorDescription getMeasurementDescription() const override;

    /**
     * @brief getInputDescription access the `bfl::VectorDescription`.
     * @return the input vector description.
     */
    bfl::VectorDescription getInputDescription() const override;

    /**
     * @brief getMeasurementSize access the length of the measurement vector
     * @return the length of measurement vector
     */
    std::size_t getMeasurementSize();

    /**
     * Set a `System::VariableHandler` describing the variables composing the state.
     * @param stateVariableHandler is the variable handler
     */
    void setStateVariableHandler(System::VariablesHandler stateVariableHandler);

    /**
     * @brief innovation computes the innovation step of the ukf update as the difference between the predicted_measurement
     * and the measurement.
     * @param predicted_measurements is a `blf::Data` reference representing the measurement predicted in the update step.
     * @param measurements is a `blf::Data` reference representing the measurements coming from the user.
     * @return a `std::pair<bool, bfl::Data>` where the boolean value is always true and the `bfl::Data` is the innovation term.
     */
    std::pair<bool, bfl::Data> innovation(const bfl::Data& predictedMeasurements, const bfl::Data& measurements) const override;

    /**
     * @brief measure get the updated measurement.
     * @param data is a `const blf::Data` reference and is optional parameter.
     * @return a pair of a boolean value which is always true and the measurements.
     */
    std::pair<bool, bfl::Data> measure(const bfl::Data& data = bfl::Data()) const override;

    /**
     * @brief freeze update the measurement using data from sensors.
     * @param data is a generic object representing data coming from sensors.
     * @return true.
     * @note data in this case must be a `std::map<std::string, Eigen::VectorXd>` where
     * the first element represents the name of each measurement dynamics
     * and the second element is the vector containing the measurement
     * of the sensor associated to that variable.
     */
    bool freeze(const bfl::Data& data = bfl::Data()) override;

}; // classUkfMeasurement

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_UKF_MEASUREMENT_H
