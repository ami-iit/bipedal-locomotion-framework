/**
 * @file MotorCurrentMeasurementDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_MOTOR_CURRENT_MEASUREMENT_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_MOTOR_CURRENT_MEASUREMENT_DYNAMICS_H

#include <memory>

// Eigen
#include <Eigen/Dense>

#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The MotorCurrentMeasurementDynamics class is a concrete implementation of the Dynamics.
 * Please use this element to define the measurement of the motor current.
 * The MotorCurrentMeasurementDynamics represents the following equation in the continuous time:
 * \f[
 * \dot{i_{m}} = \tau_{m} / (k_{\tau} r)
 * \f]
 */
class MotorCurrentMeasurementDynamics : public Dynamics
{
    UKFInput m_ukfInput;
    Eigen::VectorXd m_motorTorque; /**< Vector of joint velocities. */
    Eigen::VectorXd m_currentFrictionTorque; /**< Vector of friction torques. */
    Eigen::VectorXd m_kTau; /**< Torque constant. */
    Eigen::VectorXd m_gearRatio; /**< Gearbox reduction ratio. */
    std::string m_name; /**< Name of dynamics. */
    std::vector<std::string> m_elements = {}; /**< Elements composing the variable vector. */
    System::VariablesHandler m_stateVariableHandler; /**< Variable handler describing the variables
                                                        and the sizes in the ukf state vector. */

public:
    /*
     * Constructor
     */
    MotorCurrentMeasurementDynamics();

    /*
     * Destructor
     */
    virtual ~MotorCurrentMeasurementDynamics();

    /**
     * Initialize the measurement object.
     * @param paramHandler pointer to the parameters handler.
     * @param name name of the dynamics.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |            `input_name`            | `string` |        Name of the variable in input to the main application corresponding to this dynamics             |    Yes    |
     * |            `covariance`            | `vector` |                                Process covariances                                                      |    Yes    |
     * |           `dynamic_model`          | `string` |               Type of dynamic model describing the measurement dynamics.                                |    Yes    |
     * |          `torque_constant`         | `vector` |  Vector of coefficients k0. For more info check the class description.                                  |    Yes    |
     * |            `gear_ratio`            | `vector` |  Vector of coefficients k1. For more info check the class description.                                  |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool
    initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler, const std::string& name) override;

    /**
     * Finalize the Dynamics.
     * @param measurementVariableHandler object describing the variables in the measurement vector.
     * @note You should call this method after you add ALL the measurement dynamics to the
     * measurement variable handler.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& measurementVariableHandler) override;

    /**
     * Set the KinDynWrapper object.
     * @param subModelList list of SubModel objects
     * @param kinDynWrapperList list of pointers to KinDynWrapper objects.
     * @return True in case of success, false otherwise.
     */
    bool setSubModels(
        const std::vector<SubModel>& subModelList,
        const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList) override;

    /**
     * Controls whether the variable handler contains the variables on which the dynamics depend.
     * @return True in case all the dependencies are contained in the variable handler, false
     * otherwise.
     */
    bool checkStateVariableHandler() override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the state of the ukf needed to update the dynamics of the measurement variable associated
     * to ths object
     * @param ukfState reference to the ukf state.
     * @return true if the current state has been updated correctly.
     */
    void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

    /**
     * Set a `UKFInput` object.
     * @param ukfInput reference to the UKFInput struct.
     */
    void setInput(const UKFInput& ukfInput) override;
};

BLF_REGISTER_UKF_DYNAMICS(MotorCurrentMeasurementDynamics,
                          ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_MOTOR_CURRENT_MEASUREMENT_DYNAMICS_H
