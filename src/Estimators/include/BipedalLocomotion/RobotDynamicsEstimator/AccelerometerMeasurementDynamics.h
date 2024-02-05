/**
 * @file AccelerometerMeasurementDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_ACCELEROMETER_MEASUREMENT_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_ACCELEROMETER_MEASUREMENT_DYNAMICS_H

#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <memory>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The AccelerometerMeasurementDynamics class is a concrete implementation of the Dynamics.
 * Please use this element if you want to use the model dynamics of an accelerometer defined,
 * using the kinematics, as the time derivative of the frame linear velocity:
 * \f[
 * v = J \nu
 * \f]
 * The AccelerometerMeasurementDynamics represents the following equation in the continuous time:
 * \f[
 * \dot{v}^{accelerometer} = \dot{J} \nu + J \dot{\nu} = \dot{J} \nu + J^{base} \dot{v}^{base} +
 * J^{joints} \ddot{s} \f] where the joint acceleration is given by the forward dynamics equation.
 */
class AccelerometerMeasurementDynamics : public Dynamics
{
    bool m_useBias{false}; /**< If true the dynamics depends on a bias additively. */
    Eigen::VectorXd m_bias; /**< The bias is initialized and used only if m_useBias is true. False
                               if not specified. */
    std::string m_biasVariableName; /**< Name of the variable containing the bias in the variable
                                       handler. */
    std::vector<SubModel> m_subModelList; /** List of SubModel objects. */
    std::vector<std::shared_ptr<KinDynWrapper>> m_kinDynWrapperList; /**< List of pointers
                                                                                to
                                                                                KinDynWrapper
                                                                                objects. */
    std::string m_accelerometerFrameName; /**< Name of the frame associated to the accelerometer in the model. */
    bool m_isSubModelListSet{false}; /**< Boolean flag saying if the sub-model list has been set. */
    std::vector<Eigen::VectorXd> m_subModelJointAcc; /**< Updated joint acceleration of each
                                                        sub-model. */
    Eigen::Vector3d m_gravity; /**< Gravitational acceleration. */
    std::vector<std::size_t> m_subModelsWithAccelerometer; /**< List of indeces saying which
                                                              sub-model in the m_subDynamics list
                                                              containa the accelerometer. */
    UKFInput m_ukfInput; /**< Input of the UKF used to update the dynamics. */
    std::string m_name; /**< Name of dynamics. */
    System::VariablesHandler m_stateVariableHandler; /**< Variable handler describing the variables
                                                        and the sizes in the ukf state vector. */
    Eigen::VectorXd m_covSingleVar; /**< Covariance of the accelerometer measurement from
                                       configuration. */
    manif::SE3d::Tangent m_subModelBaseAcceleration; /**< Base acceleration of the sub-model. */
    manif::SE3d::Tangent m_accelerometerFameVelocity; /** Velocity of the accelerometer given by the forward dynamics. */
    manif::SE3d::Tangent m_accelerometerFameAcceleration; /** Acceleration of the accelerometer given by the forward dynamics. */
    manif::SO3d m_imuRworld; /**< Rotation matrix of the inertial frame with respect to the imu frame expressed in the imu frame. */

    Eigen::VectorXd m_JdotNu; /**< Jdot nu. */
    Eigen::VectorXd m_Jvdot; /**< Jacobian times the base acceleration. */
    Eigen::VectorXd m_Jsdotdot; /**< Jacobian times the joint acceleration. */
    Eigen::Vector3d m_accRg; /**< Gravity rotated in the accelerometer frame. */
    Eigen::Vector3d m_vCrossW; /**< Accelerometer linear velocity cross accelerometer angular
                                  velocity. */

public:
    /*
     * Constructor
     */
    AccelerometerMeasurementDynamics();

    /*
     * Destructor
     */
    virtual ~AccelerometerMeasurementDynamics();

    /**
     * Initialize the state dynamics.
     * @param paramHandler pointer to the parameters handler.
     * @param name name of the dynamics.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |            `input_name`            | `string` |        Name of the variable in input to the main application corresponding to this dynamics             |    Yes    |
     * |            `covariance`            | `vector` |                                Process covariances                                                      |    Yes    |
     * |           `dynamic_model`          | `string` |               Type of dynamic model describing the state dynamics.                                      |    Yes    |
     * |             `use_bias`             |`boolean` |     Boolean saying if the dynamics depends on a bias. False if not specified.                           |    No     |
     * @return True in case of success, false otherwise.
     */
    bool
    initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler, const std::string& name) override;

    /**
     * Finalize the Dynamics.
     * @param stateVariableHandler object describing the variables in the state vector.
     * @note You should call this method after you add ALL the state dynamics to the state variable
     * handler.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& stateVariableHandler) override;

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
     * to ths object.
     * @param ukfState reference to the ukf state.
     */
    void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

    /**
     * Set a `UKFInput` object.
     * @param ukfInput reference to the UKFInput struct.
     */
    void setInput(const UKFInput& ukfInput) override;

}; // class AccelerometerMeasurementDynamics

BLF_REGISTER_UKF_DYNAMICS(AccelerometerMeasurementDynamics,
                          ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_ACCELEROMETER_MEASUREMENT_DYNAMICS_H
