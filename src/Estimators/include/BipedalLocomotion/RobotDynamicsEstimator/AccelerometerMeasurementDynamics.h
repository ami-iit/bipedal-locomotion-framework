/**
 * @file AccelerometerMeasurementDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_ACCELEROMETER_MEASUREMENT_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_ACCELEROMETER_MEASUREMENT_DYNAMICS_H

#include <memory>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelDynamics.h>

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
 * \dot{v}^{accelerometer} = \dot{J} \nu + J \dot{\nu} = \dot{J} \nu + J^{base} \dot{v}^{base} + J^{joints} \ddot{s}
 * \f]
 * where the joint acceleration is given by the forward dynamics equation.
 */

class AccelerometerMeasurementDynamics : public Dynamics
{
    bool m_useBias{false}; /**< If true the dynamics depends on a bias additively. */
    Eigen::VectorXd m_bias; /**< The bias is initialized and used only if m_useBias is true. False if not specified. */
    std::string m_biasVariableName; /**< Name of the variable containing the bias in the variable handler. */
    std::vector<SubModel> m_subModelList; /** List of SubModel objects. */
    std::vector<std::shared_ptr<SubModelKinDynWrapper>> m_kinDynWrapperList; /**< List of pointers to SubModelKinDynWrapper objects. */
    bool m_isSubModelListSet{false}; /**< Boolean flag saying if the sub-model list has been set. */
    double m_dT; /**< Sampling time. */
    std::vector<Eigen::VectorXd> m_subModelJointAcc; /**< Updated joint acceleration of each sub-model. */
    Eigen::Vector3d m_gravity; /**< Gravitational acceleration. */
    std::vector<std::size_t> m_subModelsWithAccelerometer; /**< List of indeces saying which sub-model in the m_subDynamics list containa the accelerometer. */
    manif::SE3d::Tangent m_accFrameVel; /**< Velocity at the accelerometer frame. */

protected:
    Eigen::VectorXd m_covSingleVar;
    Eigen::VectorXd m_JdotNu;
    Eigen::VectorXd m_JvdotBase;
    Eigen::VectorXd m_Jsdotdot;
    Eigen::Vector3d m_accRg;
    Eigen::Vector3d m_vCrossW;
    Eigen::Vector3d m_linVel;
    Eigen::Vector3d m_angVel;

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
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |               `name`               | `string` |   Name of the state contained in the `VariablesHandler` describing the state associated to this dynamics|    Yes    |
     * |            `covariance`            | `vector` |                                Process covariances                                                      |    Yes    |
     * |           `dynamic_model`          | `string` |               Type of dynamic model describing the state dynamics.                                      |    Yes    |
     * |             `elements`             | `vector` |  Vector of strings describing the list of sub variables composing the state associated to this dynamics.|    No     |
     * |             `use_bias`             |`boolean` |     Boolean saying if the dynamics depends on a bias. False if not specified.                           |    No     |
     * |                `dT`                | `double` |                                Sampling time.                                                           |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler) override;

    /**
     * Finalize the Dynamics.
     * @param stateVariableHandler object describing the variables in the state vector.
     * @note You should call this method after you add ALL the state dynamics to the state variable handler.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& stateVariableHandler) override;

    /**
     * Set the SubModelKinDynWrapper object.
     * @param kinDynWrapper pointer to a SubModelKinDynWrapper object.
     * @return True in case of success, false otherwise.
     */
    bool setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList) override;

    /**
      * Controls whether the variable handler contains the variables on which the dynamics depend.
      * @return True in case all the dependencies are contained in the variable handler, false otherwise.
      */
    bool checkStateVariableHandler() override;

    /**
     * Update the content of the element.
     * @return True in case of success, false otherwise.
     */
    bool update() override;

    /**
     * Set the state of the ukf needed to update the dynamics.
     * @param ukfState reference to the ukf state.
     */
     void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

     /**
      * Set a `UKFInput` object.
      * @param ukfInput reference to the UKFInput struct.
      */
      void setInput(const UKFInput & ukfInput) override;

}; // class AccelerometerMeasurementDynamics

BLF_REGISTER_DYNAMICS(AccelerometerMeasurementDynamics, ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_ACCELEROMETER_MEASUREMENT_DYNAMICS_H
