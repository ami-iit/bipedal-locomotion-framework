/**
 * @file JointVelocityStateDynamics.h
 * @authors Ines Sorrentino
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_ESTIMATORS_JOINT_VELOCITY_STATE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_ESTIMATORS_JOINT_VELOCITY_STATE_DYNAMICS_H

#include <memory>
#include <map>

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelKinDynWrapper.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModelDynamics.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The JointVelocityDynamics class is a concrete implementation of the Dynamics.
 * Please use this element if you want to use the robot dynaic model to update the joint dynamics.
 * The JointVelocityDynamics represents the following equation in the continuous time:
 * \f[
 * \ddot{s} = H^{-1} [\tau_{m} - \tau_{F} + (\sum J^T_{FT} f_{FT}) +
 * + (\sum J^T_{ext} f_{ext})   - F^T {}^B \dot v  - h ]
 * \f]
 */

class JointVelocityStateDynamics : public Dynamics
{
    double m_dT; /**< Sampling time. */
    Eigen::VectorXd m_jointVelocityFullModel; /**< Joint velocities of full-model. */

public:
    /*
     * Constructor
     */
    JointVelocityStateDynamics();

    /*
     * Destructor
     */
    virtual ~JointVelocityStateDynamics();

    /**
     * Set the SubModelKinDynWrapper object.
     * @param kinDynWrapper pointer to a SubModelKinDynWrapper object.
     * @return True in case of success, false otherwise.
     */
    bool setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<SubModelKinDynWrapper>>& kinDynWrapperList) override;

    /**
     * Initialize the state dynamics.
     * @param paramHandler pointer to the parameters handler.
     * @note the following parameters are required by the class
     * |           Parameter Name           |   Type   |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:--------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |               `name`               | `string` |   Name of the state contained in the `VariablesHandler` describing the state associated to this dynamics|    Yes    |
     * |            `covariance`            | `vector` |                                Process covariances                                                      |    Yes    |
     * |         `initial_covariance`       | `vector` |                             Initial state covariances                                                   |    Yes    |
     * |           `dynamic_model`          | `string` |               Type of dynamic model describing the state dynamics.                                      |    Yes    |
     * |             `elements`             | `vector` |  Vector of strings describing the list of sub variables composing the state associated to this dynamics.|    No     |
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
      * Controls whether the variable handler contains the variables on which the dynamics depend.
      * @return True in case all the dependencies are contained in the variable handler, false otherwise.
      */
    bool checkStateVariableHandler() override;

    /**
     * Update the state.
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
};

BLF_REGISTER_DYNAMICS(JointVelocityStateDynamics, ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_JOINT_VELOCITY_STATE_DYNAMICS_H
