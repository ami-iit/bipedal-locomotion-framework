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
#include <chrono>

#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/Dynamics.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/SubModel.h>
#include <BipedalLocomotion/RobotDynamicsEstimator/KinDynWrapper.h>

namespace BipedalLocomotion
{
namespace Estimators
{
namespace RobotDynamicsEstimator
{

/**
 * The JointVelocityDynamics class is a concrete implementation of the Dynamics.
 * Please use this element if you want to use the robot dynamic model to update the joint dynamics.
 * The JointVelocityDynamics represents the following equation in the continuous time:
 * \f[
 * \ddot{s} = H^{-1} [\tau_{m} - \tau_{F} + (\sum J^T_{FT} f_{FT}) +
 * + (\sum J^T_{ext} f_{ext})   - F^T {}^B \dot v  - h ]
 * \f]
 * where \ddot{s} is given as an input to this class.
 * The discretized model becomes:
 * \f[
 * \dot{s}_{k+1} = \dot{s}_{k} + \Delta T \ddot{s}_{k}
 * \f]
 */
class JointVelocityStateDynamics : public Dynamics
{
    std::chrono::nanoseconds m_dT{std::chrono::nanoseconds::zero()}; /**< Sampling time. */
    Eigen::VectorXd m_jointVelocityFullModel; /**< Joint velocities of full-model. */
    UKFInput m_ukfInput; /**< Input of the UKF used to update the dynamics. */
    std::string m_name; /**< Name of dynamics. */
    std::vector<std::string> m_elements = {}; /**< Elements composing the variable vector. */
    System::VariablesHandler m_stateVariableHandler; /**< Variable handler describing the variables and the sizes in the ukf state vector. */

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
     * Set the KinDynWrapper object.
     * @param subModelList list of SubModel objects
     * @param kinDynWrapperList list of pointers to KinDynWrapper objects.
     * @return True in case of success, false otherwise.
     */
    bool setSubModels(const std::vector<SubModel>& subModelList, const std::vector<std::shared_ptr<KinDynWrapper>>& kinDynWrapperList) override;

    /**
     * Initialize the state dynamics.
     * @param paramHandler pointer to the parameters handler.
     * @param name name of the dynamics.
     * @note the following parameters are required by the class
     * |           Parameter Name           |         Type          |                                       Description                                                       | Mandatory |
     * |:----------------------------------:|:---------------------:|:-------------------------------------------------------------------------------------------------------:|:---------:|
     * |            `input_name`            |       `string`        |        Name of the variable in input to the main application corresponding to this dynamics             |    Yes    |
     * |            `covariance`            |       `vector`        |                                Process covariances                                                      |    Yes    |
     * |         `initial_covariance`       |       `vector`        |                             Initial state covariances                                                   |    Yes    |
     * |           `dynamic_model`          |       `string`        |               Type of dynamic model describing the state dynamics.                                      |    Yes    |
     * |                `dT`                | `chrono::nanoseconds` |                                Sampling time.                                                           |    Yes    |
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> paramHandler, const std::string& name) override;

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
     * Set the state of the ukf needed to update the dynamics of the state variable associated to ths object.
     * @param ukfState reference to the ukf state.
     */
     void setState(const Eigen::Ref<const Eigen::VectorXd> ukfState) override;

     /**
      * Set a `UKFInput` object.
      * @param ukfInput reference to the UKFInput struct.
      */
      void setInput(const UKFInput & ukfInput) override;
};

BLF_REGISTER_UKF_DYNAMICS(JointVelocityStateDynamics, ::BipedalLocomotion::Estimators::RobotDynamicsEstimator::Dynamics);

} // namespace RobotDynamicsEstimator
} // namespace Estimators
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ESTIMATORS_JOINT_VELOCITY_STATE_DYNAMICS_H
