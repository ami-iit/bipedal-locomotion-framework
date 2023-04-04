/**
 * @file FloatingBaseDynamicsWithCompliantContacts.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_DYNAMICS_WITH_COMPLIANT_CONTACTS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_DYNAMICS_WITH_COMPLIANT_CONTACTS_H

#include <chrono>
#include <memory>
#include <vector>

#include <BipedalLocomotion/ContinuousDynamicalSystem/CompliantContactWrench.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/impl/traits.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <Eigen/Dense>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
class FloatingBaseDynamicsWithCompliantContacts;
}
}

// Please read it as
// BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(
//     FloatingBaseDynamicsWithCompliantContacts
//     (base velocity expressed in mixed, joint velocities, base position, base orientation, joint positions),
//     (base acceleration expressed in mixed, joint acceleration, base linear velocity, rate of change of the base rotation matrix, joint velocities),
//     (joint torques, list containing the compliant contact models)

BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(FloatingBaseDynamicsWithCompliantContacts,
                                                         (Eigen::Matrix<double, 6, 1>,
                                                          Eigen::VectorXd,
                                                          Eigen::Vector3d,
                                                          Eigen::Matrix3d,
                                                          Eigen::VectorXd),
                                                         (Eigen::Matrix<double, 6, 1>,
                                                          Eigen::VectorXd,
                                                          Eigen::Vector3d,
                                                          Eigen::Matrix3d,
                                                          Eigen::VectorXd),
                                                         (Eigen::VectorXd,
                                                          std::vector<CompliantContactWrench>));

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
/**
 * FloatingBaseDynamicalSystem describes a floating base dynamical system.
 * The FloatingBaseDynamicalSystem inherits from a generic DynamicalSystem where:
 * - DynamicalSystem::StateType is described by an std::tuple containing:
 *   - Eigen::Vector6d: the base velocity expressed in mixed representation;
 *   - Eigen::VectorXd: the joint velocities [in rad/s];
 *   - Eigen::Vector3d: position of the base w.r.t. the inertial frame
 *   - Eigen::Matrix3d: rotation matrix \f${} ^ I R _ {b}\f$. Matrix that transform a vector
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - Eigen::VectorXd: the joint positions [in rad].
 * - DynamicalSystem::StateDerivativeType is described by an std::tuple containing:
 *   - Eigen::Vector6d: the base acceleration expressed in mixed representation;
 *   - Eigen::VectorXd: the joint accelerations [in rad/s^2];
 *   - Eigen::Vector3d: base velocity w.r.t. the inertial frame;
 *   - Eigen::Matrix3d: rate of change of the rotation matrix \f${} ^ I \dot{R} _ {b}\f$.
 * whose coordinates are expressed in the base frame in the inertial frame;
 *   - Eigen::VectorXd: the joint velocities [in rad/s].
 * - DynamicalSystem::InputType is described by an std::tuple containing:
 *   - Eigen::VectorXd: the joint torques [in Nm];
 *   - std::vector<CompliantContactWrench>: List of contact wrenches.
 */
class FloatingBaseDynamicsWithCompliantContacts
    : public DynamicalSystem<FloatingBaseDynamicsWithCompliantContacts>
{
    static constexpr size_t m_baseDoFs = 6; /**< Number of degree of freedom associated to the
                                               floating base */

    iDynTree::KinDynComputations m_kinDyn; /**< kinDynComputations object */
    std::size_t m_actuatedDoFs{0}; /**< Number of actuated degree of freedom */

    Eigen::Vector3d m_gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector
                                                                                */

    std::string m_robotBase; /**< Name of the frame associated to the robot base */

    Eigen::MatrixXd m_massMatrix; /**< Floating-base mass matrix  */

    Eigen::MatrixXd m_jacobianMatrix; /**< Jacobian Matrix  */

    // quantities useful to avoid dynamic allocation in the dynamic allocation in the
    // FloatingBaseDynamicsWithCompliantContacts::dynamics method
    Eigen::VectorXd m_generalizedRobotAcceleration;
    Eigen::VectorXd m_knownCoefficent;

    bool m_useMassMatrixRegularizationTerm{false};
    Eigen::MatrixXd m_massMatrixReglarizationTerm;

    double m_rho{0.01}; /**< Regularization term used for the Baumgarte stabilization over the SO(3)
                           group */

    State m_state;
    Input m_controlInput;

public:
    /**
     * Initialize the FloatingBaseDynamicsWithCompliantContacts system.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are used
     * | Parameter Name |   Type   |                                          Description                                         | Mandatory |
     * |:--------------:|:--------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |    `gravity`   | `double` |     Value of the Gravity. If not defined Math::StandardAccelerationOfGravitation is used     |    No     |
     * |      `rho`     | `double` |       Baumgarte stabilization parameter over the SO(3) group. The default value is 0.01      |    No     |
     * |  `base_link`  | `string` |  Name of the link considered as fixed base in the model. If not defined the default link will be used. Please check [here](https://robotology.github.io/idyntree/master/classiDynTree_1_1Model.html#a1a8dc1c97b99ffc51dbf93ecff20e8c1)    |    No     |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Set the model of the robot.
     * @param model an iDynTree robot model.
     * @return true in case of success, false otherwise.
     */
    bool setRobotModel(const iDynTree::Model& model);

    /**
     * Set the mass matrix regularization term. i.e. \f$\bar{M} = M + M _ {reg}\f$. Where  \f$M\f$
     * is the mass matrix and  \f$M_{reg}\f$ is the matrix regularization term.
     * @param matrix the regularization term for the mass matrix.
     * @notice Calling this function is not mandatory. Call it only if you want to add a
     * regularization term.
     * @return true in case of success, false otherwise.
     */
    bool setMassMatrixRegularization(const Eigen::Ref<const Eigen::MatrixXd>& matrix);

    /**
     * Computes the floating based system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative);

    /**
     * Set the state of the dynamical system.
     * @param state tuple containing a const reference to the state elements.
     * @return true in case of success, false otherwise.
     */
    bool setState(const State& state);

    /**
     * Get the state to the dynamical system.
     * @return the current state of the dynamical system
     */
    const State& getState() const;

    /**
     * Set the control input to the dynamical system.
     * @param controlInput the value of the control input used to compute the system dynamics.
     * @return true in case of success, false otherwise.
     */
    bool setControlInput(const Input& controlInput);
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FLOATING_BASE_SYSTEM_DYNAMICS_WITH_COMPLIANT_CONTACTS_H
