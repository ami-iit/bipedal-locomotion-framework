/**
 * @file FixedBaseDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_BASE_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_BASE_DYNAMICS_H

#include <chrono>
#include <memory>
#include <tuple>
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
class FixedBaseDynamics;
}
} // namespace BipedalLocomotion

namespace BipedalLocomotion::ContinuousDynamicalSystem::internal
{
template <> struct traits<FixedBaseDynamics>
{
    using State = GenericContainer::named_tuple<BLF_NAMED_PARAM(s, Eigen::VectorXd),
                                      BLF_NAMED_PARAM(ds, Eigen::VectorXd)>;
    using StateDerivative = GenericContainer::named_tuple<BLF_NAMED_PARAM(ds, Eigen::VectorXd),
                                                BLF_NAMED_PARAM(dds, Eigen::VectorXd)>;
    using Input = GenericContainer::named_tuple<BLF_NAMED_PARAM(tau, Eigen::VectorXd)>;
    using DynamicalSystem = FixedBaseDynamics;
};
} // namespace BipedalLocomotion::ContinuousDynamicalSystem::internal


namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * FixedBaseDynamics describes a the dynamics of a fixed base system
 * The FixedBaseDynamics inherits from a generic DynamicalSystem where the State is
 * described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |         Description         |
 * |:----:|:-----------------:|:---------------------------:|
 * |  `s` | `Eigen::VectorXd` |  Joint positions [in rad]  |
 * | `ds` | `Eigen::VectorXd` | Joint velocities [in rad/s] |
 *
 * The `StateDerivative` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |            Description           |
 * |:----:|:-----------------:|:--------------------------------:|
 * | `ds` | `Eigen::VectorXd` |     Joint velocities [in rad]    |
 * | dds` | `Eigen::VectorXd` | Joint accelerations [in rad/s^2] |
 *
 * The `Input` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |            Description           |
 * |:----:|:-----------------:|:--------------------------------:|
 * |`tau` | `Eigen::VectorXd` |      Joint torque [in N/m]       |
 */
class FixedBaseDynamics : public DynamicalSystem<FixedBaseDynamics>
{
    iDynTree::KinDynComputations m_kinDyn; /**< kinDynComputations object */
    std::size_t m_actuatedDoFs{0}; /**< Number of actuated degree of freedom */

    Eigen::Vector3d m_gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector
                                                                                */

    Eigen::MatrixXd m_massMatrix; /**< Floating-base mass matrix  */
    std::string m_robotBase; /**< Name of the frame associated to the robot base */

    // quantities useful to avoid dynamic allocation in the dynamic allocation in the
    // FixedBaseDynamics::dynamics() method
    Eigen::VectorXd m_knownCoefficent;

    bool m_useMassMatrixRegularizationTerm{false};
    Eigen::MatrixXd m_massMatrixReglarizationTerm;

    State m_state;
    Input m_controlInput;

public:

    /**
     * Initialize the FixedBaseDynamics system.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are used
     * | Parameter Name |   Type   |                                          Description                                         | Mandatory |
     * |:--------------:|:--------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |    `gravity`   | `double` |     Value of the Gravity. If not defined Math::StandardAccelerationOfGravitation is used     |    No     |
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
     * is the mass matrix and \f$M_{reg}\f$ is the matrix regularization term.
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

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_FIXED_BASE_DYNAMICS_H
