/**
 * @file LinearTimeInvariantSystem.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H

#include <memory>
#include <vector>

#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/impl/traits.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

// Forward declare for type traits specialization

class LinearTimeInvariantSystem;
}
}

namespace BipedalLocomotion::ContinuousDynamicalSystem::internal
{
template <> struct traits<LinearTimeInvariantSystem>
{
    using State = GenericContainer::named_tuple<BLF_NAMED_PARAM(x, Eigen::VectorXd)>;
    using StateDerivative = GenericContainer::named_tuple<BLF_NAMED_PARAM(dx, Eigen::VectorXd)>;
    using Input = GenericContainer::named_tuple<BLF_NAMED_PARAM(u, Eigen::VectorXd)>;
    using DynamicalSystem = LinearTimeInvariantSystem;
};
} // namespace BipedalLocomotion::ContinuousDynamicalSystem::internal

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * LinearTimeInvariantSystem describes a MIMO linear time invariant system of the form \f$\dot{x} =
 * Ax + Bu\f$ where \a x is the state and \a u the control input. The state, its derivative and the
 * control input are described by vectors
 * The LinearTimeInvariantSystem inherits from a generic DynamicalSystem where the State is
 * described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |                     Description                   |
 * |:----:|:-----------------:|:-------------------------------------------------:|
 * |  `x` | `Eigen::VectorXd` | A generic vector belonging to \f$\mathbb{R}^n\f$  |
 *
 * The `StateDerivative` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |                         Description                        |
 * |:----:|:-----------------:|:----------------------------------------------------------:|
 * | `dx` | `Eigen::VectorXd` | A state vector derivative belonging to \f$\mathbb{R}^n\f$  |
 *
 * The `Input` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * | Name |        Type       |                     Description                   |
 * |:----:|:-----------------:|:-------------------------------------------------:|
 * |  `u` | `Eigen::VectorXd` | A control vector belonging to \f$\mathbb{R}^m\f$  |
 */
class LinearTimeInvariantSystem : public DynamicalSystem<LinearTimeInvariantSystem>
{
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_B;

    bool m_isInitialized{false};

    State m_state;
    Input m_controlInput;

public:
    /**
     * Set the system matrices.
     * @param A the A matrix.
     * @param B the B matrix.
     * @return true in case of success, false otherwise.
     */
    bool setSystemMatrices(const Eigen::Ref<const Eigen::MatrixXd>& A,
                           const Eigen::Ref<const Eigen::MatrixXd>& B);

    /**
     * Initialize the Dynamical system.
     * @param handler pointer to the parameter handler.
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

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
     * @note In principle, there is no need to override this method. This value is stored in an
     * internal buffer.
     * @param controlInput the value of the control input used to compute the system dynamics.
     * @return true in case of success, false otherwise.
     */
    bool setControlInput(const Input& controlInput);

    /**
     * Computes the system dynamics. It return \f$Ax + Bu\f$
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const std::chrono::nanoseconds& time, StateDerivative& stateDerivative);
};

} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_LINEAR_TIME_INVARIANT_SYSTEM_H
