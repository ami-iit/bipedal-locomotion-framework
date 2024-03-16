/**
 * @file CentroidalDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_CENROIDAL_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_CENROIDAL_DYNAMICS_H

#include <chrono>
#include <map>
#include <memory>
#include <optional>
#include <string>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/GenericContainer/NamedTuple.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <Eigen/Dense>

namespace BipedalLocomotion::ContinuousDynamicalSystem
{
class CentroidalDynamics;
} // namespace BipedalLocomotion::ContinuousDynamicalSystem

namespace BipedalLocomotion::ContinuousDynamicalSystem::internal
{
template <> struct traits<CentroidalDynamics>
{
    using Contacts = std::map<std::string, //
                              BipedalLocomotion::Contacts::DiscreteGeometryContact>;
    using ExternalWrench = std::optional<BipedalLocomotion::Math::Wrenchd>;

    using State = GenericContainer::named_tuple<BLF_NAMED_PARAM(com_pos, Eigen::Vector3d),
                                                BLF_NAMED_PARAM(com_vel, Eigen::Vector3d),
                                                BLF_NAMED_PARAM(angular_momentum, Eigen::Vector3d)>;
    using StateDerivative
        = GenericContainer::named_tuple<BLF_NAMED_PARAM(com_vel, Eigen::Vector3d),
                                        BLF_NAMED_PARAM(com_acc, Eigen::Vector3d),
                                        BLF_NAMED_PARAM(angular_momentum_rate, Eigen::Vector3d)>;
    using Input = GenericContainer::named_tuple<BLF_NAMED_PARAM(contacts, Contacts),
                                                BLF_NAMED_PARAM(external_wrench, ExternalWrench)>;
    using DynamicalSystem = CentroidalDynamics;
};
} // namespace BipedalLocomotion::ContinuousDynamicalSystem::internal

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

// clang-format off
/**
 * CentroidalDynamics describes the centroidal dynamics of a multi-body system.
 * The centroidal momentum \f${}_{\bar{G}} h ^\top= \begin{bmatrix} {}_{\bar{G}} h^{p\top} & {}_{\bar{G}} h^{\omega\top} \end{bmatrix} \in \mathbb{R}^6\f$
 * is the aggregate linear and angular momentum of each link of the robot referred to the robot
 * center of mass (CoM). The vectors \f${}_{\bar{G}} h^p \in \mathbb{R}^3\f$ and
 * \f${}_{\bar{G}} h^\omega * \in \mathbb{R}^3\f$ are the linear and angular momentum, respectively.
 * The coordinates of \f${}_{\bar{G}} h\f$ are expressed w.r.t. a frame centered in the robot CoM
 * and oriented as the inertial frame \f$\mathcal{I}\f$
 * The CentroidalDynamics class describes the dynamics of the centroidal momentum and of the CoM.
 * Indeed the time derivative of the centroidal momentum depends on the external contact wrenches
 * acting on the system, thus leading to:
 * \f[
 * {}_{\bar{G}} \dot{h} = \sum_{i = 1}^{n_c}
 * \begin{bmatrix}
 * I_3 & 0_3 \
 * (p_{\mathcal{C}_i} - p_{\text{CoM}})^\wedge & I_3
 * \end{bmatrix} \mathrm{f}_i + m\bar{g}
 * \f]
 * where \f$\bar{g}^\top = \begin{bmatrix} g^\top & 0_{1\times3} \end{bmatrix}\f$, \f$m\f$
 * is the mass of the robot and \f$n_c\f$ the number of active contacts.
 * The relation between the linear momentum \f${}_{\bar{G}} h^{p}\f$ and the robot CoM velocity
 * is linear and depends on the robot mass \f$m\f$, i.e. \f${}_{\bar{G}} h^{p} = m v_{\text{CoM}}\f$.
 *
 * The CentroidalDynamics inherits from a generic DynamicalSystem where the State is
 * described by a BipedalLocomotion::GenericContainer::named_tuple
 * |        Name       |        Type       |                                        Description                                        |
 * |:-----------------:|:-----------------:|:-----------------------------------------------------------------------------------------:|
 * |      `com_pos`    | `Eigen::Vector3d` |                   Position of the CoM written in the inertial frame                       |
 * |      `com_vel`    | `Eigen::Vector3d` |                   Velocity of the CoM written in the inertial frame                       |
 * |`angular_momentum` | `Eigen::Vector3d` | The centroidal angular momentum whose coordinates are written respect the inertial frame. |
 *
 * The `StateDerivative` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |           Name         |        Type       |                                     Description                                    |
 * |:----------------------:|:-----------------:|:----------------------------------------------------------------------------------:|
 * |        `com_vel`       | `Eigen::Vector3d` |              Velocity of the CoM written in the inertial frame                     |
 * |        `com_acc`       | `Eigen::Vector3d` |            Acceleration of the CoM written in the inertial frame                   |
 * |`angular_momentum_rate` | `Eigen::Vector3d` | The centroidal angular momentum rate of change written respect the inertial frame. |
 *
 * The `Input` is described by a BipedalLocomotion::GenericContainer::named_tuple
 * |        Name       |                                          Type                                           |                                                         Description                                                         |
 * |:-----------------:|:---------------------------------------------------------------------------------------:|:---------------------------------------------------------------------------------------------------------------------------:|
 * |     `contacts`    | `std::unordered_map<std::string, BipedalLocomotion::Contacts::DiscreteGeometryContact>` |  List of contact where each force applied in the discrete geometry contact is expressed with respect to the inertial frame  |
 * | `external_wrench` |               `std::optional<BipedalLocomotion::Math::Wrenchd>`                         |         Optional wrench applied to the robot center of mass. The coordinates are expressed in the inertial frame            |
 */
class CentroidalDynamics : public DynamicalSystem<CentroidalDynamics>
// clang-format on
{
    /** Gravity vector expressed in the inertial frame*/
    Eigen::Vector3d m_gravity{0, 0, -Math::StandardAccelerationOfGravitation};
    double m_mass{1}; /**< Entire mass of the robot in kg */

    State m_state; /**< State of the dynamical system */
    Input m_controlInput; /**< Input of the dynamical system */

public:
    // clang-format off
    /**
     * Initialize the CentroidalDynamics system.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are used
     * | Parameter Name |   Type   |                                          Description                                         | Mandatory |
     * |:--------------:|:--------:|:--------------------------------------------------------------------------------------------:|:---------:|
     * |   `gravity`    | `double` |     Value of the Gravity. If not defined Math::StandardAccelerationOfGravitation is used     |    No     |
     * |     `mass`     | `double` |                   Value of the mass in kg. If not defined 1 is used.                         |    No     |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);
    // clang-format on

    /**
     * Computes the centroidal momentum dynamics. It return \f$f(x, u, t)\f$.
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

    /**
     * Set the gravity vector of the dynamical system.
     * @param gravity the value of the gravity vector used in the system dynamics.
     * @return true in case of success, false otherwise.
     */
    void setGravityVector(const Eigen::Ref<const Eigen::Vector3d>& gravity);
};
} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_CENROIDAL_DYNAMICS_H
