/**
 * @file CentroidalDynamics.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_CENROIDAL_DYNAMICS_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_CENROIDAL_DYNAMICS_H

#include <memory>
#include <tuple>
#include <map>
#include <vector>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/DynamicalSystem.h>
#include <BipedalLocomotion/ContinuousDynamicalSystem/impl/traits.h>
#include <BipedalLocomotion/Math/Constants.h>

#include <Eigen/Dense>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{
class CentroidalDynamics;
}
} // namespace BipedalLocomotion


// Please read it as
// BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(
//     FixedBaseDynamics,
//     (joint velocities, joint positions),
//     (joint accelerations, joints velocities),
//     (joint torques)
BLF_DEFINE_CONTINUOUS_DYNAMICAL_SYSTEM_INTERAL_STRUCTURE(
    CentroidalDynamics,
    (Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d),
    (Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d),
    (std::map<std::string, BipedalLocomotion::Contacts::ContactWithCorners>))

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

class CentroidalDynamics : public DynamicalSystem<CentroidalDynamics>

{
    Eigen::Vector3d m_gravity{0, 0, -Math::StandardAccelerationOfGravitation}; /**< Gravity vector
                                                                                */
    double m_mass{1};

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
     * |      `mass`    | `double` |                       Value of the mass. If not defined 1 is used.                            |    No     |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler);


    /**
     * Computes the floating based system dynamics. It return \f$f(x, u, t)\f$.
     * @note The control input and the state have to be set separately with the methods
     * setControlInput and setState.
     * @param time the time at witch the dynamics is computed.
     * @param stateDynamics tuple containing a reference to the element of the state derivative
     * @return true in case of success, false otherwise.
     */
    bool dynamics(const double& time, StateDerivative& stateDerivative);


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
}
}

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_CENROIDAL_DYNAMICS_H
