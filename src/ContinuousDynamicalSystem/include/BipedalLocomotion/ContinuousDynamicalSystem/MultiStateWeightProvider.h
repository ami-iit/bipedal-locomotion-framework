/**
 * @file MultiStateWeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_MULTISTATE_WEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_MULTISTATE_WEIGHT_PROVIDER_H

#include <memory>
#include <string>
#include <unordered_map>

#include <Eigen/Dense>

#include <BipedalLocomotion/ContinuousDynamicalSystem/FirstOrderSmoother.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/IWeightProvider.h>

namespace BipedalLocomotion
{
namespace ContinuousDynamicalSystem
{

/**
 * MultiStateWeightProvider describes the provider for a constant weight.
 */
class MultiStateWeightProvider : public System::IWeightProvider
{
public:

    /**
     * Get the weight associated to the provider
     * @return A vector representing the diagonal matrix of the weight
     */
    Eigen::Ref<const Eigen::VectorXd> getWeight() const final;

    /**
     * Initialize constant weight provider.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required:
     * |  Parameter Name  |        Type      |                             Description                              | Mandatory |
     * |:----------------:|:----------------:|:--------------------------------------------------------------------:|:---------:|
     * |    `states`      | `vector<string>` |  Vector associated to the names of the state stored in the provider  |    Yes    |
     * |  `sampling_time`  | `double` | Sampling time used to discrete the linear system |     Yes    |
     * |  `settling_time`  | `double` | 5% settling time (The settling time, \f$T_s\f$, is the time required for the system output to fall within a certain percentage (i.e. 5%) of the steady-state value for a step input.) |    Yes    |
     *
     * For each `state` defined in `states`, you should define a Group called `state` that contains the following parameters
     * |  Parameter Name  |        Type      |                              Description                          | Mandatory |
     * |:----------------:|:----------------:|:-----------------------------------------------------------------:|:---------:|
     * |    `weight`      | `vector<double>` |  Vector representing the diagonal matrix of a constant weight     |    Yes    |
     * |     `name`       |     `string`     |                Name of the state                                  |    Yes    |
     * @return true in case of success/false otherwise.
     * @note the inner dynamical system is initialized with the first `state` stored in `states` vector.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);

    /**
     * Set the state of the provider.
     * @param state name of the state. Please notice that the name should be the one passed in the
     * initialization
     * @return true in case of success/false otherwise.
     */
    bool setState(const std::string& state);

    /**
     * Reset the internal weight of the provider. This method reset also the value of the dynamical
     * system.
     * @param state name of the state. Please notice that the name should be the one passed in the
     * initialization.
     * @return true in case of success/false otherwise.
     */
    bool reset(const std::string& state);

    /**
     * Update the content of the weight provider
     * @return True in case of success and false otherwise
     */
    bool advance();

private:
    std::unordered_map<std::string, Eigen::VectorXd> m_states; /**< Map representing the states of
                                                                   the provider */
    FirstOrderSmoother m_smoother; /**< Smoother required to perform the weight scheduling */
};
} // namespace ContinuousDynamicalSystem
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_CONTINUOUS_DYNAMICAL_SYSTEM_MULTISTATE_WEIGHT_PROVIDER_H
