/**
 * @file ConstantWeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/System/IWeightProvider.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <Eigen/src/Core/Matrix.h>


namespace BipedalLocomotion
{
namespace System
{

/**
 * ConstantWeightProvider describes the provider for a constant weight.
 */
struct ConstantWeightProvider : public IWeightProvider
{

    Eigen::VectorXd weight; /**< Vector representing the diagonal matrix of a weight */

    /**
     * Default construct
     */
    ConstantWeightProvider() = default;

    /**
     * Constrict from a vector representing the diagonal matrix of a weight
     * @param weight a vector representing the diagonal matrix of a weight
     */
    ConstantWeightProvider(Eigen::Ref<const Eigen::VectorXd> weight);

    /**
     * Get the weight associated to the provider
     * @return A vector representing the diagonal matrix of the weight
     */
    Eigen::Ref<const Eigen::VectorXd> getWeight() const final;

    /**
     * Initialize constant weight provider.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required:
     * |  Parameter Name  |        Type      |                          Description                              | Mandatory |
     * |:----------------:|:----------------:|:-----------------------------------------------------------------:|:---------:|
     * |    `weight`      | `vector<double>` |  Vector representing the diagonal matrix of a constant weight     |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler);
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H
