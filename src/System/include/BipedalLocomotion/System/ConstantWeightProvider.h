/**
 * @file ConstantWeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H

#include <memory>

#include <Eigen/Dense>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/WeightProvider.h>

namespace BipedalLocomotion
{
namespace System
{

/**
 * ConstantWeightProvider describes the provider for a constant weight.
 */
class ConstantWeightProvider : public WeightProvider
{

    Eigen::VectorXd m_weight; /**< Vector representing the diagonal matrix of a weight */

public:
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
     * Since the weight is constant this will always return true and it does nothing
     */
    bool advance() final;

    /**
     * Get the weight associated to the provider
     * @return A vector representing the diagonal matrix of the weight
     */
    const Eigen::VectorXd& getOutput() const final;

    /**
     * Determines the validity of the weight
     * @return True if the weight is valid, false otherwise.
     */
    bool isOutputValid() const final;

    /**
     * Initialize constant weight provider.
     * @param handler pointer to the parameter handler.
     * @note The following parameters are required:
     * |  Parameter Name  |        Type      |                          Description | Mandatory |
     * |:----------------:|:----------------:|:-----------------------------------------------------------------:|:---------:|
     * |    `weight`      | `vector<double>` |  Vector representing the diagonal matrix of a constant weight     |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H
