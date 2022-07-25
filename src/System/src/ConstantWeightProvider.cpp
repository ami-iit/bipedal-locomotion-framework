/**
 * @file ConstantWeightProvider.cpp
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/System/ConstantWeightProvider.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::System;
using namespace BipedalLocomotion::ParametersHandler;

ConstantWeightProvider::ConstantWeightProvider(Eigen::Ref<const Eigen::VectorXd> weight)
    : weight(weight)
{
}

bool ConstantWeightProvider::initialize(std::weak_ptr<const IParametersHandler> handler)
{
    constexpr auto logPrefix = "[ConstantWeightProvider::initialize]";

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", logPrefix);
        return false;
    }

    if (!ptr->getParameter("weight", weight))
    {
        log()->error("{} Unable to get the parameter named 'weight'.", logPrefix);
        return false;
    }

    return true;
}

Eigen::Ref<const Eigen::VectorXd> ConstantWeightProvider::getWeight() const
{
    return weight;
}
