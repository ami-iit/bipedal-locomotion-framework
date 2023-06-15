/**
 * @file WeightProvider.h
 * @authors Giulio Romualdi
 * @copyright 2022 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_WEIGHT_PROVIDER_H
#define BIPEDAL_LOCOMOTION_SYSTEM_WEIGHT_PROVIDER_H

#include <BipedalLocomotion/System/Factory.h>
#include <BipedalLocomotion/System/Source.h>

/**
 * BLF_REGISTER_WEIGHT_PROVIDER is a macro that can be used to register a WeightProvider. The key of
 * the task will be the stringified version of the Task C++ Type
 * @param _type the type of the task
 */
#define BLF_REGISTER_WEIGHT_PROVIDER(_type)                                                     \
    static std::shared_ptr<::BipedalLocomotion::System::WeightProvider> _type##FactoryBuilder() \
    {                                                                                           \
        return std::make_shared<_type>();                                                       \
    };                                                                                          \
                                                                                                \
    static std::string _type##BuilderAutoRegHook = ::BipedalLocomotion::System::                \
        WeightProviderFactory::registerBuilder(#_type, _type##FactoryBuilder);

namespace BipedalLocomotion
{
namespace System
{

/**
 * WeightProvider describes the provider for a weight.
 */
struct WeightProvider : public Source<Eigen::VectorXd>
{
};

/**
 * WeightProviderFactory implements the factory design patter for constructing a WeightProvidergiven
 * its type.
 */
class WeightProviderFactory : public Factory<WeightProvider>
{

public:
    virtual ~WeightProviderFactory() = default;
};

} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_CONSTANT_WEIGHT_PROVIDER_H
