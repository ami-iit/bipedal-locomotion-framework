/**
 * @file ILinearTaskFactory.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY
#define BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY

#include <memory>
#include <string>
#include <type_traits>
#include <utility>

#include <BipedalLocomotion/System/Factory.h>
#include <BipedalLocomotion/System/LinearTask.h>

/**
 * BLF_REGISTER_TASK is a macro that can be used to register a task given a baseType. The key of the
 * task will be the stringified version of the Task C++ Type
 * @param _type the type of the task
 * @param _baseType the base type from which the _task inherits.
 */
#define BLF_REGISTER_TASK(_type, _baseType)                   \
    static std::shared_ptr<_baseType> _type##FactoryBuilder() \
    {                                                         \
        return std::make_shared<_type>();                     \
    };                                                        \
                                                              \
    static std::string _type##BuilderAutoRegHook              \
        = ::BipedalLocomotion::System::ILinearTaskFactory<    \
            _baseType>::registerBuilder(#_type, _type##FactoryBuilder);

namespace BipedalLocomotion
{
namespace System
{

/**
 * ILinearTaskFactory implements the factory design patter for constructing a linear task given its
 * type.
 */
template <typename _Task> class ILinearTaskFactory : public Factory<_Task>
{
    static_assert(std::is_base_of_v<LinearTask, _Task>,
                  "The _Task template argument of ILinearTaskFactory must inherits from "
                  "LinearTask");

public:
    using Task = typename Factory<_Task>::Type;
    using TaskBuilder = typename Factory<_Task>::Builder;

    virtual ~ILinearTaskFactory() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY
