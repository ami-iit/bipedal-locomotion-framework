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

#include <BipedalLocomotion/System/LinearTask.h>

/**
 * BLF_REGISTER_TASK is a macro that can be used to register a task given a baseType. The key of the
 * task will be the stringified version of the Task C++ Type
 * @param _type the type of the task
 * @param _baseType the base type from which the _task inherits.
 */
#define BLF_REGISTER_TASK(_type, _baseType)                   \
    static std::shared_ptr<_baseType> _type##FactoryCreator() \
    {                                                         \
        return std::make_shared<_type>();                     \
    };                                                        \
                                                              \
    static std::string _type##CreatorAutoRegHook              \
        = ::BipedalLocomotion::System::ILinearTaskFactory<    \
            _baseType>::registerCreator(#_type, _type##FactoryCreator);

namespace BipedalLocomotion
{
namespace System
{

/**
 * ILinearTaskFactory implements the factory design patter for constructing a linear task given its
 * type.
 */
template <typename _Task> class ILinearTaskFactory
{
    static_assert(std::is_base_of_v<LinearTask, _Task>,
                  "The _Task template argument of ILinearTaskFactory must inherits from "
                  "LinearTask");

public:
    using Task = _Task;
    using TaskCreator = typename std::add_pointer<std::shared_ptr<Task>()>::type; /**< Pointer to
                                                                                     the task
                                                                                     builder */

private:
    /**
     * Get the map containing a map of idKey and the associated TaskCreator function.
     * @return an unordered_map of idKey and pointer to create the task
     */
    static std::unordered_map<std::string, TaskCreator>& getMapFactory()
    {
        static std::unordered_map<std::string, TaskCreator> m_mapFactory;
        return m_mapFactory;
    }

public:
    /**
     * Add a creator for a given task.
     * @param idKey the string representing the type of the task. i.e., the stringify version of the
     * class type
     * @param classCreator pointer to the function that creates a given task.
     * @return the idKey
     */
    static std::string registerCreator(std::string idKey, TaskCreator classCreator)
    {
        getMapFactory().insert(std::pair<std::string, TaskCreator>(idKey, classCreator));
        return idKey;
    }

    /**
     * Create an instance of a Task given its id.
     * @param idKey the string representing the type of the task. i.e., the stringify version of the
     * class type
     * @return a pointer to the Task. In case of issues, the pointer will be a nullptr.
     */
    static std::shared_ptr<Task> createInstance(const std::string& idKey)
    {
        auto it = getMapFactory().find(idKey);
        if (it == getMapFactory().end())
        {
            return std::shared_ptr<Task>();
        }

        if (it->second == nullptr)
        {
            return std::shared_ptr<Task>();
        }

        return it->second();
    }

    virtual ~ILinearTaskFactory() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_ILINEAR_TASK_FACTORY
