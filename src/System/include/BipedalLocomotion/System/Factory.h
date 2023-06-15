/**
 * @file Factory.h
 * @authors Giulio Romualdi
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_SYSTEM_FACTORY
#define BIPEDAL_LOCOMOTION_SYSTEM_FACTORY

#include <memory>
#include <string>
#include <type_traits>
#include <unordered_map>
#include <utility>

namespace BipedalLocomotion
{
namespace System
{

/**
 * Factory implements the factory design patter for constructing a an object of a given Type given
 * its id.
 */
template <typename _Type> class Factory
{
public:
    using Type = _Type;
    using Builder = typename std::add_pointer<std::shared_ptr<Type>()>::type; /**< Pointer to the
                                                                                 Type builder */

private:
    /**
     * Get the map containing a map of idKey and the associated Builder function.
     * @return an unordered_map of idKey and pointer to create the Type
     */
    static std::unordered_map<std::string, Builder>& getMapFactory()
    {
        static std::unordered_map<std::string, Builder> m_mapFactory;
        return m_mapFactory;
    }

public:
    /**
     * Add a Builder for a given Type.
     * @param idKey the string representing the type of the Type. i.e., the stringify version of the
     * class type
     * @param classBuilder pointer to the function that creates a given Type.
     * @return the idKey
     */
    static std::string registerBuilder(std::string idKey, Builder classBuilder)
    {
        getMapFactory().insert(std::pair<std::string, Builder>(idKey, classBuilder));
        return idKey;
    }

    /**
     * Create an instance of a Type given its id.
     * @param idKey the string representing the type of the Type. i.e., the stringify version of the
     * class type
     * @return a pointer to the Type. In case of issues, the pointer will be a nullptr.
     */
    static std::shared_ptr<Type> createInstance(const std::string& idKey)
    {
        auto it = getMapFactory().find(idKey);
        if (it == getMapFactory().end())
        {
            return std::shared_ptr<Type>();
        }

        if (it->second == nullptr)
        {
            return std::shared_ptr<Type>();
        }

        return it->second();
    }

    virtual ~Factory() = default;
};
} // namespace System
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_SYSTEM_FACTORY
