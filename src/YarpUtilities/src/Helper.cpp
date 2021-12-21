/**
 * @file Helper.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/YarpUtilities/Helper.h>

using namespace BipedalLocomotion;
bool YarpUtilities::addVectorOfStringToProperty(yarp::os::Property& prop,
                                                const std::string& key,
                                                const std::vector<std::string>& list)
{
    // check if the key already exists
    if (prop.check(key))
    {
        log()->error("[YarpUtilities::addVectorOfStringToProperty::addVectorOfStringToProperty] "
                     "The property already exist.");
        return false;
    }

    prop.addGroup(key);
    yarp::os::Bottle& bot = prop.findGroup(key).addList();
    for (size_t i = 0; i < list.size(); i++)
        bot.addString(list[i].c_str());

    return true;
}

void YarpUtilities::populateBottleWithStrings(yarp::os::Bottle& bottle,
                                              const std::initializer_list<std::string>& strings)
{
    for (const auto& string : strings)
        bottle.addString(string);
}

template <> int YarpUtilities::convertValue<int>(const yarp::os::Value& value)
{
    return value.asInt32();
}

template <> double YarpUtilities::convertValue<double>(const yarp::os::Value& value)
{
    return value.asFloat64();
}

template <> std::string YarpUtilities::convertValue<std::string>(const yarp::os::Value& value)
{
    return value.asString();
}

template <> bool YarpUtilities::convertValue<bool>(const yarp::os::Value& value)
{
    return value.asBool();
}

template <>
bool YarpUtilities::getVectorFromSearchable<std::vector<bool>>(const yarp::os::Searchable& config,
                                                               const std::string& key,
                                                               std::vector<bool>& vector)
{
    constexpr auto logPrefix = "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable]";

    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        log()->debug("{} Missing field {}.", logPrefix, key);
        return false;
    }

    if (value->isNull())
    {
        log()->debug("{} Empty input value named: {}.", logPrefix, key);
        return false;
    }

    if (!value->isList())
    {
        log()->debug("{} The value named: {} is not associated to a list.", logPrefix, key);
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();
    if (inputPtr == nullptr)
    {
        log()->debug("{} The list associated to the value named: {} is empty.", logPrefix, key);
        return false;
    }

    // resize the vector
    vector.resize(inputPtr->size());

    for (size_t i = 0; i < inputPtr->size(); i++)
    {
        if (!inputPtr->get(i).isBool())
        {
            log()->debug("{} The element of the list associated to the value named {} is not a "
                         "Boolean.",
                         logPrefix,
                         key);
            return false;
        }

        vector[i] = YarpUtilities::convertValue<bool>(inputPtr->get(i));
    }
    return true;
}
