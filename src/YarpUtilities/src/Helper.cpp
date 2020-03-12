/**
 * @file Helper.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/YarpUtilities/Helper.h>

using namespace BipedalLocomotion;
bool YarpUtilities::addVectorOfStringToProperty(yarp::os::Property& prop,
                                                const std::string& key,
                                                const std::vector<std::string>& list)
{
    // check if the key already exists
    if (prop.check(key))
    {
        std::cerr << "[YarpUtilities::addVectorOfStringToProperty::addVectorOfStringToProperty] "
                     "The property already exist."
                  << std::endl;
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
    return value.asInt();
}

template <> double YarpUtilities::convertValue<double>(const yarp::os::Value& value)
{
    return value.asDouble();
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
    yarp::os::Value* value;
    if (!config.check(key, value))
    {
        std::cerr << "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable] "
                     "Missing field "
                  << key << std::endl;
        return false;
    }

    if (value->isNull())
    {
        std::cerr << "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable] Empty "
                     "input value named "
                  << key << std::endl;
        return false;
    }

    if (!value->isList())
    {
        std::cerr << "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable] The "
                     "value named "
                  << key << "is not associated to a list." << std::endl;
        return false;
    }

    yarp::os::Bottle* inputPtr = value->asList();
    if (inputPtr == nullptr)
    {
        std::cerr << "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable] The "
                     "list associated to the value named "
                  << key << " is empty." << std::endl;
        return false;
    }

    // resize the vector
    vector.resize(inputPtr->size());

    for (size_t i = 0; i < inputPtr->size(); i++)
    {
        if (!(inputPtr->get(i).isBool()) && !(inputPtr->get(i).isInt()))
        {
            std::cerr << "[BipedalLocomotion::YarpUtilities::getVectorFromSearchable] "
                         "The element of the list associated to the value named "
                      << key << " is not a boolean ." << std::endl;
            return false;
        }

        vector[i] = YarpUtilities::convertValue<bool>(inputPtr->get(i));
    }
    return true;
}
