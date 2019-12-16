/**
 * @file Helper.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/YarpUtilities/Helper.h>

using namespace BipedalLocomotionControllers;
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
