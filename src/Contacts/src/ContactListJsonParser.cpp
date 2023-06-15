/**
 * @file ContactListJsonParser.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <fstream>

#include <nlohmann/json.hpp>

#include <Eigen/Dense>

#include <manif/SE3.h>
#include <manif/SO3.h>

// BipedalLocomotion
#include <BipedalLocomotion/Contacts/ContactList.h>
#include <BipedalLocomotion/Contacts/ContactListJsonParser.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

namespace BipedalLocomotion
{
namespace Contacts
{

ContactListMap contactListMapFromJson(const std::string& filename)
{
    constexpr auto logPrefix = "[contactListMapFromJson]";
    ContactListMap map;

    // open the file
    std::ifstream inputStream(filename);

    if (!inputStream.is_open())
    {
        log()->error("{} Failed to open {}. An empty map will be returned.", logPrefix, filename);
        return map;
    }

    nlohmann::json jsonFile;
    inputStream >> jsonFile;

    // iterate over the name of the contacts
    PlannedContact tempContact;
    for (const auto& [name, contacts] : jsonFile.items())
    {
        // iterate over all the contacts
        for (const auto& contact : contacts)
        {
            try
            {
                const std::vector<double>& position = contact.at("position");
                const std::vector<double>& quaternion = contact.at("quaternion");

                if (position.size() != 3 || quaternion.size() != 4)
                {
                    log()->error("{} The size of the position and/or the quaternion is different "
                                 "from "
                                 "expected. Position size {}, expected: 4. Quaternion size: {}, "
                                 "expected: 4. An empty ContactListMap is returned",
                                 logPrefix,
                                 position.size(),
                                 quaternion.size());
                    return ContactListMap();
                }

                tempContact.pose.quat(Eigen::Map<const manif::SO3d>(quaternion.data()));
                tempContact.pose.translation(Eigen::Map<const Eigen::Vector3d>(position.data()));
                tempContact.activationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(contact.at("activation_time")));
                tempContact.deactivationTime = std::chrono::duration_cast<std::chrono::nanoseconds>(
                    std::chrono::duration<double>(contact.at("deactivation_time")));
                tempContact.name = contact.at("name");
                tempContact.index = contact.at("index");
            } catch (const nlohmann::json::out_of_range& exception)
            {
                log()->error("{} Unable to parse the file named {}. Contact name {}.",
                             logPrefix,
                             filename,
                             name);
                log()->debug("{} The following exception has been thrown: {}.",
                             logPrefix,
                             exception.what());

                return ContactListMap();
            }

            map[name].addContact(tempContact);
        }
    }

    // close the stream
    inputStream.close();

    return map;
}

bool contactListMapToJson(const ContactListMap& map, const std::string& filename)
{
    constexpr auto logPrefix = "[contactListMapToJson]";

    // open the file
    std::ofstream outStream(filename);

    if (!outStream.is_open())
    {
        log()->error("{} Failed to open {}.", logPrefix, filename);
        return false;
    }

    nlohmann::json jsonFile;
    nlohmann::json jsonContact;

    for (const auto& [key, contacts] : map)
    {
        // iterate over all the contacts
        for (const auto& contact : contacts)
        {
            jsonContact["name"] = contact.name;
            jsonContact["index"] = contact.index;
            jsonContact["activation_time"]
                = std::chrono::duration<double>(contact.activationTime).count();
            jsonContact["deactivation_time"]
                = std::chrono::duration<double>(contact.deactivationTime).count();

            // copy the position
            for (int i = 0; i < 3; i++)
            {
                jsonContact["position"][i] = contact.pose.translation()[i];
            }

            // copy the quaternion
            for (int i = 0; i < 4; i++)
            {
                jsonContact["quaternion"][i] = contact.pose.quat().coeffs()[i];
            }

            jsonFile[key].push_back(jsonContact);
        }
    }

    // store the contact list map
    outStream << jsonFile;

    // close the stream
    outStream.close();

    return true;
}

} // namespace Contacts
} // namespace BipedalLocomotion
