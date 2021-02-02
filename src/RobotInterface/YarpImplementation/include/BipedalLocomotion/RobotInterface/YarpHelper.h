/**
 * @file YarpHelper.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H
#define BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H

// std
#include <memory>
#include <string>

// YARP
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * PolyDriverDescriptor describes a PolyDriver.
 * @note Please be careful with the namespaces then to avoid clashes with YARP
 * [PolyDriverDescriptor](https://www.yarp.it/git-master/classyarp_1_1dev_1_1PolyDriverDescriptor.html).
 */
struct PolyDriverDescriptor
{
    std::string key{""}; /**< key associated to the polydriver */
    std::shared_ptr<yarp::dev::PolyDriver> poly{nullptr}; /**< Pointer associated to the polydriver. */

    /**
     * Constructor.
     */
    PolyDriverDescriptor(const std::string& key, std::shared_ptr<yarp::dev::PolyDriver> poly);

    /**
     * Constructor.
     */
    PolyDriverDescriptor();

    /**
     * Check if the poly driver descriptor is valid.
     * @return True if the polydriver is valid, false otherwise.
     */
    bool isValid() const;
};

/**
 * Helper function that can be used to build a RemoteControlBoardRemapper device.
 * @param handler pointer to a parameter handler interface.
 * @note the following parameters are required by the function
 * |      Parameter Name     |       Type       |                         Description                        | Mandatory |
 * |:-----------------------:|:----------------:|:----------------------------------------------------------:|:---------:|
 * |      `joints_list`      | `vector<string>` |                List of the controlled joints               |    Yes    |
 * | `remote_control_boards` | `vector<string>` | List of the remote control boards associated to the joints |    Yes    |
 * |       `robot_name`      |     `string`     |                      Name of the robot                     |    Yes    |
 * |      `local_prefix`     |     `string`     |    Prefix of the local port (e.g. the application name)    |    Yes    |
 * @return A PolyDriverDescriptor. If one of the parameters is missing an invalid PolyDriverDescriptor is returned.
 */
PolyDriverDescriptor constructRemoteControlBoardRemapper(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H
