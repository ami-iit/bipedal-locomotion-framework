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

// YARP
#include <yarp/dev/PolyDriver.h>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

namespace BipedalLocomotion
{
namespace RobotInterface
{

/**
 * Helper function that can be used to build a RemoteControlBoardRemapper device.
 * @param handler pointer to a parameter handler interface.
 * @note the following parameters are required by the function
 * |      Parameter Name     |       Type       |                         Description                        | Mandatory |
 * |:-----------------------:|:----------------:|:----------------------------------------------------------:|:---------:|
 * |      `joints_list`      | `vector<string>` |                List of the controlled joints               |    Yes    |
 * | `remote_control_boards` | `vector<string>` | List of the remote control boards associated to the joints |    Yes    |
 * |       `robot_name`      |     `string`     |                      Name of the robot                     |    Yes    |
 * |       `local_name`      |     `string`     |                Name of the local application               |    Yes    |
 * @return A shared_ptr to the PolyDriver. If one of the parameters is missing a nullptr is returned.
 */
std::shared_ptr<yarp::dev::PolyDriver> constructYarpRobotDevice(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

} // namespace RobotInterface
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_ROBOT_INTERFACE_YARP_HELPER_H
