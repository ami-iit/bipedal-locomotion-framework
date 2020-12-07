/**
 * @file YarpHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/RobotInterface/YarpHelper.h>

std::shared_ptr<yarp::dev::PolyDriver> BipedalLocomotion::RobotInterface::constructYarpRobotDevice(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto robotDevice = std::make_shared<yarp::dev::PolyDriver>();
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        std::cerr << "[constructYarpRobotDevice] IParametershandler is empty." << std::endl;
        return robotDevice;
    }

    bool ok = true;

    std::vector<std::string> jointsList;
    ok = ok && ptr->getParameter("joints_list", jointsList);

    std::vector<std::string> controlBoards;
    ok = ok && ptr->getParameter("remote_control_boards", controlBoards);

    std::string robotName;
    ok = ok && ptr->getParameter("robot_name", robotName);

    std::string localName;
    ok = ok && ptr->getParameter("local_name", localName);

    if (!ok)
    {
        std::cerr << "[constructYarpRobotDevice] Unable to get all the parameters from "
                     "configuration file."
                  << std::endl;
        return robotDevice;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");

    options.addGroup("axesNames");
    yarp::os::Bottle& bottle = options.findGroup("axesNames").addList();
    for (const auto& joint : jointsList)
        bottle.addString(joint);

    yarp::os::Bottle remoteControlBoards;

    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (const auto& controlBoard : controlBoards)
        remoteControlBoardsList.addString("/" + robotName + "/" + controlBoard);

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + localName + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    if (!robotDevice->open(options) && !robotDevice->isValid())
    {
        std::cerr << "[configureRobot] Could not open remotecontrolboardremapper object."
                  << std::endl;
        return robotDevice;
    }

    return robotDevice;
}
