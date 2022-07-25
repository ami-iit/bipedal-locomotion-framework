/**
 * @file YarpHelper.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/RobotInterface/YarpHelper.h>

using namespace BipedalLocomotion::RobotInterface;

PolyDriverDescriptor::PolyDriverDescriptor(const std::string& key,
                                           std::shared_ptr<yarp::dev::PolyDriver> poly)
    : key(key)
    , poly(poly)
{
}

PolyDriverDescriptor::PolyDriverDescriptor() = default;

bool PolyDriverDescriptor::isValid() const
{
    return ((!key.empty()) && (poly != nullptr));
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructRemoteControlBoardRemapper(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr std::string_view errorPrefix = "[constructRemoteControlBoardRemapper] ";

    auto ptr = handler.lock();


    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "IParametershandler is empty." << std::endl;
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::vector<std::string> jointsList;
    ok = ok && ptr->getParameter("joints_list", jointsList);

    std::vector<std::string> controlBoards;
    ok = ok && ptr->getParameter("remote_control_boards", controlBoards);

    std::string robotName;
    ok = ok && ptr->getParameter("robot_name", robotName);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    if (!ok)
    {
        std::cerr << errorPrefix << "Unable to get all the parameters from configuration file."
                  << std::endl;
        return PolyDriverDescriptor();
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
    options.put("localPortPrefix", "/" + localPrefix + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    PolyDriverDescriptor device("remoteControlBoards", std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        std::cerr << errorPrefix << "Could not open polydriver object." << std::endl;
        return PolyDriverDescriptor();
    }

    return device;
}

PolyDriverDescriptor BipedalLocomotion::RobotInterface::constructGenericSensorClient(
    std::weak_ptr<const BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    constexpr std::string_view errorPrefix = "[constructGenericSensorClient] ";

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "IParametershandler is empty." << std::endl;
        return PolyDriverDescriptor();
    }

    bool ok = true;

    std::string description;
    ok = ok && ptr->getParameter("description", description);

    std::string remotePortName;
    ok = ok && ptr->getParameter("remote_port_name", remotePortName);

    std::string localPrefix;
    ok = ok && ptr->getParameter("local_prefix", localPrefix);

    std::string localPortNamePostfix;
    ok = ok && ptr->getParameter("local_port_name_postfix", localPortNamePostfix);

    if (!ok)
    {
        std::cerr << errorPrefix << "Unable to get all the parameters from configuration file."
                  << std::endl;
        return PolyDriverDescriptor();
    }

    // open the YARP device
    yarp::os::Property options;
    options.put("device", "genericSensorClient");
    options.put("remote", remotePortName);
    options.put("local", "/" + localPrefix + localPortNamePostfix);

    PolyDriverDescriptor device(description, std::make_shared<yarp::dev::PolyDriver>());

    if (!device.poly->open(options) && !device.poly->isValid())
    {
        std::cerr << errorPrefix << "Could not open polydriver object." << std::endl;
        return PolyDriverDescriptor();
    }

    return device;
}
