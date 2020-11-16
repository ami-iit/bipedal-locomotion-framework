/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <fstream>
#include <iomanip>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/JointPositionTracking/Module.h>

using Vector1d = Eigen::Matrix<double, 1, 1>;

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::JointPositionTracking;

double Module::getPeriod()
{
    return m_dT;
}

bool Module::createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    // create the polydriver
    m_robotDevice = std::make_shared<yarp::dev::PolyDriver>();

    auto robotInterfaceOptions = handler->getGroup("ROBOT_INTERFACE");

    if (robotInterfaceOptions.lock() == nullptr)
    {
        std::cerr << "[Module::createPolydriver] Robot interface options is empty." << std::endl;
        return false;
    }

    std::vector<std::string> jointsList;
    robotInterfaceOptions.lock()->getParameter("joints_list", jointsList);

    std::vector<std::string> controlBoards;
    robotInterfaceOptions.lock()->getParameter("remote_control_boards", controlBoards);

    if(jointsList.size() != 1 || controlBoards.size() != 1)
    {
        std::cerr << "[Module::createPolydriver] The current implementation can be used to control "
                     "only one joint. Please be sure that the size of the joint_list and "
                     "remote_control_boards is equal to one."
                  << std::endl;
        return false;
    }

    std::string robotName;
    robotInterfaceOptions.lock()->getParameter("robot_name", robotName);

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");

    options.addGroup("axesNames");
    yarp::os::Bottle& bot = options.findGroup("axesNames").addList();
    for (const auto& joint : jointsList)
        bot.addString(joint);

    yarp::os::Bottle remoteControlBoards;

    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for (const auto& controlBoard : controlBoards)
        remoteControlBoardsList.addString("/" + robotName + "/" + controlBoard);

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + this->getName() + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    if (!m_robotDevice->open(options))
    {
        std::cerr << "[Module::createPolydriver] Could not open remotecontrolboardremapper object."
                  << std::endl;
        return false;
    }
    return true;
}

bool Module::initializeRobotControl(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_robotControl.initialize(handler->getGroup("ROBOT_CONTROL")))
    {
        std::cerr << "[Module::initializeRobotControl] Unable to initialize the "
                     "control board"
                  << std::endl;
        return false;
    }
    if (!m_robotControl.setDriver(m_robotDevice))
    {
        std::cerr << "[Module::initializeRobotControl] Unable to initialize the "
                     "control board"
                  << std::endl;
        return false;
    }

    return true;
}

bool Module::instantiateSensorBridge(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    if (!m_sensorBridge.initialize(handler->getGroup("SENSOR_BRIDGE")))
    {
        std::cerr << "[Module::initializeSensorBridge] Unable to initialize the sensor bridge"
                  << std::endl;
        return false;
    }

    yarp::dev::PolyDriverList list;
    list.push(m_robotDevice.get(), "Remote control board");
    if (!m_sensorBridge.setDriversList(list))
    {
        std::cerr << "[Module::initializeSensorBridge] Unable to set the driver list" << std::endl;
        return false;
    }

    return true;
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    m_currentJointPos.resize(1);

    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if(!parametersHandler->getParameter("name", name))
        return false;
    this->setName(name.c_str());

    if(!parametersHandler->getParameter("sampling_time", m_dT))
        return false;

    double maxValue = 0;
    if(!parametersHandler->getParameter("max_angle_deg", maxValue))
        return false;

    maxValue *= M_PI / 180;

    double minValue = 0;
    if(!parametersHandler->getParameter("min_angle_deg", minValue))
        return false;

    minValue *= M_PI / 180;

    double trajectoryDuration = 5;
    if(!parametersHandler->getParameter("trajectory_duration", trajectoryDuration))
        return false;

    this->createPolydriver(parametersHandler);
    this->initializeRobotControl(parametersHandler);
    this->instantiateSensorBridge(parametersHandler);

    m_setPoints.push_back((maxValue + minValue) / 2);
    m_setPoints.push_back(maxValue);
    m_setPoints.push_back(minValue);
    m_setPoints.push_back((maxValue + minValue) / 2);

    m_spline.setAdvanceTimeStep(m_dT);
    m_spline.setInitialConditions(Vector1d::Zero(), Vector1d::Zero());
    m_spline.setFinalConditions(Vector1d::Zero(), Vector1d::Zero());

    m_timeKnots.clear();
    m_timeKnots.push_back(0);
    m_timeKnots.push_back(trajectoryDuration);

    if (!m_sensorBridge.advance())
    {
        std::cerr << "[Module::updateModule] Unable to read the sensor." << std::endl;
        return false;
    }
    m_sensorBridge.getJointPositions(m_currentJointPos);

    // the trajectory will bring the robot in the initial configuration
    m_setPoints.push_back(m_currentJointPos[0]);
    m_currentSetPoint = m_setPoints.begin();

    m_trajectoryKnots.push_back(m_currentJointPos);
    m_trajectoryKnots.push_back(Vector1d::Constant(*m_currentSetPoint));

    m_spline.setKnots(m_trajectoryKnots, m_timeKnots);

    m_initTrajectoryTime = yarp::os::Time::now();

    std::cout << "[Module::configure] Starting the experiment." << std::endl;

    return true;
}

bool Module::generateNewTrajectory()
{
    double initTrajectory = *m_currentSetPoint;

    // the trajectory is ended
    if (std::next(m_currentSetPoint, 1) == m_setPoints.end())
        return false;

    std::advance(m_currentSetPoint, 1);
    double endTrajectory = *m_currentSetPoint;

    m_trajectoryKnots[0] = Vector1d::Constant(initTrajectory);
    m_trajectoryKnots[1] = Vector1d::Constant(endTrajectory);

    m_spline.setKnots(m_trajectoryKnots, m_timeKnots);

    m_initTrajectoryTime = yarp::os::Time::now();

    return true;
}

bool Module::updateModule()
{
    if (!m_sensorBridge.advance())
    {
        std::cerr << "[Module::updateModule] Unable to read the sensor." << std::endl;
        return false;
    }

    m_sensorBridge.getJointPositions(m_currentJointPos);

    // set the reference
    m_robotControl.setReferences(m_spline.get().position,
                                 RobotInterface::IRobotControl::ControlMode::PositionDirect);

    m_logJointPos.push_back(m_currentJointPos[0]);
    m_logDesiredJointPos.push_back(m_spline.get().position[0]);

    // advance the spline
    m_spline.advance();

    const double now = yarp::os::Time::now();
    if (now - m_initTrajectoryTime > m_timeKnots.back() + 2)
    {
        std::cout << "[Module::updateModule] Generate a new trajectory." << std::endl;

        if (!generateNewTrajectory())
        {
            std::cerr << "[Module::updateModule] Experiment finished." << std::endl;
            return false;
        }
    }

    return true;
}

bool Module::close()
{
    std::cout << "[Module::close] I'm storing the dataset." << std::endl;

    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::ofstream stream; /**< std stream. */
    std::stringstream fileName;

    fileName << "Dataset_" << m_robotControl.getJointList().front() << "_"
             << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".txt";
    stream.open(fileName.str().c_str());

    const auto minSize = std::min(m_logJointPos.size(), m_logDesiredJointPos.size());

    stream << "desired_joint, measured_joint" << std::endl;
    for (int i = 0; i < minSize; i++)
        stream << m_logDesiredJointPos[i] << ", " << m_logJointPos[i] << std::endl;

    stream.close();

    std::cout << "[Module::close] Dataset stored. Closing." << std::endl;

    // switch back in position control
    m_robotControl.setReferences(m_spline.get().position,
                                 RobotInterface::IRobotControl::ControlMode::Position);

    return true;
}
