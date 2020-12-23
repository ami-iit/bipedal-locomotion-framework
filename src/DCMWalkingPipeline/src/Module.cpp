/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <memory>
#include <thread>

#include <BipedalLocomotion/DCMWalkingPipeline/Module.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <yarp/dev/PolyDriver.h>

#include <iDynTree/ModelIO/ModelLoader.h>

using namespace BipedalLocomotion::DCMWalkingPipeline;
using namespace BipedalLocomotion;

double Module::getPeriod()
{
    return m_dT;
}

bool Module::updateModule()
{
    std::lock_guard lock(m_mutex);

    if(!m_sensorBridge.advance())
    {
        std::cerr << "unable to read" << std::endl;
        return false;
    }


    unsigned counter = 0;
    constexpr unsigned maxIter = 100;
    constexpr unsigned timeout = 500;
    yarp::sig::Vector* base = NULL;
    while (base == nullptr)
    {
        base = m_robotBasePort.read(false);
        if (++counter == maxIter)
        {
            std::cerr << "[Module::updateModule] Error reading the base pose." << std::endl;
            return false;
        }

        // Sleep for some while
        std::this_thread::sleep_for(std::chrono::microseconds(timeout));
    }

    // TODO remove me
    Eigen::Vector3d translation = Eigen::Map<Eigen::Vector3d>(base->data(), 3);
    translation(2) -= 0.0105;

    m_baseTransform.translation(translation);
    m_baseTransform.quat(Eigen::AngleAxisd((*base)(5), Eigen::Vector3d::UnitZ())
                         * Eigen::AngleAxisd((*base)(4), Eigen::Vector3d::UnitY())
                         * Eigen::AngleAxisd((*base)(3), Eigen::Vector3d::UnitX()));

    m_baseVelocity.coeffs() = Eigen::Map<Eigen::Matrix<double, 6, 1>>(base->data() + 6, 6);

    if(!m_pipeline.setRobotState(m_baseTransform,
                                 m_sensorBridge.getJointPositions(),
                                 m_baseVelocity,
                                 m_sensorBridge.getJointVelocities()))
    {
        std::cerr << "Unable to set the robot state" << std::endl;
        return false;
    }

    if (m_state == State::Walking)
    {
        if(!m_pipeline.computeControl())
        {
            std::cerr << "Unable to compyte the control law" << std::endl;
            return  false;
        }

        m_robotControl.setReferences(m_pipeline.getJointTorques(),
                                     RobotInterface::IRobotControl::ControlMode::Torque);

        if(!m_pipeline.advance())
        {
            std::cerr << "unable to advance " << std::endl;
            return false;
        }
    }

    return true;
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    parametersHandler->getParameter("name", name);

    this->setName(name.c_str());
    parametersHandler->getParameter("sampling_time", m_dT);


    // create the polydriver
    m_robotDevice = std::make_shared<yarp::dev::PolyDriver>();

    auto robotInterfaceOptions = parametersHandler->getGroup("ROBOT_INTERFACE");

    if(robotInterfaceOptions.lock() == nullptr)
    {
        std::cerr << "[Module::configure] Robot interface options is empty." << std::endl;
        return false;
    }

    std::vector<std::string> jointsList;
    robotInterfaceOptions.lock()->getParameter("joints_list", jointsList);


    std::vector<std::string> controlBoards;
    robotInterfaceOptions.lock()->getParameter("remote_control_boards", controlBoards);

    std::string robotName;
    robotInterfaceOptions.lock()->getParameter("robot_name", robotName);

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property options;
    options.put("device", "remotecontrolboardremapper");


    options.addGroup("axesNames");
    yarp::os::Bottle& bot = options.findGroup("axesNames").addList();
    for(const auto& joint : jointsList)
        bot.addString(joint);

    yarp::os::Bottle remoteControlBoards;

    yarp::os::Bottle& remoteControlBoardsList = remoteControlBoards.addList();
    for(const auto & controlBoard : controlBoards)
        remoteControlBoardsList.addString("/" + robotName + "/" + controlBoard);

    options.put("remoteControlBoards", remoteControlBoards.get(0));
    options.put("localPortPrefix", "/" + name + "/remoteControlBoard");
    yarp::os::Property& remoteControlBoardsOpts = options.addGroup("REMOTE_CONTROLBOARD_OPTIONS");
    remoteControlBoardsOpts.put("writeStrict", "on");

    if (!m_robotDevice->open(options))
    {
        std::cerr << "[configureRobot] Could not open remotecontrolboardremapper object."
                  << std::endl;
        return false;
    }


    m_robotControl.initialize(parametersHandler->getGroup("ROBOT_CONTROL"));
    m_robotControl.setDriver(m_robotDevice);


    auto sensorBridgeOptions = parametersHandler->getGroup("SENSOR_BRIDGE").lock();
    if(sensorBridgeOptions == nullptr)
    {
        std::cerr << "[configureRobot] Could not load SENSOR_BRIDGE group."
                  << std::endl;
        return false;
    }

    sensorBridgeOptions->setParameter("joints_list", jointsList);

    if(!m_sensorBridge.initialize(sensorBridgeOptions))
        return false;

    yarp::dev::PolyDriverList list;
    list.push(m_robotDevice.get(), "Remote control board");
    if(!m_sensorBridge.setDriversList(list))
        return false;

    // load the model in iDynTree::KinDynComputations
    const std::string model = rf.check("model", yarp::os::Value("model.urdf")).asString();
    const std::string pathToModel
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    std::cout << "[WalkingModule::setRobotModel] The model is found in: " << pathToModel
              << std::endl;

    iDynTree::ModelLoader modelLoader;
    // only the controlled joints are extracted from the URDF file
    if (!modelLoader.loadReducedModelFromFile(pathToModel, jointsList))
    {
        std::cerr << "[WalkingModule::setRobotModel] Error while loading the model from "
                  << pathToModel << std::endl;
        return false;
    }

    if (!m_homing.initialize(rf.findGroup("HOMING"), modelLoader.model(), jointsList))
    {
        std::cerr << "[WalkingModule::setRobotModel] Error while initializing the homing."
                  << std::endl;
        return false;
    }

    if (!m_pipeline.setRobotModel(modelLoader.model()))
    {
        std::cerr << "error set robot pipeline" << std::endl;
        return false;
    }
    if (!m_pipeline.initialize(parametersHandler->getGroup("PIPELINE").lock()))
    {
        std::cerr << "error initalize pipeline" << std::endl;
        return false;
    }

    // todo remove me
    Eigen::VectorXd jointMax(static_cast<int>(modelLoader.model().getNrOfJoints()));
    for (int i = 0; i < jointMax.size(); i++)
    {
        jointMax(i) = 3.14;
    }

    if (!m_pipeline.initializeTorqueController(rf.findGroup("TORQUE_CONTROL"), jointMax, -jointMax))
    {
        std::cerr << "error initalize pipeline" << std::endl;
        return false;
    }


     // open RPC port for external command
    const std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        std::cerr << "[WalkingModule::configure] Could not open" << rpcPortName << " RPC port."
                  << std::endl;
        return false;
    }


    // open robot base port
    const std::string portRobotBase = "/" + getName() + "/robotBase:i";
    m_robotBasePort.open(portRobotBase);

    yarp::os::Network::connect("/icubSim/floating_base/state:o", portRobotBase);

    m_state = State::Configured;

    return true;
}

bool Module::close()
{
    std::cerr << "];" << std::endl;

    m_robotControl
        .setReferences(m_sensorBridge.getJointPositions(),
                       BipedalLocomotion::RobotInterface::IRobotControl::ControlMode::Position);

    m_pipeline.close();

    return true;
}


bool Module::homing()
{
    std::lock_guard lock(m_mutex);

    std::cerr << "I'm generating the trajectories" << std::endl;

    auto t1 = std::chrono::high_resolution_clock::now();


    if(!m_pipeline.generateTrajectory(m_baseTransform))
        return false;

    auto t2 = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::microseconds>( t2 - t1 ).count();
    std::cout << duration;


    m_homing.setRobotState(m_sensorBridge.getJointPositions());
    m_homing.setDesiredCoMPosition(m_pipeline.getDCMPosition());
    m_homing.setDesiredFeetTransform(m_pipeline.getLeftFootTransform(),
                                     m_pipeline.getRightFootTransform());

    if(!m_homing.solveIK())
    {
        std::cerr << "[Module::homing] Unable to solve the IK" << std::endl;
        return false;
    }


    m_robotControl.setReferences(m_homing.getJointPos(),
                                 RobotInterface::IRobotControl::ControlMode::Position);

    m_pipeline.setRegularization(m_homing.getJointPos());
    m_state = State::Prepared;

    return true;
}

bool Module::startWalking()
{
    std::cerr << "desired dcm [" << std::endl;
    std::lock_guard lock(m_mutex);
    m_state = State::Walking;

    return true;
}
