/**
 * @file Module.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iomanip>
#include <thread>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <iDynTree/ModelIO/ModelLoader.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/CartesianTrajectoryPlayer/Module.h>

#include <yarp/dev/IEncoders.h>

#include <BipedalLocomotion/Conversions/matioCppConversions.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::CartesianTrajectoryPlayer;

double Module::getPeriod()
{
    return m_dT;
}

bool Module::createPolydriver(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{
    auto ptr = handler->getGroup("ROBOT_INTERFACE").lock();
    if (ptr == nullptr)
    {
        std::cerr << "[Module::createPolydriver] Robot interface options is empty." << std::endl;
        return false;
    }
    ptr->setParameter("local_prefix", this->getName());
    m_poly.push_back(RobotInterface::constructRemoteControlBoardRemapper(ptr));
    if (!m_poly.back().isValid())
    {
        std::cerr << "[Module::createPolydriver] the robot polydriver has not been constructed."
                  << std::endl;
        return false;
    }

    return true;
}

bool Module::createContactWrenchInterface(std::shared_ptr<ParametersHandler::IParametersHandler> handler)
{

    constexpr std::string_view errorPrefix = "[Module::createContactWrenchInterface] ";

    auto ptr = handler->getGroup("CONTACT_WRENCH_INTERFACE").lock();
    if (ptr == nullptr)
    {
        std::cerr << errorPrefix << "Robot interface options is empty." << std::endl;
        return false;
    }

    int numberOfContacts = 0;
    ptr->getParameter("max_number_of_contacts", numberOfContacts);

    for (int i = 0; i < numberOfContacts; i++)
    {
        auto groupWeak = ptr->getGroup("CONTACT_" + std::to_string(i));
        auto group = groupWeak.lock();
        if (group == nullptr)
        {
            std::cerr << errorPrefix << "The group named: CONTACT_" << i << " do not exist."
                      << std::endl;
            return false;
        }
        group->setParameter("local_prefix", this->getName());

        m_poly.push_back(RobotInterface::constructGenericSensorClient(group));
        if (!m_poly.back().isValid())
        {
            std::cerr << errorPrefix << "The robot polydriver has not been constructed."
                      << std::endl;
            return false;
        }
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
    if (!m_robotControl.setDriver(m_poly.front().poly))
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
    for(const auto& p : m_poly)
        list.push(p.poly.get(), p.key.c_str());

    if (!m_sensorBridge.setDriversList(list))
    {
        std::cerr << "[Module::initializeSensorBridge] Unable to set the driver list" << std::endl;
        return false;
    }

    return true;
}

// bool Module::readStateFromFile(const std::string& filename, const std::size_t numFields)
// {
//     std::deque<Eigen::VectorXd> data;

//     matioCpp::File input(filename);

//     if (!input.isOpen())
//     {
//         std::cout << "[Module::readStateFromFile] Failed to open " << filename << "." << std::endl;
//         return false;
//     } else
//     {
//         m_traj = input.read("traj").asMultiDimensionalArray<double>(); // Read a multi dimensional
//                                                                        // array named "traj"
//         if (!m_traj.isValid())
//         {
//             std::cerr << "[Module::readStateFromFile] Error reading input file: " << filename << "."
//                       << std::endl;
//             return false;
//         }

//         return true;
//     }
// }

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if (!parametersHandler->getParameter("name", name))
        return false;
    this->setName(name.c_str());

    if (!parametersHandler->getParameter("sampling_time", m_dT))
        return false;


    this->createPolydriver(parametersHandler);
    this->createContactWrenchInterface(parametersHandler);

    std::this_thread::sleep_for (std::chrono::seconds(1));

    this->initializeRobotControl(parametersHandler);
    this->instantiateSensorBridge(parametersHandler);

    m_numOfJoints = m_robotControl.getJointList().size();
    if (m_numOfJoints == 0)
    {
        std::cerr << "[Module::configure] No joints to control." << std::endl;
        return false;
    }

    m_axisList = m_robotControl.getJointList();
    m_currentJointPos.resize(m_numOfJoints);
    m_currentJointVel.resize(m_numOfJoints);
    m_contactForces.resize(4);

    // left foot trajectory
    Planners::ContactList leftContats;
    manif::SE3d leftTransform{{0, 0, 0}, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())};
    leftContats.addContact(leftTransform, 0.0, 9);


    // right foot trajectory
    Planners::ContactList rightContats;
    manif::SE3d rightTransform1{{0, -0.14, 0.16}, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())};
    rightContats.addContact(rightTransform1, 0.0, 1);

    manif::SE3d rightTransform2{{0, -0.14, 0.1}, Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ())};
    rightContats.addContact(rightTransform2, 3.0, 4);

    manif::SE3d rightTransform3{{0, -0.14, 0.1},
                                Eigen::AngleAxisd(10 * M_PI / 180, Eigen::Vector3d::UnitX())};
    rightContats.addContact(rightTransform3, 6.0, 7);

    manif::SE3d rightTransform4{{0, -0.14, 0.1},
                                Eigen::AngleAxisd(-10 * M_PI / 180, Eigen::Vector3d::UnitX())};
    rightContats.addContact(rightTransform4, 9.0, 10);

    manif::SE3d rightTransform5{{0, -0.14, 0.1},
                                Eigen::AngleAxisd(0, Eigen::Vector3d::UnitX())};
    rightContats.addContact(rightTransform5, 12.0, 13);


    if(!m_rightFootPlanner.initialize(parametersHandler->getGroup("FOOT_PLANNER").lock()))
    {
        return false;
    }

    if(!m_leftFootPlanner.initialize(parametersHandler->getGroup("FOOT_PLANNER").lock()))
    {
        return false;
    }

    m_leftFootPlanner.setContactList(leftContats);
    m_rightFootPlanner.setContactList(rightContats);

    // open RPC port for external command
    const std::string rpcPortName = "/" + getName() + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        std::cerr << "[WalkingModule::configure] Could not open" << rpcPortName << " RPC port."
                  << std::endl;
        return false;
    }

    // load the model in iDynTree::KinDynComputations
    const std::string model = rf.check("model", yarp::os::Value("model.urdf")).asString();
    const std::string pathToModel
        = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(model);

    std::cout << "[WalkingModule::setRobotModel] The model is found in: " << pathToModel
              << std::endl;

    iDynTree::ModelLoader modelLoader;
    // only the controlled joints are extracted from the URDF file
    if (!modelLoader.loadReducedModelFromFile(pathToModel, m_axisList))
    {
        std::cerr << "[WalkingModule::setRobotModel] Error while loading the model from "
                  << pathToModel << std::endl;
        return false;
    }

    if (!m_homing.initialize(rf.findGroup("HOMING"), modelLoader.model(), m_axisList))
    {
        std::cerr << "[WalkingModule::setRobotModel] Error while initializing the homing."
                  << std::endl;
        return false;
    }


    m_state = State::idle;

    return true;
}

bool Module::updateModule()
{

    std::lock_guard lock(m_mutex);

    if (!m_sensorBridge.advance())
    {
        std::cerr << "[Module::updateModule] Unable to read the sensor." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointPositions(m_currentJointPos))
    {
        std::cerr << "[Module::updateModule] Error in reading current position." << std::endl;
        return false;
    }

    if (!m_sensorBridge.getJointPositions(m_currentJointVel))
    {
        std::cerr << "[Module::updateModule] Error in reading current position." << std::endl;
        return false;
    }


    // std::cerr << m_currentJointPos.transpose() << std::endl;
    std::vector<std::string> wrenchesList;
    m_sensorBridge.getCartesianWrenchesList(wrenchesList);

    for(int i = 0; i < 4; i++)
        if(!m_sensorBridge.getCartesianWrench(wrenchesList[i], m_contactForces[i]))
            return false;


    // log data
    m_log["time"].push_back(yarp::os::Time::now());
    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_axisList[i] + "_pos"].push_back(m_currentJointPos[i]);
    }

    for (int i = 0; i < m_numOfJoints; i++)
    {
        m_log[m_axisList[i] + "_vel"].push_back(m_currentJointPos[i]);
    }

    for(int i = 0; i < 4; i++)
    {
        m_log[wrenchesList[i] + "_fx"].push_back(m_contactForces[i][0]);
        m_log[wrenchesList[i] + "_fy"].push_back(m_contactForces[i][1]);
        m_log[wrenchesList[i] + "_fz"].push_back(m_contactForces[i][2]);

        m_log[wrenchesList[i] + "_tx"].push_back(m_contactForces[i][3]);
        m_log[wrenchesList[i] + "_ty"].push_back(m_contactForces[i][4]);
        m_log[wrenchesList[i] + "_tz"].push_back(m_contactForces[i][5]);
    }


    bool isMotionDone;
    bool isTimeExpired;
    std::vector<std::pair<std::string, double>> jointlist;




    switch (m_state)
    {
    case State::positioning:


        if (!m_robotControl.checkMotionDone(isMotionDone, isTimeExpired, jointlist))
        {
            std::cerr << "[Module::updateModule] Impossible to check if the motion is done."
                      << std::endl;
            return false;
        }
        if (isTimeExpired)
        {
            std::cerr << "[Module::updateModule] List of joints not finishing in time: "
                      << std::endl;
            for (int i = 0; i < jointlist.size(); i++)
            {
                std::cerr << "Joint " << jointlist[i].first << "--> Error  " << jointlist[i].second
                          << " rad" << std::endl;
            }
            return false;
        }
        if (isMotionDone)
        {
            std::cerr << "I'm ready" << std::endl;
            m_state = State::ready;
        }
        break;

    case State::running:

        m_homing.setRobotState(m_currentJointPos);

        m_homing.setDesiredFeetTransform(m_leftFootPlanner.get().transform,
                                         m_rightFootPlanner.get().transform);

        if(!m_homing.solveIK())
        {
            std::cerr << "[Module::homing] Unable to solve the IK" << std::endl;
            return false;
        }


        m_robotControl.setReferences(m_homing.getJointPos(),
                                     RobotInterface::IRobotControl::ControlMode::PositionDirect);

        m_leftFootPlanner.advance();
        m_rightFootPlanner.advance();

        break;

    default:
        break;
    }

    return true;
}

bool Module::close()
{
    std::cout << "[Module::close] I'm storing the dataset." << std::endl;

    // set the file name
    std::time_t t = std::time(nullptr);
    std::tm tm = *std::localtime(&t);

    std::stringstream fileName;

    fileName << "Dataset_Measured" << "_"
             << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".mat";

    matioCpp::File file = matioCpp::File::Create(fileName.str().c_str());

    for (auto& [key, value] : m_log)
    {
        matioCpp::Vector<double> out(key);
        out = value;
        file.write(out);
    }

    std::cout << "[Module::close] Dataset stored. Closing." << std::endl;

    // switch back in position control
    m_robotControl.setReferences(m_currentJointPos,
                                 RobotInterface::IRobotControl::ControlMode::Position);

    return true;
}


bool Module::homing()
{
    std::lock_guard lock(m_mutex);

    std::cerr << "I'm homing the robot" << std::endl;


    Eigen::Vector3d comPos;
    comPos.setZero();
    comPos(2) = 0.6;
    m_homing.setRobotState(m_currentJointPos);
    m_homing.setDesiredCoMPosition(comPos);
    m_homing.setDesiredFeetTransform(m_leftFootPlanner.get().transform,
                                     m_rightFootPlanner.get().transform);

    if(!m_homing.solveIK())
    {
        std::cerr << "[Module::homing] Unable to solve the IK" << std::endl;
        return false;
    }


    m_robotControl.setReferences(m_homing.getJointPos(),
                                 RobotInterface::IRobotControl::ControlMode::Position);

    m_state = State::positioning;

    return true;
}

bool Module::start()
{
    std::lock_guard lock(m_mutex);
    m_state = State::running;

    return true;
}
