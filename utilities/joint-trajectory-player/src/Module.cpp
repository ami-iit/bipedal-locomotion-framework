/**
 * @file Module.cpp
 * @authors Ines Sorrentino
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <fstream>
#include <iomanip>

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <BipedalLocomotion/RobotInterface/YarpHelper.h>
#include <BipedalLocomotion/RobotInterface/YarpRobotControl.h>
#include <BipedalLocomotion/RobotInterface/YarpSensorBridge.h>

#include <Eigen/Dense>

#include <BipedalLocomotion/JointTrajectoryPlayer/Module.h>

#include <yarp/dev/IEncoders.h>

using namespace BipedalLocomotion;
using namespace BipedalLocomotion::JointTrajectoryPlayer;

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
    ptr->setParameter("local_name", this->getName());
    m_robotDevice = RobotInterface::constructYarpRobotDevice(ptr);
    if (m_robotDevice == nullptr)
    {
        std::cerr << "[Module::createPolydriver] the robot polydriver has not been constructed."
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
    if (!m_robotControl.initialize(handler->getGroup("PID")))
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

std::pair<bool, std::deque<Eigen::VectorXd>>
Module::readStateFromFile(const std::string& filename, const std::size_t numFields)
{
    std::deque<Eigen::VectorXd> data;

    std::ifstream istrm(filename);

    if (!istrm.is_open())
    {
        std::cout << "Failed to open " << filename << '\n';
        return std::make_pair(false, data);
    } else
    {
        std::vector<std::string> istrm_strings;
        std::string line;
        while (std::getline(istrm, line))
        {
            istrm_strings.push_back(line);
        }

        Eigen::VectorXd vector;
        vector.resize(numFields);
        std::size_t found_lines = 0;
        for (auto line : istrm_strings)
        {
            std::size_t found_fields = 0;
            std::string number_str;
            std::istringstream iss(line);

            while (iss >> number_str)
            {
                vector(found_fields) = std::stod(number_str);
                found_fields++;
            }
            if (numFields != found_fields)
            {
                std::cout << "[Module::readStateFromFile] Malformed input file " << filename
                          << std::endl;
                std::cout << "[Module::readStateFromFile] Expected " << numFields
                          << " columns, found " << found_fields << std::endl;

                return std::make_pair(false, data);
            }
            data.push_back(vector);
            found_lines++;
        }

        return std::make_pair(true, data);
    }
}

bool Module::configure(yarp::os::ResourceFinder& rf)
{
    auto parametersHandler = std::make_shared<ParametersHandler::YarpImplementation>(rf);

    std::string name;
    if (!parametersHandler->getParameter("name", name))
        return false;
    this->setName(name.c_str());

    if (!parametersHandler->getParameter("sampling_time", m_dT))
        return false;

    std::string trajectoryFile;
    if (!parametersHandler->getParameter("trajectory_file", trajectoryFile))
    {
        std::cerr << "[Module::configure] trajectory_file parameter not specified." << std::endl;
        return false;
    }

    this->createPolydriver(parametersHandler);
    this->initializeRobotControl(parametersHandler);
    this->instantiateSensorBridge(parametersHandler);

    m_numOfJoints = m_robotControl.getJointList().size();
    if (m_numOfJoints == 0)
    {
        std::cerr << "[Module::configure] No joints to control." << std::endl;
        return false;
    }

    m_currentJointPos.resize(m_numOfJoints);

    m_qDesired.clear();
    auto data = readStateFromFile(trajectoryFile, m_numOfJoints);
    if (!data.first)
    {
        return false;
    }
    m_qDesired = data.second;

    // check if vector is not initialized
    if (m_qDesired.empty())
    {
        std::cerr << "[Module::configure] Cannot advance empty reference signals." << std::endl;
        return false;
    }

    std::cout << "[Module::configure] Starting the experiment." << std::endl;

    // Reach the first position of the desired trajectory in position control
    m_robotControl.setReferences(m_qDesired.front(),
                                 RobotInterface::IRobotControl::ControlMode::Position);

    m_state = State::positioning;

    return true;
}

bool Module::advanceReferenceSignals()
{
    m_qDesired.pop_front();

    // check if the vector is empty. If true the trajectory is ended
    if (m_qDesired.empty())
    {
        return false;
    }

    return true;
}

bool Module::updateModule()
{
    bool isMotionDone;
    bool isTimeExpired;
    std::vector<std::pair<std::string, double>> jointlist;

    switch (m_state)
    {
    case State::positioning:
        if (!m_robotControl.checkMotionDone(isMotionDone, isTimeExpired, jointlist))
        {
            std::cerr << "";
            return false;
        }
        if (isTimeExpired)
        {
            std::cerr << "Printa lista giunti che non hanno finito (primo nome del giunto secondo "
                         "errore in radianti)";
            return false;
        }
        if (isMotionDone)
        {
            m_state = State::running;
        }
        break;

    case State::running:
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

        // set the reference
        if (!m_robotControl
                 .setReferences(m_qDesired.front(),
                                RobotInterface::IRobotControl::ControlMode::PositionDirect))
        {
            std::cerr << "[Module::updateModule] Error while setting the reference position to "
                         "iCub."
                      << std::endl;
            return false;
        }

        m_logJointPos.push_back(m_currentJointPos);

        if (!advanceReferenceSignals())
        {
            std::cout << "[Module::updateModule] Experiment finished." << std::endl;

            return false;
        }
        break;

    default:
        std::cerr << "";
        return false;
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

    std::ofstream stream; /**< std stream. */
    std::stringstream fileName;

    fileName << "Dataset_Measured_" << m_robotControl.getJointList().front() << "_"
             << std::put_time(&tm, "%Y_%m_%d_%H_%M_%S") << ".txt";
    stream.open(fileName.str().c_str());

    const auto sizeTraj = m_logJointPos.size();

    for (int i = 0; i < sizeTraj; i++)
        stream << m_logJointPos[i].transpose() << std::endl;

    stream.close();

    std::cout << "[Module::close] Dataset stored. Closing." << std::endl;

    // switch back in position control
    m_robotControl.setReferences(m_currentJointPos,
                                 RobotInterface::IRobotControl::ControlMode::Position);

    return true;
}
