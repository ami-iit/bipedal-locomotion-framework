/**
 * @file MasImuTest.cpp
 * @authors Stefano Dafarra
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/MasImuTest.h>
#include <BipedalLocomotionControllers/YarpUtilities/Helper.h>
#include <iDynTree/ModelIO/ModelLoader.h>
#include <iDynTree/yarp/YARPConfigurationsLoader.h>
#include <iDynTree/Model/IJoint.h>
#include <yarp/os/LogStream.h>
#include <iostream>
#include <cassert>

using namespace BipedalLocomotionControllers;

bool MasImuTest::MasImuData::setupModel()
{
    bool ok = m_group->getParameter("imu_frame", m_frameName);
    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setupModel] Setup failed.";
        return false;
    }

    m_frame = m_commonDataPtr->fullModel.getFrameIndex(m_frameName);

    if (m_frame == iDynTree::FRAME_INVALID_INDEX)
    {
        yError() << "[MasImuTest::MasImuData::setupModel] The frame " << m_frameName << " does not exists in the robot model."
                 << ". Configuration failed.";
        return false;
    }

    m_link = m_commonDataPtr->fullModel.getFrameLink(m_frame);
    assert(m_link != iDynTree::LINK_INVALID_INDEX);

    m_consideredJointIndexes.clear();
    m_consideredJointNames.clear();

    iDynTree::LinkIndex baseLinkIndex = m_commonDataPtr->traversal.getBaseLink()->getIndex();
    iDynTree::LinkIndex currentLink = m_link;
    while (currentLink != baseLinkIndex) {
        const iDynTree::IJoint* joint = m_commonDataPtr->traversal.getParentJointFromLinkIndex(currentLink);
        assert(joint);
        m_consideredJointIndexes.push_back(joint->getIndex());
        m_consideredJointNames.push_back(m_commonDataPtr->fullModel.getJointName(m_consideredJointIndexes.back()));
        currentLink = m_commonDataPtr->traversal.getParentLinkFromLinkIndex(currentLink)->getIndex();
    }

    iDynTree::ModelLoader  reducedModelLoader;
    ok = reducedModelLoader.loadReducedModelFromFullModel(m_commonDataPtr->fullModel, m_consideredJointNames);

    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setupModel] Failed to build the reduced model. Configuration failed.";
        return false;
    }

    ok = m_kinDyn.loadRobotModel(reducedModelLoader.model());

    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setupModel] Failed to load the reduced model. Configuration failed.";
        return false;
    }

    return true;
}

bool MasImuTest::MasImuData::setupOrientationSensors()
{
    std::string remote;
    bool ok = m_group->getParameter("remote",remote);
    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setupOrientationSensors] Setup failed.";
        return false;
    }

    yarp::os::Property inertialClientProperty;
    inertialClientProperty.put("remote", "/" + m_commonDataPtr->robotName + "/" + remote);
    inertialClientProperty.put("local", "/" + m_commonDataPtr->prefix + "/" + remote);
    inertialClientProperty.put("device","multipleanalogsensorsclient");

    if (!m_orientationDriver.open(inertialClientProperty))
    {
        yError() << "[MasImuTest::MasImuData::setupOrientationSensors] Failed to open multipleanalogsensorsclient on remote "
                 << remote << ". Setup failed.";
        return false;
    }

    if (!m_orientationDriver.view(m_orientationInterface) || !m_orientationInterface)
    {
        yError() << "[MasImuTest::MasImuData::setupOrientationSensors] Failed to open multipleanalogsensorsclient on remote "
                 << remote << ". Setup failed.";
        return false;
    }

    m_sensorIndex = 0;
    std::string name;
    bool found = false;
    do
    {
        bool ok = m_orientationInterface->getOrientationSensorFrameName(m_sensorIndex, name);
        if (ok)
        {
            found = name == m_frameName;

            if (!found)
            {
                m_sensorIndex++;
            }
        }
    }
    while (ok && (m_sensorIndex < m_orientationInterface->getNrOfOrientationSensors()) && !found);

    if (!found)
    {
        yError() << "[MasImuTest::MasImuData::setupOrientationSensors] The interface contains no orientation sensors on frame "
                 << m_frameName << ". Setup failed.";
        return false;
    }

    return true;
}

bool MasImuTest::MasImuData::setupEncoders()
{
    std::vector<std::string> inputControlBoards;
    bool ok = m_group->getParameter("remote_control_boards", inputControlBoards);
    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setupEncoders] Setup failed.";
        return false;
    }

    // open the remotecontrolboardremepper YARP device
    yarp::os::Property remapperOptions;
    remapperOptions.put("device", "remotecontrolboardremapper");

    YarpUtilities::addVectorOfStringToProperty(remapperOptions, "axesNames", m_consideredJointNames);

    // prepare the remotecontrolboards
    yarp::os::Bottle remoteControlBoardsYarp;
    yarp::os::Bottle& remoteControlBoardsYarpList = remoteControlBoardsYarp.addList();
    for (auto& rcb : inputControlBoards)
        remoteControlBoardsYarpList.addString("/" + m_commonDataPtr->robotName + "/" + rcb);

    remapperOptions.put("remoteControlBoards", remoteControlBoardsYarp.get(0));
    remapperOptions.put("localPortPrefix", "/" + m_commonDataPtr->prefix + "/remoteControlBoard");

    // open the device
    if (!m_robotDriver.open(remapperOptions))
    {
        yError() << "[MasImuTest::MasImuData::setupEncoders] Could not open remotecontrolboardremapper object. Setup failed.";
        return false;
    }

    if(!m_robotDriver.view(m_encodersInterface) || !m_encodersInterface)
    {
        yError() << "[MasImuTest::MasImuData::setupEncoders] Cannot obtain IEncoders interface. Setup failed.";
        return false;
    }

    return true;
}

bool MasImuTest::MasImuData::setup(ParametersHandler::YarpImplementation::shared_ptr group,
                                   std::shared_ptr<CommonData> commonDataPtr)
{

    m_commonDataPtr = commonDataPtr;
    m_group = group;

    bool ok = setupModel();
    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setup] setupModel failed.";
        return false;
    }

    ok = setupOrientationSensors();
    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setup] setupOrientationSensors failed.";
        return false;
    }

    ok = setupEncoders();
    if (!ok)
    {
        yError() << "[MasImuTest::MasImuData::setup] setupEncoders failed.";
        return false;
    }

    return true;
}

void MasImuTest::MasImuData::reset()
{

}

bool MasImuTest::MasImuData::close()
{
    if(!m_orientationDriver.close())
    {
        yError() << "[MasImuTest::MasImuData::close] Unable to close the orientation driver.";
        return false;
    }

    if(!m_robotDriver.close())
    {
        yError() << "[MasImuTest::MasImuData::close] Unable to close the robot driver.";
        return false;
    }
}


void MasImuTest::reset()
{
    m_state = State::PREPARED;
    m_leftIMU.reset();
    m_rightIMU.reset();
}

double MasImuTest::getPeriod()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    return m_period;
}

bool MasImuTest::updateModule()
{
    std::lock_guard<std::mutex> lock(m_mutex);

    if (m_state == State::RUNNING)
    {

    }

    if (m_state == State::FIRST_RUN)
    {

        m_state = State::RUNNING;
    }

    return true;
}

bool MasImuTest::configure(yarp::os::ResourceFinder &rf)
{
    std::lock_guard<std::mutex> lock(m_mutex);

    m_parametersPtr = BipedalLocomotionControllers::ParametersHandler::YarpImplementation::make_unique(rf);
    m_commonDataPtr = std::make_shared<CommonData>();

    bool ok = m_parametersPtr->getParameter("name", m_commonDataPtr->prefix);
    if (!ok)
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }

    ok = m_parametersPtr->getParameter("period", m_period);
    if (!ok)
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }

    if (m_period < 0)
    {
        yError() << "[MasImuTest::configure] The period cannot be negative. Configuration failed.";
        return false;
    }

    ok = m_parametersPtr->getParameter("robot", m_commonDataPtr->robotName);
    if (!ok)
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }
    std::string robotModelName;
    ok = m_parametersPtr->getParameter("model", robotModelName);
    if (!ok)
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }
    std::string pathToModel = yarp::os::ResourceFinder::getResourceFinderSingleton().findFileByName(robotModelName);
    iDynTree::ModelLoader modelLoader;
    if (!modelLoader.loadModelFromFile(pathToModel))
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }

    m_commonDataPtr->fullModel = modelLoader.model();

    std::string baseLink;
    ok = m_parametersPtr->getParameter("base_link", baseLink);
    if (!ok)
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }

    iDynTree::LinkIndex baseLinkIndex = m_commonDataPtr->fullModel.getLinkIndex(baseLink);
    if (baseLinkIndex == iDynTree::LINK_INVALID_INDEX)
    {
        yError() << "[MasImuTest::configure] The link " << baseLink << " does not exists in " << robotModelName
                 << ". Configuration failed.";
        return false;
    }

    ok = m_commonDataPtr->fullModel.computeFullTreeTraversal(m_commonDataPtr->traversal, baseLinkIndex);

    if (!ok)
    {
        yError() << "[MasImuTest::configure] Failed to build the traversal. Configuration failed.";
        return false;
    }

    if(!iDynTree::parseRotationMatrix(rf, "base_rotation", m_commonDataPtr->baseRotation))
    {
        m_commonDataPtr->baseRotation = iDynTree::Rotation::Identity();
        yInfo() << "Using the identity as desired rotation for the additional frame";
    }

    ok = m_parametersPtr->getParameter("filter_yaw", m_commonDataPtr->filterYaw);
    if (!ok)
    {
        yError() << "[MasImuTest::configure] Configuration failed.";
        return false;
    }

    auto leftLegGroup = m_parametersPtr->getGroup("LEFT_LEG").lock();
    if (!leftLegGroup)
    {
        yError() << "[MasImuTest::configure] LEFT_LEG group not available. Configuration failed.";
        return false;
    }

    m_leftIMU.setup(leftLegGroup, m_commonDataPtr);

    auto rightLegGroup = m_parametersPtr->getGroup("RIGHT_LEG").lock();
    if (!leftLegGroup)
    {
        yError() << "[MasImuTest::configure] RIGHT_LEG group not available. Configuration failed.";
        return false;
    }

    m_rightIMU.setup(rightLegGroup, m_commonDataPtr);

    // open RPC port for external command
    std::string rpcPortName = "/" + m_commonDataPtr->prefix + "/rpc";
    this->yarp().attachAsServer(this->m_rpcPort);
    if(!m_rpcPort.open(rpcPortName))
    {
        yError() << "[MasImuTest::configure] Could not open" << rpcPortName << " RPC port.";
        return false;
    }

    m_state = State::PREPARED;

    return true;
}

bool MasImuTest::close()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    bool okL = m_leftIMU.close();
    if (!okL)
    {
        yError() << "Failed to close left leg part.";
    }
    bool okR = m_rightIMU.close();
    if (!okR)
    {
        yError() << "Failed to close right leg part.";
    }

    return okL && okR;

}

bool MasImuTest::startTest()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    if (m_state == State::PREPARED)
    {
        reset();
        m_state = State::FIRST_RUN;
        return true;
    }

    return false;

}

void MasImuTest::stopTest()
{
    std::lock_guard<std::mutex> lock(m_mutex);
    m_state = State::PREPARED;
}

