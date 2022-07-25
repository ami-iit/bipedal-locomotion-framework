/**
 * @file RosPublisher.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include "BipedalLocomotion/YarpUtilities/RosPublisher.h"

#include <yarp/os/Time.h>
#include <yarp/os/Node.h>
#include <yarp/os/Publisher.h>

#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/IFrameTransform.h>
#include <yarp/sig/Matrix.h>

#include <yarp/rosmsg/TickTime.h>
#include <yarp/rosmsg/sensor_msgs/JointState.h>
#include <yarp/rosmsg/geometry_msgs/WrenchStamped.h>

#include <unordered_map>
#include <chrono>

using namespace BipedalLocomotion::YarpUtilities;
using namespace BipedalLocomotion;

template <typename PublisherPtr, typename Msg>
struct PublisherDetails
{
    PublisherPtr ptr{nullptr}; /**< YARP object for a ROS  publisher */
    std::string topic{"/topic"}; /**< topic over which the message is published */
    Msg msg; /**< YARP object for a ROS message */
};

using JointStateMsg = yarp::rosmsg::sensor_msgs::JointState;
using JointStatePublisherPtr = std::unique_ptr< yarp::os::Publisher<JointStateMsg> >;
using WrenchStampedMsg = yarp::rosmsg::geometry_msgs::WrenchStamped;
using WrenchStampedPublisherPtr = std::unique_ptr< yarp::os::Publisher<WrenchStampedMsg> >;
using WrenchPublisherDetails = PublisherDetails<WrenchStampedPublisherPtr, WrenchStampedMsg>;
using JointStatePublisherDetails = PublisherDetails<JointStatePublisherPtr, JointStateMsg>;

class RosPublisher::Impl
{
public:
    /**
     * Get ROS timestamp from Yarp time now
     */
    yarp::rosmsg::TickTime getTimeStampFromYarp();

    /**
     * Open the ROS topic for the corresponding publisher
     * In ROS terms, advertise the topic over which the node is going to publish
     */
    template <typename Publisher>
    bool openPublisher(Publisher* pub, const std::string& topicName)
    {
        std::string_view printPrefix = "[RosPublisher::Impl::openPublisher] ";
        if (!pub->topic(topicName))
        {
            std::cerr << printPrefix << "Could not open ROS topic " << topicName << std::endl;
            return false;
        }
        return true;
    }


    template <typename PublisherMsg>
    void configurePublisher(std::unique_ptr<yarp::os::Publisher<PublisherMsg> >& ptr)
    {
        if (ptr == nullptr)
        {
            ptr = std::make_unique< yarp::os::Publisher <PublisherMsg> >();
        }

        ptr->close();
    }


    /**
     * Utility function to load vector parameters
     */
    template<typename Scalar>
    bool setupParamV(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler,
                     const std::string& param, std::vector<Scalar>& vec, const std::string& prefix)
    {
        auto handle = handler.lock();
        if (handle == nullptr)
        {
            return false;
        }

        if (!handle->getParameter(param, vec))
        {
            std::cerr << prefix << "The parameter handler could not find \"" << param << "\" in the configuration file. This is a required parameter." << std::endl;
            return false;
        }
        return true;
    }

    void resizeJointStateBuffer(const std::size_t& size);

    std::string nodeName; /**< name of the ROS node */
    std::unique_ptr<yarp::os::Node> node; /**< YARP object for a ROS node */

    JointStatePublisherDetails jointStatePublisher;
    std::unordered_map<std::string, WrenchPublisherDetails> wrenchPublisherMap; /**< map of wrench frame and corresponding publisher*/

    yarp::dev::PolyDriver transformBroadcaster; /**< YARP Polydriver object to open a transform client */
    yarp::dev::IFrameTransform* transformInterface{nullptr}; /**< YARP IFrameTransform interface to publish transforms */

    bool publishTF{false}; /**< flag to enable publishing transforms to YARP transform server */
    bool initialized{false}; /**< flag to chekc if the publisher was initialized */

    // placeholder variables
    yarp::sig::Matrix pose; /**< placeholder variable for publishing transform data*/
    std::vector<double> jointStateZero; /**< placeholder variable for zero joint state data*/
};

RosPublisher::RosPublisher(const std::string& nodeName) : m_pimpl(std::make_unique<Impl>())
{
    m_pimpl->nodeName = nodeName;
    m_pimpl->node = std::make_unique<yarp::os::Node>(m_pimpl->nodeName);

    m_pimpl->pose.resize(4, 4);

    std::cout <<  "[RosPublisher] Ensure roscore is running and yarpserver was run with --ros option." << std::endl;
}

RosPublisher::~RosPublisher() = default;

bool RosPublisher::initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    std::string printPrefix{"[RosPublisher::initialize] "};
    bool ok{true};

    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << printPrefix << "The parameter handler has expired. Please check its scope." << std::endl;
        return false;
    }

    // configure joint state publisher if it exists
    if (handle->getParameter("joint_states_topic", m_pimpl->jointStatePublisher.topic))
    {
        if (!configureJointStatePublisher(m_pimpl->jointStatePublisher.topic))
        {
            std::cerr << printPrefix << "Could not configure joint states publisher." << std::endl;
            return false;
        }
    }

    // configure transform publisher
    std::string tfServerPort;
    if (handle->getParameter("transform_server_port", tfServerPort))
    {
        m_pimpl->publishTF = true;
        if (!configureTransformServer(tfServerPort))
        {
            return false;
        }
    }

    auto wrenchHandle = handle->getGroup("WrenchPublishers");
    auto wrenchHandler = wrenchHandle.lock();
    if (wrenchHandler != nullptr)
    {
        std::vector<std::string> frames;
        if (!m_pimpl->setupParamV(wrenchHandler, "frame_names", frames, printPrefix))
        {
            return false;
        }

        std::vector<std::string> topics;
        if (!m_pimpl->setupParamV(wrenchHandler, "topics", topics, printPrefix))
        {
            return false;
        }

        if (frames.size() != topics.size())
        {
            std::cerr << printPrefix << " WrenchPublishers group: frame_names and topics must be of same size." << std::endl;
            return false;
        }

        for (std::size_t idx = 0; idx < frames.size(); idx++)
        {
            if (!configureWrenchPublisher(frames[idx], topics[idx]))
            {
                std::cerr << printPrefix << "Could not configure wrench publisher for frame: "
                          << frames[idx] << " and topic: " << topics[idx] << std::endl;
                return false;
            }
        }
    }

    if (!ok)
    {
        std::cerr << printPrefix << "Failed to initialize the ROSPublisher" << std::endl;
        return false;
    }

    m_pimpl->initialized = true;

    return true;
}

bool RosPublisher::configureJointStatePublisher(const std::string& topicName)
{
    m_pimpl->configurePublisher(m_pimpl->jointStatePublisher.ptr);

    m_pimpl->jointStatePublisher.topic = topicName;
    if (!m_pimpl->openPublisher(m_pimpl->jointStatePublisher.ptr.get(), m_pimpl->jointStatePublisher.topic))
    {
        return false;
    }

    return true;
}

bool RosPublisher::configureWrenchPublisher(const std::string& frameName,
                                            const std::string& topicName)
{
    std::string_view printPrefix = "[RosPublisher::configureWrenchPublisher] ";
    if (m_pimpl->wrenchPublisherMap.find(frameName) == m_pimpl->wrenchPublisherMap.end())
    {
        std::cerr << printPrefix << "Wrench publisher does not already exist. Adding a wrench publisher for " << frameName << "." << std::endl;
        m_pimpl->wrenchPublisherMap[frameName] = WrenchPublisherDetails();
    }

    m_pimpl->configurePublisher(m_pimpl->wrenchPublisherMap.at(frameName).ptr);
    m_pimpl->wrenchPublisherMap.at(frameName).topic = topicName;
    if (!m_pimpl->openPublisher(m_pimpl->wrenchPublisherMap.at(frameName).ptr.get(), m_pimpl->wrenchPublisherMap.at(frameName).topic))
    {
        return false;
    }

    return true;
}

bool RosPublisher::removeWrenchPublisher(const std::string& frameName)
{
    std::string_view printPrefix = "[RosPublisher::removeWrenchPublisher] ";
    if (m_pimpl->wrenchPublisherMap.find(frameName) == m_pimpl->wrenchPublisherMap.end())
    {
        std::cerr << printPrefix << "Wrench publisher does not already exist." << std::endl;
        return false;
    }

    m_pimpl->wrenchPublisherMap.at(frameName).ptr->close();
    m_pimpl->wrenchPublisherMap.at(frameName).ptr = nullptr;
    m_pimpl->wrenchPublisherMap.at(frameName).msg.clear();

    m_pimpl->wrenchPublisherMap.erase(frameName);
    return true;
}

bool RosPublisher::configureTransformServer(const std::string& transformServerPort)
{
    if (m_pimpl->transformBroadcaster.isValid())
    {
        m_pimpl->transformBroadcaster.close();
        m_pimpl->transformInterface = nullptr;
    }

    yarp::os::Property tfBroadcasterSettings{{"device", yarp::os::Value("transformClient")},
                                             {"remote", yarp::os::Value(transformServerPort)},
                                             {"local", yarp::os::Value(m_pimpl->nodeName + "/transformClient")}};

    std::string_view printPrefix = "[RosPublisher::openTransformBroadcaster] ";
    if (!m_pimpl->transformBroadcaster.open(tfBroadcasterSettings))
    {
        std::cerr << printPrefix << "Unable to open transform broadcaster. Ensure transform server is already running." << std::endl;
        return false;
    }

    if (!m_pimpl->transformBroadcaster.view(m_pimpl->transformInterface))
    {
        std::cerr << printPrefix << "Unable to access transform interface." << std::endl;
        return false;
    }

    return true;
}

bool RosPublisher::publishTransform(const std::string& target,
                                    const std::string& source,
                                    const BipedalLocomotion::GenericContainer::Vector<const double>::Ref transformAsVector16d)
{
    std::string_view printPrefix = "[RosPublisher::publishTransform] ";

    if (!m_pimpl->initialized)
    {
        std::cerr << printPrefix << "Please call initialize before publishing." << std::endl;
        return false;
    }

    if (!m_pimpl->publishTF || m_pimpl->transformInterface == nullptr)
    {
        std::cerr << printPrefix << "Transform broadcaster was not configured. Unable to publish transforms." << std::endl;
        return false;
    }

    if (transformAsVector16d.size() != 16)
    {
        std::cerr << printPrefix << "Malformed Transform data. Expecting a 16d vector." << std::endl;
        return false;
    }

    for (size_t i = 0; i < 4; i++)
    {
        for (size_t j = 0; j < 4; j++)
        {
            m_pimpl->pose(i, j) = transformAsVector16d( (i*4) + j);
        }
    }

    if (!m_pimpl->transformInterface->setTransform(target, source, m_pimpl->pose))
    {
        std::cerr << printPrefix << "Could not publish transform for source: "
                  << source << " and target: " << target << " frames." << std::endl;
        return false;
    }

    return true;
}

void RosPublisher::Impl::resizeJointStateBuffer(const std::size_t& size)
{
    if (jointStateZero.size() != size)
    {
        jointStateZero.resize(size, 0.0);
    }
}

bool RosPublisher::publishJointStates(const GenericContainer::Vector<const std::string>::Ref jointList,
                                      const GenericContainer::Vector<const double>::Ref jointPositions)
{
    m_pimpl->resizeJointStateBuffer(jointList.size());
    return publishJointStates(jointList, jointPositions, m_pimpl->jointStateZero, m_pimpl->jointStateZero);
}

bool RosPublisher::publishJointStates(const GenericContainer::Vector<const std::string>::Ref jointList,
                                      const GenericContainer::Vector<const double>::Ref jointPositions,
                                      const GenericContainer::Vector<const double>::Ref jointVelocities)
{
    m_pimpl->resizeJointStateBuffer(jointList.size());
    return publishJointStates(jointList, jointPositions, jointVelocities, m_pimpl->jointStateZero);
}

bool RosPublisher::publishJointStates(const GenericContainer::Vector<const std::string>::Ref jointList,
                                      const GenericContainer::Vector<const double>::Ref jointPositions,
                                      const GenericContainer::Vector<const double>::Ref jointVelocities,
                                      const GenericContainer::Vector<const double>::Ref jointEfforts)
{
    std::string_view printPrefix = "[RosPublisher::publishJointStates] ";

    if (!m_pimpl->initialized)
    {
        std::cerr << printPrefix << "Please call initialize before publishing." << std::endl;
        return false;
    }

    auto& pub = m_pimpl->jointStatePublisher;
    if (pub.ptr == nullptr)
    {
        std::cerr << printPrefix << "Joint state publisher was not configured. Unable to publish joint states." << std::endl;
        return false;
    }

    if ( (jointList.size() != jointPositions.size()) ||
         (jointList.size() != jointVelocities.size()) ||
         (jointList.size() != jointEfforts.size()))
    {
        std::cerr << printPrefix << "Joint states size mismatch. Unable to publish joint states." << std::endl;
        return false;
    }

    pub.msg.header.seq++;
    pub.msg.header.stamp = m_pimpl->getTimeStampFromYarp();

    pub.msg.name.clear();
    pub.msg.position.clear();
    pub.msg.velocity.clear();
    pub.msg.effort.clear();

    for (std::size_t idx = 0; idx < jointList.size(); idx++)
    {
        pub.msg.name.emplace_back(jointList(idx));
        pub.msg.position.emplace_back(jointPositions(idx));
        pub.msg.velocity.emplace_back(jointVelocities(idx));
        pub.msg.effort.emplace_back(jointEfforts(idx));
    }

    pub.ptr->write(pub.msg);
    return true;
}

bool RosPublisher::publishWrench(const std::string& frame,
                                 BipedalLocomotion::GenericContainer::Vector<const double>::Ref wrench6d)
{
    std::string_view printPrefix = "[RosPublisher::publishWrench] ";

    if (!m_pimpl->initialized)
    {
        std::cerr << printPrefix << "Please call initialize before publishing." << std::endl;
        return false;
    }

    if (m_pimpl->wrenchPublisherMap.find(frame) == m_pimpl->wrenchPublisherMap.end())
    {
        std::cerr << printPrefix << "Frame does not exist. Please add wrench publisher before publishing." << std::endl;
        return false;
    }

    auto& pub = m_pimpl->wrenchPublisherMap.at(frame);
    if (pub.ptr == nullptr)
    {
        std::cerr << printPrefix << "Wrench publisher was not configured. Unable to publish wrench stamped messages." << std::endl;
        return false;
    }

    if (wrench6d.size() != 6)
    {
        std::cerr << printPrefix << "Expecting a 6d input vector. Unable to publish wrench." << std::endl;
        return false;
    }

    pub.msg.header.seq++;
    pub.msg.header.frame_id = frame;
    pub.msg.header.stamp = m_pimpl->getTimeStampFromYarp();
    pub.msg.wrench.force.x = wrench6d(0);
    pub.msg.wrench.force.y = wrench6d(1);
    pub.msg.wrench.force.z = wrench6d(2);

    pub.msg.wrench.torque.x = wrench6d(3);
    pub.msg.wrench.torque.y = wrench6d(4);
    pub.msg.wrench.torque.z = wrench6d(5);
    pub.ptr->write(pub.msg);

    return true;
}

yarp::rosmsg::TickTime RosPublisher::Impl::getTimeStampFromYarp()
{
    std::string_view printPrefix = "[RosPublisher::Impl::getTimeStampFromYarp] ";
    yarp::rosmsg::TickTime rosTickTime;

    double yarpTimeNow{yarp::os::Time::now()};
    std::chrono::duration<double> timeStamp(yarpTimeNow);
    uint64_t secPart = std::chrono::duration_cast<std::chrono::seconds>(timeStamp).count();
    uint64_t nsecPart = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::duration<double>(yarpTimeNow - secPart)).count();

    if (secPart > std::numeric_limits<unsigned int>::max()) {
        std::cerr << printPrefix
                  << "Timestamp exceeded the 64 bit representation, resetting it to 0" << std::endl;
        secPart = 0;
    }

    rosTickTime.sec = static_cast<unsigned>(secPart);
    rosTickTime.nsec = static_cast<unsigned>(nsecPart);

    return rosTickTime;
}


void RosPublisher::stop()
{
    if (!m_pimpl->wrenchPublisherMap.empty())
    {
        std::vector<std::string> frames;
        // use two loops to avoid segfaults due to map::erase in a for loop
        // avoid iterator invalidation
        for (auto& pair : m_pimpl->wrenchPublisherMap)
        {
            frames.push_back(pair.first);
        }

        for (auto& frame : frames)
        {
            removeWrenchPublisher(frame);
        }
    }

    m_pimpl->wrenchPublisherMap.clear();
    if (m_pimpl->jointStatePublisher.ptr != nullptr)
    {
        m_pimpl->jointStatePublisher.ptr->close();
        m_pimpl->jointStatePublisher.ptr = nullptr;
    }

    if (m_pimpl->transformBroadcaster.isValid())
    {
        m_pimpl->transformBroadcaster.close();
    }
}
