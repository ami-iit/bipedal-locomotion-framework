/**
 * @file RosPublisher.h
 * @authors Prashanth Ramadoss
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_YARP_UTILITIES_ROS_PUBLISHER_H
#define BIPEDAL_LOCOMOTION_YARP_UTILITIES_ROS_PUBLISHER_H

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/GenericContainer/Vector.h>
#include <memory>
#include <iostream>
#include <string>
namespace BipedalLocomotion
{

/**
 * Helper for YARP library.
 */
namespace YarpUtilities
{

/**
 * The class internally contains a YARP based ROS node and a set of publishers.
 * Current implementation consists of
 *  - Joint states publisher
 *  - Wrenches publisher
 *  - Transform broadcaster
 * Although the class might be ROS independent, in order to run the code, ROS is required and usual YARP-ROS connections need to be made.
 */
class [[deprecated("The ROS 1-based BipedalLocomotion::YarpUtilities::RosPublisher is deprecated, use ROS 2 instead.")]] RosPublisher
{
public:
    /**
     * Constructor
     * @param[in] nodeName Name of the ROS publisher node
     */
    RosPublisher(const std::string& nodeName);

    /**
     * Destructor
     */
    ~RosPublisher();

    /**
     * Configures the publishers and ROS topics for publishing messages
     * The parameters used to configure the RosPublisher,
     * - "joint_states_topic" name of the topic over which the joint states need to be published
     * - "transform_server_port" name of the already running transform server's remote port
     * - "WrenchPublishers" is group with the following parameters,
     *      - "frame_names" a list containing the frames at which the published wrenches should be expressed
     *      - "topics" a list containing the topics over which the wrenches need to be published. Must be the same size and order as frame_names.
     * @param[in] handler Parameter handler
     * @note this method needs to be called after construction and before publishing
     */
    bool initialize(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler);

    /**
     * Clear internal configuration and close the node
     */
    void stop();

    /**
     * Publish the joint states over the configured joint states topic
     * The joint velocities and joint efforts are set to zero.
     * @param[in] jointList list of joints
     * @param[in] jointPositions list of joint positions in m or rad with the same size as the joints list
     */
    bool publishJointStates(const BipedalLocomotion::GenericContainer::Vector<const std::string>::Ref jointList,
                            const BipedalLocomotion::GenericContainer::Vector<const double>::Ref jointPositions);

    /**
     * Publish the joint states over the configured joint states topic
     * The joint velocities and joint efforts are set to zero.
     * @param[in] jointList list of joints
     * @param[in] jointPositions list of joint positions in m or rad with the same size as the joints list
     * @param[in] jointVelocities list of joint velocities in m/s or rad/s with the same size as the joints list
     */
    bool publishJointStates(const BipedalLocomotion::GenericContainer::Vector<const std::string>::Ref jointList,
                            const BipedalLocomotion::GenericContainer::Vector<const double>::Ref jointPositions,
                            const BipedalLocomotion::GenericContainer::Vector<const double>::Ref jointVelocities);

    /**
     * Publish the joint states over the configured joint states topic
     * The joint velocities and joint efforts are set to zero.
     * @param[in] jointList list of joints
     * @param[in] jointPositions list of joint positions in m or rad with the same size as the joints list
     * @param[in] jointVelocities list of joint velocities in m/s or rad/s with the same size as the joints list
     * @param[in] jointPositions list of joint torques/forces in Nm or N with the same size as the joints list
     */
    bool publishJointStates(const BipedalLocomotion::GenericContainer::Vector<const std::string>::Ref jointList,
                            const BipedalLocomotion::GenericContainer::Vector<const double>::Ref jointPositions,
                            const BipedalLocomotion::GenericContainer::Vector<const double>::Ref jointVelocities,
                            const BipedalLocomotion::GenericContainer::Vector<const double>::Ref jointEfforts);

    /**
     * Publish the wrenches over the configured wrench topic associated to the frame
     * @param[in] frameName frame at which the wrench will be expressed
     * @param[in] wrench6d 6d wrench as force-torque in N-Nm with serialziation as fx fy fz tx ty tz
     */
    bool publishWrench(const std::string& frameName,
                       BipedalLocomotion::GenericContainer::Vector<const double>::Ref wrench6d);

    /**
     * Publish transforms to the transform server
     * @param[in] target  target frame for the transform
     * @param[in] source source frame of the transform
     * @param[in] transformAsVector16d 4x4 transform as a vector data in row-major order
     */
    bool publishTransform(const std::string& target, const std::string& source,
                          const BipedalLocomotion::GenericContainer::Vector<const double>::Ref transformAsVector16d);

    /**
     * Configure which transform server to connect to
     * @param[in] transformServerPort name of the transform server port
     */
    bool configureTransformServer(const std::string& transformServerPort);

    /**
     * Configure the topic over which joint states are published
     * @param[in] topicName topic name
     */
    bool configureJointStatePublisher(const std::string& topicName);

    /**
     * Add or reconfigure a wrench publisher
     * @param[in] frameName frame name
     * @param[in] topicName topic name
     */
    bool configureWrenchPublisher(const std::string& frameName,
                                  const std::string& topicName);

    /**
     * Remove wrench publisher
     * @param[in] frameName frame name
     */
    bool removeWrenchPublisher(const std::string& frameName);

    RosPublisher(const RosPublisher& other) = delete; /**< delete copy constructor */
    RosPublisher(RosPublisher&& other) = delete; /**< delete move constructor */
    RosPublisher& operator=(const RosPublisher& other) = delete; /**< delete copy assignment operator */
    RosPublisher& operator=(RosPublisher&& other) = delete; /**< delete move assignment operator */
private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;

};

} // namespace YarpUtilities
} // namespace BipedalLocomotion
#endif // BIPEDAL_LOCOMOTION_YARP_UTILITIES_ROS_PUBLISHER_H

