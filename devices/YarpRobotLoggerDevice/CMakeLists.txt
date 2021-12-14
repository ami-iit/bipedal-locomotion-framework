# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_YarpUtilities)
# Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name

find_package(YARP COMPONENTS telemetry)


add_bipedal_yarp_device(
  NAME YarpRobotLoggerDevice
  TYPE BipedalLocomotion::YarpRobotLoggerDevice
  SOURCES src/YarpRobotLoggerDevice.cpp
  PUBLIC_HEADERS include/BipedalLocomotion/YarpRobotLoggerDevice.h
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES}
    BipedalLocomotion::YarpUtilities
    BipedalLocomotion::VectorsCollection
    BipedalLocomotion::ParametersHandlerYarpImplementation
    BipedalLocomotion::RobotInterfaceYarpImplementation
    BipedalLocomotion::matioCppConversions
    BipedalLocomotion::TextLogging
    BipedalLocomotion::TextLoggingYarpImplementation
    Eigen3::Eigen
    BipedalLocomotion::SystemYarpImplementation
    YARP::YARP_telemetry
  CONFIGURE_PACKAGE_NAME yarp_robot_logger_test_device)
endif()

