# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpRobotLoggerDevice)
  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name

  add_bipedal_yarp_device(
    NAME YarpRobotLoggerDevice
    TYPE BipedalLocomotion::YarpRobotLoggerDevice
    SOURCES src/YarpRobotLoggerDevice.cpp
    PUBLIC_HEADERS include/BipedalLocomotion/YarpRobotLoggerDevice.h
    PUBLIC_LINK_LIBRARIES
      Eigen3::Eigen
      YARP::YARP_os
      YARP::YARP_dev
      YARP::YARP_telemetry
      BipedalLocomotion::TextLogging
      BipedalLocomotion::YarpUtilities
      BipedalLocomotion::VectorsCollection
      BipedalLocomotion::ParametersHandlerYarpImplementation
      BipedalLocomotion::RobotInterfaceYarpImplementation
      BipedalLocomotion::TextLoggingYarpImplementation
      BipedalLocomotion::SystemYarpImplementation
    CONFIGURE_PACKAGE_NAME yarp_robot_logger_device)
endif()

