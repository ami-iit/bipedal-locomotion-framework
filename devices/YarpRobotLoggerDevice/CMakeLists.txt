# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_YarpRobotLoggerDevice)
  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name

  add_bipedal_yarp_device(
    NAME YarpRobotLoggerDevice
    TYPE BipedalLocomotion::YarpRobotLoggerDevice
    SOURCES src/YarpRobotLoggerDevice.cpp src/YarpTextLoggingUtilities.cpp
    PUBLIC_HEADERS include/BipedalLocomotion/YarpRobotLoggerDevice.h include/BipedalLocomotion/YarpTextLoggingUtilities.h
    PUBLIC_LINK_LIBRARIES
      Eigen3::Eigen
      YARP::YARP_os
      YARP::YARP_dev
      YARP::YARP_profiler
      robometry::robometry
      BipedalLocomotion::TextLogging
      BipedalLocomotion::System
      BipedalLocomotion::YarpUtilities
      BipedalLocomotion::VectorsCollection
      BipedalLocomotion::ParametersHandlerYarpImplementation
      BipedalLocomotion::RobotInterfaceYarpImplementation
      BipedalLocomotion::PerceptionInterfaceYarpImplementation
      BipedalLocomotion::TextLoggingYarpImplementation
      BipedalLocomotion::SystemYarpImplementation
    CONFIGURE_PACKAGE_NAME yarp_robot_logger_device)
endif()

