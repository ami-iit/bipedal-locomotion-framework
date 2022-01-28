# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_YarpUtilities)
  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name

  add_bipedal_yarp_device(
    NAME YarpCameraBridgeArucoTestDevice
    TYPE BipedalLocomotion::YarpCameraBridgeArucoTestDevice
    SOURCES src/YarpCameraBridgeArucoTestDevice.cpp
    PUBLIC_HEADERS include/BipedalLocomotion/YarpCameraBridgeArucoTestDevice.h
    PUBLIC_LINK_LIBRARIES
      Eigen3::Eigen
      YARP::YARP_os
      YARP::YARP_dev
      BipedalLocomotion::PerceptionInterface
      BipedalLocomotion::TextLogging
      BipedalLocomotion::YarpUtilities
      BipedalLocomotion::ParametersHandlerYarpImplementation
      BipedalLocomotion::RobotInterfaceYarpImplementation
      BipedalLocomotion::PerceptionInterfaceYarpImplementation
      BipedalLocomotion::PerceptionFeatures
      BipedalLocomotion::TextLoggingYarpImplementation
      BipedalLocomotion::SystemYarpImplementation
    CONFIGURE_PACKAGE_NAME yarp_camera_bridge_aruco_test_device)
endif()

