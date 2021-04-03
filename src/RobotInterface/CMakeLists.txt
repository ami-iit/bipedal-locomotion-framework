# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/BipedalLocomotion/RobotInterface)

if (FRAMEWORK_COMPILE_RobotInterface)

add_bipedal_locomotion_library(
    NAME                   RobotInterface
    SOURCES                src/IRobotControl.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/ISensorBridge.h ${H_PREFIX}/IRobotControl.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging
    SUBDIRECTORIES         YarpImplementation tests)
endif()

if (FRAMEWORK_COMPILE_Perception)

add_bipedal_locomotion_library(
    NAME                   PerceptionInterface
    IS_INTERFACE
    PUBLIC_HEADERS         ${H_PREFIX}/ICameraBridge.h ${H_PREFIX}/IPointCloudBridge.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler ${OpenCV_LIBS} ${PCL_LIBRARIES}
    INSTALLATION_FOLDER    RobotInterface)
endif()
