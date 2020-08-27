# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if (FRAMEWORK_COMPILE_RobotInterface)

set(H_PREFIX include/BipedalLocomotion/RobotInterface)

add_bipedal_locomotion_library(
    NAME                   RobotInterface
    PUBLIC_HEADERS         ${H_PREFIX}/ISensorBridge.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler Eigen3::Eigen
    SUBDIRECTORIES    tests
    IS_INTERFACE)
endif()
