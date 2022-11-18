# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_RosImplementation)

  add_bipedal_locomotion_library(
    NAME                   SystemRosImplementation
    SOURCES                src/RosClock.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/System/RosClock.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::System rclcpp::rclcpp
    INSTALLATION_FOLDER    System)

endif()
