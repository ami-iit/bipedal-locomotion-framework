# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_RosImplementation)

  add_bipedal_locomotion_library(
    NAME                   TextLoggingRosImplementation
    SOURCES                src/RosLogger.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/TextLogging/RosLogger.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::TextLogging rclcpp::rclcpp
    INSTALLATION_FOLDER    TextLogging)

endif()
