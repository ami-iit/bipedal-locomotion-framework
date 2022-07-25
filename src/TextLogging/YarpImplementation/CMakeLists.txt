# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_YarpImplementation)

  add_bipedal_locomotion_library(
    NAME                   TextLoggingYarpImplementation
    SOURCES                src/YarpLogger.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/TextLogging/YarpLogger.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::TextLogging YARP::YARP_os
    INSTALLATION_FOLDER    TextLogging)

endif()
