# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_YarpImplementation)

  add_bipedal_locomotion_library(
    NAME                   SystemYarpImplementation
    SOURCES                src/YarpClock.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/System/YarpClock.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::System YARP::YARP_init
    PRIVATE_LINK_LIBRARIES YARP::YARP_os
    INSTALLATION_FOLDER    System)

endif()
