# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_SimplifiedModelControllers)

  set(H_PREFIX include/BipedalLocomotion/SimplifiedModelControllers)

  add_bipedal_locomotion_library(
    NAME                   SimplifiedModelControllers
    PUBLIC_HEADERS         ${H_PREFIX}/CoMZMPController.h
    SOURCES                src/CoMZMPController.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler MANIF::manif
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging)
endif()
