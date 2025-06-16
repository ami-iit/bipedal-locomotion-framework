# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_JointLevelControllers)

  set(H_PREFIX include/BipedalLocomotion/JointLevelControllers)

  add_bipedal_locomotion_library(
    NAME                   JointLevelControllers
    PUBLIC_HEADERS         ${H_PREFIX}/PositionToCurrentController.h
    SOURCES                src/PositionToCurrentController.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System
    SUBDIRECTORIES         tests)

endif()
