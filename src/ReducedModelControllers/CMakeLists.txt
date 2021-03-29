# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


# if (FRAMEWORK_COMPILE_ReducedModelControllers)

  set(H_PREFIX include/BipedalLocomotion/ReducedModelControllers)

  add_bipedal_locomotion_library(
    NAME                   ReducedModelControllers
    PUBLIC_HEADERS         ${H_PREFIX}/CentroidalMPC.h
    SOURCES                src/CentroidalMPC.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts
    PRIVATE_LINK_LIBRARIES casadi BipedalLocomotion::Math BipedalLocomotion::TextLogging
    SUBDIRECTORIES        tests)

# endif()
