# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_ReducedModelControllers)

  set(H_PREFIX include/BipedalLocomotion/ReducedModelControllers)

  add_bipedal_locomotion_library(
    NAME                   ReducedModelControllers
    PUBLIC_HEADERS         ${H_PREFIX}/ICentroidalMPC.h ${H_PREFIX}/CentroidalMPC.h ${H_PREFIX}/StableCentroidalMPC.h
    SOURCES                src/CentroidalMPC.cpp src/StableCentroidalMPC.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts
    PRIVATE_LINK_LIBRARIES casadi BipedalLocomotion::Math BipedalLocomotion::TextLogging BipedalLocomotion::CasadiConversions
    SUBDIRECTORIES         tests)

endif()

