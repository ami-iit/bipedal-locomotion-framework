# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# set target name
if(FRAMEWORK_COMPILE_ContactModels)

  add_bipedal_locomotion_library(
    NAME                   ContactModels
    SOURCES                src/ContactModel.cpp src/ContinuousContactModel.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/ContactModels/ContactModel.h include/BipedalLocomotion/ContactModels/ContinuousContactModel.h
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core BipedalLocomotion::ParametersHandler
    PRIVATE_LINK_LIBRARIES Eigen3::Eigen
    SUBDIRECTORIES         tests)

endif()
