# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_Estimators)

  add_bipedal_locomotion_library(
    NAME                   Estimators
    SOURCES                src/RecursiveLeastSquare.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/Estimators/RecursiveLeastSquare.h
    SUBDIRECTORIES         tests
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::System Eigen3::Eigen)
endif()
