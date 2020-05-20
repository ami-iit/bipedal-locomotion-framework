# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
add_bipedal_locomotion_library(
    NAME           System
    PUBLIC_HEADERS include/BipedalLocomotion/System/Advanceable.h  include/BipedalLocomotion/System/DynamicalSystem.h include/BipedalLocomotion/System/FloatingBaseDynamicalSystem.h include/BipedalLocomotion/System/DynamicalSystem.tpp include/BipedalLocomotion/System/ContactWrench.h include/BipedalLocomotion/System/LinearTimeInvariantSystem.h
    SOURCES                src/FloatingBaseDynamicalSystem.cpp src/ContactWrench.cpp src/LinearTimeInvariantSystem.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler
    DEPENDS_ON_EIGEN_PRIVATE
    )
