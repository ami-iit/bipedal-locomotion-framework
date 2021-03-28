# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if(FRAMEWORK_COMPILE_System)

  set(H_PREFIX include/BipedalLocomotion/System)

  # set target name
  add_bipedal_locomotion_library(
    NAME                   System
    PUBLIC_HEADERS         ${H_PREFIX}/Advanceable.h ${H_PREFIX}/VariablesHandler.h ${H_PREFIX}/LinearTask.h ${H_PREFIX}/Block.h ${H_PREFIX}/SharedResource.h ${H_PREFIX}/BlockRunner.h
    PRIVATE_HEADERS        ${H_PREFIX}/impl/traits.h
    SOURCES                src/VariablesHandler.cpp src/LinearTask.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler Eigen3::Eigen
    SUBDIRECTORIES         tests
    )

endif()
