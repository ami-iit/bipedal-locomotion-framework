# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_RobotInterface AND FRAMEWORK_COMPILE_YarpImplementation)

  set(H_PREFIX include/BipedalLocomotion/bindings/ReducedModelControllers)

  add_bipedal_locomotion_python_module(
    NAME ReducedModelControllersBindings
    SOURCES src/CentroidalMPC.cpp
    HEADERS ${H_PREFIX}/CentroidalMPC.h 
    LINK_LIBRARIES BipedalLocomotion::ReducedModelControllers    
    )

endif()
