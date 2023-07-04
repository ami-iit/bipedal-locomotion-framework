# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::ReducedModelControllers)

  set(H_PREFIX include/BipedalLocomotion/bindings/ReducedModelControllers)

  add_bipedal_locomotion_python_module(
    NAME ReducedModelControllersBindings
    SOURCES src/CentroidalMPC.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/CentroidalMPC.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::ReducedModelControllers
    TESTS tests/test_centroidal_mpc.py)

endif()
