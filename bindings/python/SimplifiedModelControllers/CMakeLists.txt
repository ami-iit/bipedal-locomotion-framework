# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::SimplifiedModelControllers)

  set(H_PREFIX include/BipedalLocomotion/bindings/SimplifiedModelControllers)

  add_bipedal_locomotion_python_module(
    NAME SimplifiedModelControllersBindings
    SOURCES src/CoMZMPController.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/CoMZMPController.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::SimplifiedModelControllers
    TESTS tests/test_com_zmp_controller.py)

endif()
