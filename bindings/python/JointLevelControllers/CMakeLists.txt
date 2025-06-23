# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::JointLevelControllers)
  set(H_PREFIX include/BipedalLocomotion/bindings/JointLevelControllers)

  add_bipedal_locomotion_python_module(
    NAME JointLevelControllersBindings
    SOURCES src/PositionToCurrentController.cpp src/EMAWithLimits.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/PositionToCurrentController.h ${H_PREFIX}/EMAWithLimits.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::JointLevelControllers
    )
endif()

