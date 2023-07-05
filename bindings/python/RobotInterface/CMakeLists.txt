# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::RobotInterface AND TARGET BipedalLocomotion::RobotInterfaceYarpImplementation)

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME RobotInterfaceBindings
    SOURCES src/Polydriver.cpp src/RobotControl.cpp src/SensorBridge.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/Polydriver.h ${H_PREFIX}/RobotControl.h ${H_PREFIX}/SensorBridge.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::RobotInterface BipedalLocomotion::RobotInterfaceYarpImplementation
    )

endif()
