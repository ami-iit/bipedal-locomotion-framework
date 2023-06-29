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

if(TARGET BipedalLocomotion::RobotInterface
    AND TARGET BipedalLocomotion::PerceptionInterface
    AND TARGET BipedalLocomotion::RobotInterfaceYarpImplementation
    AND TARGET BipedalLocomotion::PerceptionInterfaceYarpImplementation)

  FetchContent_Declare(
    cvnp
    GIT_REPOSITORY https://github.com/pthom/cvnp
    GIT_TAG        e91b2f04e7b7e8c2858ef4d45b434591aa6c0e25
    )

  FetchContent_MakeAvailable(cvnp)

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME PerceptionInterfaceBindings
    SOURCES src/PerceptionModule.cpp src/CameraBridge.cpp
    HEADERS ${H_PREFIX}/PerceptionModule.h ${H_PREFIX}/CameraBridge.h
    LINK_LIBRARIES BipedalLocomotion::PerceptionInterface BipedalLocomotion::PerceptionInterfaceYarpImplementation cvnp
    )

endif()
