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

  set(cvnp_target_link )

  # This compiles only if pybind11 is at least v2.7.0
  # Indeed we need a feature in pybind that has been introduced by this commit
  # https://github.com/pybind/pybind11/commit/74a767d42921001fc4569ecee3b8726383c42ad4
  # https://github.com/pybind/pybind11/pull/2864
  if (${pybind11_VERSION} VERSION_GREATER_EQUAL "2.7.0")
    include(FetchContent)
    FetchContent_Declare(
      cvnp
      GIT_REPOSITORY https://github.com/pthom/cvnp
      GIT_TAG        6aeac770ccff122abf82573da1ae03e18e7a4707
      )
    FetchContent_MakeAvailable(cvnp)
    set(cvnp_target_link cvnp)
  endif()

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME PerceptionInterfaceBindings
    SOURCES src/PerceptionModule.cpp src/CameraBridge.cpp
    HEADERS ${H_PREFIX}/PerceptionModule.h ${H_PREFIX}/CameraBridge.h
    LINK_LIBRARIES BipedalLocomotion::PerceptionInterface BipedalLocomotion::PerceptionInterfaceYarpImplementation ${cvnp_target_link}
    ADDITIONAL_COMPILE_DEFINITIONS pybind11_VERSION_MAJOR=${pybind11_VERSION_MAJOR}
                                   pybind11_VERSION_MINOR=${pybind11_VERSION_MINOR}
                                   pybind11_VERSION_PATCH=${pybind11_VERSION_PATCH}
    )

endif()
