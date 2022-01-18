# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_System)

  set(H_PREFIX include/BipedalLocomotion/bindings/System)

  add_bipedal_locomotion_python_module(
    NAME SystemBindings
    SOURCES src/VariablesHandler.cpp src/LinearTask.cpp src/Module.cpp src/ITaskControllerManager.cpp src/IClock.cpp src/Clock.cpp
    HEADERS ${H_PREFIX}/VariablesHandler.h ${H_PREFIX}/LinearTask.h ${H_PREFIX}/ITaskControllerManager.h ${H_PREFIX}/ILinearTaskSolver.h ${H_PREFIX}/IClock.h ${H_PREFIX}/Clock.h
    LINK_LIBRARIES BipedalLocomotion::System
    TESTS tests/test_variables_handler.py
    )

  if(FRAMEWORK_COMPILE_YarpImplementation)

    add_bipedal_locomotion_python_module(
      NAME  SystemYarpImplementationBindings
      SOURCES src/YarpClock.cpp src/YarpModule.cpp
      HEADERS ${H_PREFIX}/YarpClock.h ${H_PREFIX}/YarpModule.h
      LINK_LIBRARIES BipedalLocomotion::SystemYarpImplementation
      )

  endif()

endif()
