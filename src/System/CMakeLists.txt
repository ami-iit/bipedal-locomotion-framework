# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


if(FRAMEWORK_COMPILE_System)

  set(H_PREFIX include/BipedalLocomotion/System)

  # set target name
  add_bipedal_locomotion_library(
    NAME                   System
    PUBLIC_HEADERS         ${H_PREFIX}/InputPort.h ${H_PREFIX}/OutputPort.h
                           ${H_PREFIX}/Advanceable.h ${H_PREFIX}/Source.h ${H_PREFIX}/Sink.h
                           ${H_PREFIX}/Factory.h
                           ${H_PREFIX}/VariablesHandler.h ${H_PREFIX}/LinearTask.h ${H_PREFIX}/ILinearTaskSolver.h ${H_PREFIX}/ILinearTaskFactory.h ${H_PREFIX}/ITaskControllerManager.h
                           ${H_PREFIX}/IClock.h ${H_PREFIX}/StdClock.h ${H_PREFIX}/Clock.h
                           ${H_PREFIX}/SharedResource.h ${H_PREFIX}/AdvanceableRunner.h
                           ${H_PREFIX}/QuitHandler.h
                           ${H_PREFIX}/Barrier.h ${H_PREFIX}/TimeProfiler.h
                           ${H_PREFIX}/WeightProvider.h ${H_PREFIX}/ConstantWeightProvider.h
    SOURCES                src/VariablesHandler.cpp src/LinearTask.cpp
                           src/StdClock.cpp src/Clock.cpp src/QuitHandler.cpp src/Barrier.cpp
                           src/ConstantWeightProvider.cpp src/TimeProfiler.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler Eigen3::Eigen
    SUBDIRECTORIES         tests YarpImplementation RosImplementation
    )

endif()
