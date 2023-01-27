# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

set(H_PREFIX include/BipedalLocomotion/bindings/TextLogging)

add_bipedal_locomotion_python_module(
  NAME TextLogging
  SOURCES
  src/Module.cpp
  src/TextLogging.cpp
  HEADERS
  ${H_PREFIX}/Module.h
  ${H_PREFIX}/TextLogging.h
  LINK_LIBRARIES
  BipedalLocomotion::TextLogging
  )

if(FRAMEWORK_COMPILE_RosImplementation)

  add_bipedal_locomotion_python_module(
    NAME  TextLoggingRosImplementation
    SOURCES src/RosLogger.cpp src/RosModule.cpp
    HEADERS ${H_PREFIX}/RosLogger.h ${H_PREFIX}/RosModule.h
    LINK_LIBRARIES BipedalLocomotion::TextLoggingRosImplementation)

endif()
