# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME ParametersHandler
  SOURCES ParametersHandlerTest.cpp
  LINKS BipedalLocomotion::ParametersHandler)

if(FRAMEWORK_COMPILE_YarpImplementation)

  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

  add_bipedal_test(
    NAME ParametersHandlerYarp
    SOURCES ParametersHandlerYarpTest.cpp
    LINKS BipedalLocomotion::ParametersHandlerYarpImplementation)

endif()


if(FRAMEWORK_COMPILE_TomlImplementation)

  add_bipedal_test(
    NAME ParametersHandlerToml
    SOURCES ParametersHandlerTomlTest.cpp
    LINKS BipedalLocomotion::ParametersHandlerTomlImplementation)

endif()
