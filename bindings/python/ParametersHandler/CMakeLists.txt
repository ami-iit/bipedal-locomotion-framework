# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

set(H_PREFIX include/BipedalLocomotion/bindings/ParametersHandler)

add_bipedal_locomotion_python_module(
  NAME ParametersHandlerBindings
  SOURCES src/ParametersHandler.cpp src/Module.cpp
  HEADERS ${H_PREFIX}/ParametersHandler.h ${H_PREFIX}/Module.h
  LINK_LIBRARIES BipedalLocomotion::ParametersHandler
  TESTS tests/test_parameters_handler_std.py
  )


if(TARGET BipedalLocomotion::ParametersHandlerYarpImplementation)

  add_bipedal_locomotion_python_module(
    NAME  ParametersHandlerYarpImplementationBindings
    SOURCES src/YarpParametersHandler.cpp src/YarpModule.cpp
    HEADERS ${H_PREFIX}/YarpParametersHandler.h ${H_PREFIX}/YarpModule.h
    LINK_LIBRARIES BipedalLocomotion::ParametersHandlerYarpImplementation
    TESTS tests/test_parameters_handler_yarp.py
    )

  configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/config.ini ${PROJECT_BINARY_DIR}/config.ini COPYONLY)


endif()


if(TARGET BipedalLocomotion::ParametersHandlerTomlImplementation)

  add_bipedal_locomotion_python_module(
    NAME  ParametersHandlerTomlImplementationBindings
    SOURCES src/TomlParametersHandler.cpp src/TomlModule.cpp
    HEADERS ${H_PREFIX}/TomlParametersHandler.h ${H_PREFIX}/TomlModule.h
    LINK_LIBRARIES BipedalLocomotion::ParametersHandlerTomlImplementation
    TESTS tests/test_parameters_handler_toml.py
    )

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/config.toml ${PROJECT_BINARY_DIR}/config.toml COPYONLY)

endif()
