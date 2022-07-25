# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_TomlImplementation)

  add_bipedal_locomotion_library(
    NAME                   ParametersHandlerTomlImplementation
    SOURCES                src/TomlImplementation.cpp
    PUBLIC_HEADERS         include/BipedalLocomotion/ParametersHandler/TomlImplementation.h include/BipedalLocomotion/ParametersHandler/TomlImplementation.tpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging tomlplusplus::tomlplusplus
    INSTALLATION_FOLDER ParametersHandler)

endif()
