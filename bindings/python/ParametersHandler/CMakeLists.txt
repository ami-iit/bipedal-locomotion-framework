# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_locomotion_python_module(
  NAME ParametersHandlerBindings
  SOURCES src/ParametersHandler.cpp src/Module.cpp
  PUBLIC_HEADERS include/BipedalLocomotion/bindings/ParametersHandler/ParametersHandler.h include/BipedalLocomotion/bindings/ParametersHandler/Module.h
  PRIVATE_LINK_LIBRARIES BipedalLocomotion::ParametersHandler
  INSTALLATION_FOLDER ParametersHandler
  TESTS tests/test_parameters_handler_std.py
  )
