# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_System)

  add_bipedal_locomotion_python_module(
    NAME System
    SOURCES src/VariablesHandler.cpp src/LinearTask.cpp src/Module.cpp
    HEADERS include/BipedalLocomotion/bindings/System/VariablesHandler.h include/BipedalLocomotion/bindings/System/LinearTask.h
    LINK_LIBRARIES BipedalLocomotion::System
    TESTS tests/test_variables_handler.py
    )

endif()
