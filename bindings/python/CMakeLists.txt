# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

find_package(Python3 COMPONENTS Interpreter Development NumPy REQUIRED)

pybind11_add_module(pybind11_BipedalLocomotion SYSTEM
  BipedalLocomotion.cpp
  BipedalLocomotion_GenericContainer.h BipedalLocomotion_GenericContainer.cpp
  BipedalLocomotion_ParametersHandler.h BipedalLocomotion_ParametersHandler.cpp
  BipedalLocomotion_Planners.h BipedalLocomotion_Planners.cpp)

target_link_libraries(pybind11_BipedalLocomotion PUBLIC
  BipedalLocomotion::GenericContainer
  BipedalLocomotion::ParametersHandler
  BipedalLocomotion::Planners)

# The generated Python dynamic module must have the same name as the pybind11
# module, i.e. `bindings`.
set_target_properties(pybind11_BipedalLocomotion PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}/BipedalLocomotion
    OUTPUT_NAME "pybind")

if(FRAMEWORK_COMPILE_tests)
    add_subdirectory(tests)
endif()

# Output package is:
# BipedalLocomotion
# |
# |- __init__.py (generated from main bindings CMake file).
# |
# |_ pybind.<cpython_extension>
install(TARGETS pybind11_BipedalLocomotion DESTINATION ${PYTHON_INSTDIR})
