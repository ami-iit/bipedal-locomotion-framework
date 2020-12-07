# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

pybind11_add_module(pybind11_blf MODULE
    # Main module
    bipedal_locomotion_framework.h
    bipedal_locomotion_framework.cpp
    # Support files
    BaseTypes.cpp
    Contacts.cpp
    DCMPlanner.cpp
    ParametersHandler.cpp
    QuinticSpline.cpp
    SwingFootPlaner.cpp
    TimeVaryingDCMPlanner.cpp
    )

target_link_libraries(pybind11_blf PUBLIC
    BipedalLocomotion::ParametersHandler
    BipedalLocomotion::Planners
    BipedalLocomotion::System
    )

target_include_directories(pybind11_blf PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}>)

# The generated Python dynamic module must have the same name as the pybind11
# module, i.e. `bindings`.
set_target_properties(pybind11_blf PROPERTIES
    LIBRARY_OUTPUT_DIRECTORY "${BLF_PYTHON_PACKAGE}"
    OUTPUT_NAME "bindings")

if(FRAMEWORK_TEST_PYTHON_BINDINGS)
    add_subdirectory(tests)
endif()

# Output package is:
#
# bipedal_locomotion
# |-- __init__.py (generated from main bindings CMake file)
# `-- bindings.<cpython_extension>
#
install(TARGETS pybind11_blf DESTINATION ${PYTHON_INSTDIR})
