# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_subdirectory(ParametersHandler)
add_subdirectory(manif)
add_subdirectory(Contacts)
add_subdirectory(Planners)
add_subdirectory(RobotInterface)
add_subdirectory(Math)
add_subdirectory(FloatingBaseEstimators)

include(ConfigureFileWithCMakeIf)

set(FRAMEWORK_COMPILE_RobotInterfaceBindings FRAMEWORK_COMPILE_RobotInterface AND FRAMEWORK_COMPILE_YarpImplementation)

configure_file_with_cmakeif(${CMAKE_CURRENT_SOURCE_DIR}/bipedal_locomotion_framework.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/bipedal_locomotion_framework.cpp
  @ONLY)


pybind11_add_module(pybind11_blf MODULE
  ${CMAKE_CURRENT_BINARY_DIR}/bipedal_locomotion_framework.cpp
  )

get_property(bindings_link_libraries GLOBAL PROPERTY BipedalLocomotionFrameworkBindings_TARGETS)


target_link_libraries(pybind11_blf PRIVATE
  ${bindings_link_libraries})

# # The generated Python dynamic module must have the same name as the pybind11
# # module, i.e. `bindings`.
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
