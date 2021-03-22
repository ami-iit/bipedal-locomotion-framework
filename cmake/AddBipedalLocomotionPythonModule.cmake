# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

function(add_bipedal_locomotion_python_module)

  set(options )
  set(oneValueArgs NAME)
  set(multiValueArgs
    SOURCES
    HEADERS
    LINK_LIBRARIES
    TESTS)

  set(prefix "bipedal_component")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(is_interface ${${prefix}_IS_INTERFACE})
  set(sources ${${prefix}_SOURCES})
  set(headers ${${prefix}_HEADERS})
  set(link_libraries ${${prefix}_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})
  set(tests ${${prefix}_TESTS})

  foreach(file ${headers})
    set_property(GLOBAL APPEND PROPERTY pybind_headers ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()

  foreach(file ${sources})
    set_property(GLOBAL APPEND PROPERTY pybind_sources ${CMAKE_CURRENT_SOURCE_DIR}/${file})
  endforeach()

  set_property(GLOBAL APPEND PROPERTY pybind_include_dirs ${CMAKE_CURRENT_SOURCE_DIR}/include)

  set_property(GLOBAL APPEND PROPERTY pybind_link_libraries ${link_libraries})

  foreach(test ${tests})
    set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFrameworkBindings_TESTS
      ${CMAKE_CURRENT_SOURCE_DIR}/${test})
  endforeach()

  message(STATUS "Added files for bindings named ${name} in ${PROJECT_NAME}.")

endfunction()
