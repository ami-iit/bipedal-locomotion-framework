# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

function(add_bipedal_locomotion_python_module)

  set(options )
  set(oneValueArgs NAME INSTALLATION_FOLDER)
  set(multiValueArgs
    SOURCES
    PUBLIC_HEADERS
    PRIVATE_HEADERS
    PUBLIC_LINK_LIBRARIES
    PRIVATE_LINK_LIBRARIES
    TESTS)

  set(prefix "bipedal_component")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(installation_folder ${${prefix}_INSTALLATION_FOLDER})
  set(is_interface ${${prefix}_IS_INTERFACE})
  set(sources ${${prefix}_SOURCES})
  set(public_headers ${${prefix}_PUBLIC_HEADERS})
  set(private_headers ${${prefix}_PRIVATE_HEADERS})
  set(public_link_libraries ${${prefix}_PUBLIC_LINK_LIBRARIES})
  set(private_link_libraries ${${prefix}_PRIVATE_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})
  set(tests ${${prefix}_TESTS})

  if(NOT installation_folder)
    set(installation_folder ${name})
  endif()


  # add an executable to the project using the specified source files.
  pybind11_add_module(${name} SHARED ${sources} ${public_headers} ${private_headers})

  # Add C++17 features
  target_compile_features(${name} PUBLIC cxx_std_17)
  target_compile_definitions(${name} PRIVATE -D_USE_MATH_DEFINES)

  target_link_libraries(${name} PUBLIC ${public_link_libraries})
  target_link_libraries(${name} PRIVATE ${private_link_libraries})

  set_target_properties(${name} PROPERTIES
    OUTPUT_NAME "${PROJECT_NAME}${name}"
    VERSION ${BipedalLocomotionFramework_VERSION}
    PUBLIC_HEADER "${public_headers}"
    PRIVATE_HEADER "${private_headers}")

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html
  #creating-relocatable-packages
  target_include_directories(${name} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${name}
    EXPORT           ${PROJECT_NAME}
    COMPONENT        runtime
    LIBRARY          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                                    COMPONENT shlib
    ARCHIVE          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                                    COMPONENT lib
    RUNTIME          DESTINATION "${CMAKE_INSTALL_BINDIR}"                                                    COMPONENT bin
    PUBLIC_HEADER    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotion/bindings/${installation_folder}"       COMPONENT dev
    PRIVATE_HEADER   DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotion/bindings/${installation_folder}/impl"  COMPONENT dev)

  # add alias
  add_library(BipedalLocomotion::${name} ALIAS ${name})

  # Add all subdirectories
  foreach(subdir ${subdirectories})
    add_subdirectory(${subdir})
  endforeach()

  set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFrameworkBindings_TARGETS ${name})

  foreach(test ${tests})
    set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFrameworkBindings_TESTS
      ${CMAKE_CURRENT_SOURCE_DIR}/${test})
  endforeach()

  message(STATUS "Created target bindings ${name} for export ${PROJECT_NAME}.")

endfunction()
