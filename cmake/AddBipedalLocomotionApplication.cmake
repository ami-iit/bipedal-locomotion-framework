# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

function(add_bipedal_locomotion_application)

  set(options)
  set(oneValueArgs NAME)
  set(multiValueArgs
    SOURCES
    HEADERS
    LINK_LIBRARIES
    SUBDIRECTORIES)

  set(prefix "bipedal_component")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(sources ${${prefix}_SOURCES})
  set(headers ${${prefix}_HEADERS})
  set(link_libraries ${${prefix}_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})

  # add an executable to the project using the specified source files.
  add_executable(${name} ${sources} ${headers})

  # Add C++17 features
  target_compile_features(${name} PRIVATE cxx_std_17)
  target_compile_definitions(${name} PRIVATE -D_USE_MATH_DEFINES)

  set_target_properties(${name} PROPERTIES
      OUTPUT_NAME "blf-${name}"
      VERSION ${BipedalLocomotionFramework_VERSION}
      PRIVATE_HEADER "${headers}")


  target_link_libraries(${name} PRIVATE ${link_libraries})

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html
  #creating-relocatable-packages
  target_include_directories(${name} PRIVATE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${name}
    RUNTIME          DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT bin)

  # Add all subdirectories
  foreach(subdir ${subdirectories})
    add_subdirectory(${subdir})
  endforeach()

  message(STATUS "Created target application ${name}.")

endfunction()
