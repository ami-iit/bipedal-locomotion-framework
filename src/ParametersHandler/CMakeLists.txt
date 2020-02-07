# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
set(LIBRARY_TARGET_NAME ParametersHandler)

# set cpp files
set(${LIBRARY_TARGET_NAME}_SRC
  )

# set hpp files
set(${LIBRARY_TARGET_NAME}_HDR
  include/BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h
  include/BipedalLocomotionControllers/ParametersHandler/IParametersHandler.tpp
  )

# add an executable to the project using the specified source files.
add_library(${LIBRARY_TARGET_NAME} INTERFACE)

add_library(BipedalLocomotionControllers::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

# Specify include directories for both compilation and installation process.
# The $<INSTALL_PREFIX> generator expression is useful to ensure to create
# relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
target_include_directories(${LIBRARY_TARGET_NAME} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
  "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

# Specify installation targets, typology and destination folders.
install(TARGETS    ${LIBRARY_TARGET_NAME}
  EXPORT     ${PROJECT_NAME}
  COMPONENT  runtime)

install(FILES ${${LIBRARY_TARGET_NAME}_HDR}
  DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotionControllers/ParametersHandler")


set_property(GLOBAL APPEND PROPERTY BipedalLocomotionControllers_TARGETS ${LIBRARY_TARGET_NAME})

message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

add_subdirectory(YarpImplementation)
