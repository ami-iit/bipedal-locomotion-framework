# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_OptimalControlUtilities)

  set(LIBRARY_TARGET_NAME OptimalControlUtilities)

  # set cpp files
  set(${LIBRARY_TARGET_NAME}_SRC
    src/PID.cpp
    src/VariableHandler.cpp
    src/ControlProblemElements.cpp
    src/OptimizationProblemElements.cpp
    )

  # set hpp files
  set(${LIBRARY_TARGET_NAME}_HDR
    include/BipedalLocomotionControllers/OptimalControlUtilities/PID.h
    include/BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h
    include/BipedalLocomotionControllers/OptimalControlUtilities/ControlProblemElements.h
    include/BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h
    )

  # add an executable to the project using the specified source files.
  add_library(${LIBRARY_TARGET_NAME} SHARED ${${LIBRARY_TARGET_NAME}_SRC} ${${LIBRARY_TARGET_NAME}_HDR})

  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES OUTPUT_NAME "${PROJECT_NAME}${LIBRARY_TARGET_NAME}")

  target_link_libraries(${LIBRARY_TARGET_NAME} PUBLIC ${iDynTree_LIBRARIES})

  add_library(BipedalLocomotionControllers::${LIBRARY_TARGET_NAME} ALIAS ${LIBRARY_TARGET_NAME})

  set_target_properties(${LIBRARY_TARGET_NAME} PROPERTIES VERSION ${BipedalLocomotionControllers_VERSION}
    PUBLIC_HEADER "${${LIBRARY_TARGET_NAME}_HDR}")

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(${LIBRARY_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${LIBRARY_TARGET_NAME}
    EXPORT     ${PROJECT_NAME}
    COMPONENT  runtime
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                               COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                               COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                               COMPONENT bin
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotionControllers/${LIBRARY_TARGET_NAME}" COMPONENT dev)

  set_property(GLOBAL APPEND PROPERTY BipedalLocomotionControllers_TARGETS ${LIBRARY_TARGET_NAME})

  message(STATUS "Created target ${LIBRARY_TARGET_NAME} for export ${PROJECT_NAME}.")

endif()
