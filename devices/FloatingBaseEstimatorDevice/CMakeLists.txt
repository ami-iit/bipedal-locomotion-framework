# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation)
  cmake_minimum_required(VERSION 3.5)
  set(PLUGIN_NAME FloatingBaseEstimatorDevice)

  set(${PLUGIN_NAME}_SRC src/FloatingBaseEstimatorDevice.cpp)
  set(${PLUGIN_NAME}_HDR include/BipedalLocomotion/FloatingBaseEstimatorDevice.h)

  yarp_prepare_plugin(${PLUGIN_NAME} CATEGORY device
                                     TYPE BipedalLocomotion::FloatingBaseEstimatorDevice
                                     INCLUDE ${${PLUGIN_NAME}_HDR}
                                     DEFAULT ON)

  yarp_add_plugin(${PLUGIN_NAME} ${${PLUGIN_NAME}_SRC} ${${PLUGIN_NAME}_HDR})

  target_link_libraries(${PLUGIN_NAME} PUBLIC ${YARP_LIBRARIES}
                                              ${iDynTree_LIBRARIES}
                                              BipedalLocomotion::YarpUtilities
                                              BipedalLocomotion::RobotInterfaceYarpImplementation
                                              BipedalLocomotion::ParametersHandlerYarpImplementation
                                              BipedalLocomotion::FloatingBaseEstimators)
  target_compile_features(${PLUGIN_NAME} PUBLIC cxx_std_17)

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(${PLUGIN_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "<INSTALLINTERFACE:<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  yarp_install(TARGETS ${PLUGIN_NAME}
               COMPONENT runtime
               LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}/
               ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}/)
  yarp_install(FILES FloatingBaseEstimatorDevice.ini
               COMPONENT runtime
               DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR}/)

  add_subdirectory(app)

  message(STATUS "Created device ${PLUGIN_NAME}.")

endif()

