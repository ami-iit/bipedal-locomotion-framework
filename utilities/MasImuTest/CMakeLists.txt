# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_MasImuTest)

  set(EXE_TARGET_NAME MasImuTest)

  # set cpp files
  set(${EXE_TARGET_NAME}_SRC
       src/main.cpp
       src/MasImuTest.cpp
     )

  # set hpp files
  set(${EXE_TARGET_NAME}_HDR
        include/BipedalLocomotionControllers/MasImuTest.h
      )

  set(${EXE_TARGET_NAME}_THRIFT_HDR
    thrifts/MasImuTestCommands.thrift
    )

  yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_GEN_FILES ${${EXE_TARGET_NAME}_THRIFT_HDR})

  add_executable(${EXE_TARGET_NAME} ${${EXE_TARGET_NAME}_SRC} ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_THRIFT_GEN_FILES})

  target_link_libraries(${EXE_TARGET_NAME} PUBLIC ${YARP_LIBRARIES} ${iDynTree_LIBRARIES}
      BipedalLocomotionControllers::YarpUtilities BipedalLocomotionControllers::ParametersHandlerYarpImplementation)
  target_compile_features(${EXE_TARGET_NAME} PUBLIC cxx_std_17)

  # Specify include directories for both compilation and installation process.
  # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
  # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
  target_include_directories(${EXE_TARGET_NAME} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

  # Specify installation targets, typology and destination folders.
  install(TARGETS    ${EXE_TARGET_NAME}
    EXPORT     ${PROJECT_NAME}
    COMPONENT  runtime
    LIBRARY       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                               COMPONENT shlib
    ARCHIVE       DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                               COMPONENT lib
    RUNTIME       DESTINATION "${CMAKE_INSTALL_BINDIR}"                                               COMPONENT bin)

  message(STATUS "Created target ${EXE_TARGET_NAME} for export ${PROJECT_NAME}.")

endif()
