# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(FRAMEWORK_COMPILE_MasImuTest)

  set(EXE_TARGET_NAME mas-imu-test)

  # set cpp files
  set(${EXE_TARGET_NAME}_SRC
       src/main.cpp
       src/MasImuTest.cpp
     )

  # set hpp files
  set(${EXE_TARGET_NAME}_HDR
        include/BipedalLocomotion/MasImuTest.h
      )

  set(${EXE_TARGET_NAME}_THRIFT_HDR
    thrifts/MasImuTestCommands.thrift
    )

  yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_GEN_FILES ${${EXE_TARGET_NAME}_THRIFT_HDR})

  add_bipedal_locomotion_application(
    NAME ${EXE_TARGET_NAME}
    SOURCES ${${EXE_TARGET_NAME}_SRC}
    HEADERS ${${EXE_TARGET_NAME}_HDR} ${${EXE_TARGET_NAME}_THRIFT_GEN_FILES}
    LINK_LIBRARIES ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} BipedalLocomotion::GenericContainer BipedalLocomotion::YarpUtilities BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::matioCppConversions
    )

  install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/app)

endif()
