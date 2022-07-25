# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_RealSenseTestApplication)

  add_bipedal_locomotion_application(
    NAME realsense-test
    SOURCES src/Main.cpp src/Module.cpp
    HEADERS include/BipedalLocomotion/RealSenseTest//Module.h
    LINK_LIBRARIES   YARP::YARP_os BipedalLocomotion::PerceptionCapture BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::YarpUtilities)

  install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)
endif()

