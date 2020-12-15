# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

if(FRAMEWORK_COMPILE_RealSenseTestApplication)

  add_bipedal_locomotion_application(
    NAME realsense-test
    SOURCES src/Main.cpp src/Module.cpp
    HEADERS include/BipedalLocomotion/RealSenseTest//Module.h
    LINK_LIBRARIES   YARP::YARP_os BipedalLocomotion::PerceptionCapture BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::YarpUtilities BipedalLocomotion::RobotInterface
    )

  install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)
endif()

