# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
# Authors: Ines Sorrentino

if(FRAMEWORK_COMPILE_JointTrajectoryPlayer)

  add_bipedal_locomotion_application(
    NAME joint-trajectory-player
    SOURCES src/Main.cpp src/Module.cpp
    HEADERS include/BipedalLocomotion/JointTrajectoryPlayer/Module.h
    LINK_LIBRARIES  YARP::YARP_dev
    BipedalLocomotion::Planners
    BipedalLocomotion::ParametersHandlerYarpImplementation
    BipedalLocomotion::RobotInterfaceYarpImplementation
    matioCpp::matioCpp
    BipedalLocomotion::matioCppConversions
    )

  install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

endif()
