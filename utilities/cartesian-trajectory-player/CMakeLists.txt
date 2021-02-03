# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

find_package(WalkingControllers REQUIRED)
find_package(osqp REQUIRED)
find_package(OsqpEigen REQUIRED)

  set(EXE_TARGET_NAME cartesian-trajectory-player)

  set(${EXE_TARGET_NAME}_THRIFT_HDR
    thrifts/CartesianTrajectoryPlayerCommands.thrift
    )

  yarp_add_idl(${EXE_TARGET_NAME}_THRIFT_GEN_FILES ${${EXE_TARGET_NAME}_THRIFT_HDR})

  add_bipedal_locomotion_application(
    NAME cartesian-trajectory-player
    SOURCES src/Main.cpp src/Module.cpp src/Homing.cpp
    HEADERS include/BipedalLocomotion/CartesianTrajectoryPlayer/Module.h
            include/BipedalLocomotion/CartesianTrajectoryPlayer/Homing.h
            ${${EXE_TARGET_NAME}_THRIFT_GEN_FILES}
    LINK_LIBRARIES  YARP::YARP_dev
    BipedalLocomotion::Planners
    BipedalLocomotion::ParametersHandlerYarpImplementation
    BipedalLocomotion::RobotInterfaceYarpImplementation
    matioCpp::matioCpp
    BipedalLocomotion::matioCppConversions
    WalkingControllers::WholeBodyControllers
    )
