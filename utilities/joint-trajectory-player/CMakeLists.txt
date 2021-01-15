# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Ines Sorrentino

add_bipedal_locomotion_application(
  NAME joint-trajectory-player
  SOURCES src/Main.cpp src/Module.cpp
  HEADERS include/BipedalLocomotion/JointTrajectoryPlayer/Module.h
  LINK_LIBRARIES   YARP::YARP_dev BipedalLocomotion::Planners BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::RobotInterfaceYarpImplementation
  )

install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)
