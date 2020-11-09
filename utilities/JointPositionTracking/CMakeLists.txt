# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

add_bipedal_locomotion_application(
  NAME JointPositionTracking
  SOURCES src/Main.cpp src/Module.cpp
  HEADERS include/BipedalLocomotion/JointPositionTracking/Module.h
  LINK_LIBRARIES   YARP::YARP_dev BipedalLocomotion::Planners BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::RobotInterfaceYarpImplementation
  )
