# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_JointTorqueControlDevice)
  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name

  add_bipedal_yarp_device(
      NAME JointTorqueControlDevice
      TYPE BipedalLocomotion::JointTorqueControlDevice
      SOURCES src/JointTorqueControlDevice.cpp src/PassThroughControlBoard.cpp
      PUBLIC_HEADERS include/BipedalLocomotion/JointTorqueControlDevice.h include/BipedalLocomotion/PassThroughControlBoard.h
      PRIVATE_LINK_LIBRARIES
      YARP::YARP_os
      YARP::YARP_dev
      YARP::YARP_sig
      BipedalLocomotion::YarpUtilities
      BipedalLocomotion::RobotInterfaceYarpImplementation
      BipedalLocomotion::ParametersHandlerYarpImplementation
      CONFIGURE_PACKAGE_NAME joint_torque_control_device)

endif()
