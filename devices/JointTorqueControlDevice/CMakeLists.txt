# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_JointTorqueControlDevice)
  add_bipedal_locomotion_yarp_thrift(
      NAME JointTorqueControlCommands
      THRIFT thrifts/JointTorqueControlCommands.thrift
      INSTALLATION_FOLDER JointTorqueControlDevice)

#   add_bipedal_yarp_device(
#       NAME JointTorqueControlDevice
#       TYPE BipedalLocomotion::JointTorqueControlDevice
#       SOURCES src/JointTorqueControlDevice.cpp
#               src/PassThroughControlBoard.cpp
#               src/PINNFrictionEstimator.cpp
#       PUBLIC_HEADERS include/BipedalLocomotion/JointTorqueControlDevice.h
#                      include/BipedalLocomotion/PassThroughControlBoard.h
#                      include/BipedalLocomotion/PINNFrictionEstimator.h
#       PRIVATE_LINK_LIBRARIES
#       BipedalLocomotion::YarpUtilities
#       BipedalLocomotion::SystemYarpImplementation
#       BipedalLocomotion::RobotInterfaceYarpImplementation
#       BipedalLocomotion::ParametersHandlerYarpImplementation
#       BipedalLocomotion::VectorsCollection
#       BipedalLocomotion::JointTorqueControlCommands
#       BipedalLocomotion::Math
#       BipedalLocomotion::ContinuousDynamicalSystem
#       PUBLIC_LINK_LIBRARIES
#       onnxruntime::onnxruntime
#       Eigen3::Eigen
#       ${YARP_LIBRARIES}
#       ${iDynTree_LIBRARIES}
#       CONFIGURE_PACKAGE_NAME joint_torque_control_device)

  add_bipedal_yarp_device(
        NAME PassThroughControlBoard
        TYPE BipedalLocomotion::PassThroughControlBoard
        SOURCES src/PassThroughControlBoard.cpp
        PUBLIC_HEADERS include/BipedalLocomotion/PassThroughControlBoard.h
        PUBLIC_LINK_LIBRARIES
        ${YARP_LIBRARIES}
        CONFIGURE_PACKAGE_NAME pass_through_control_board_device)

#  add_bipedal_yarp_device(
#        NAME PassThroughControlBoardDevice
#        TYPE BipedalLocomotion::PassThroughControlBoardDevice
#        SOURCES src/PassThroughControlBoardDevice.cpp
#                src/PassThroughControlBoard.cpp
#        PUBLIC_HEADERS include/BipedalLocomotion/PassThroughControlBoardDevice.h
#                       include/BipedalLocomotion/PassThroughControlBoard.h
#        PRIVATE_LINK_LIBRARIES
#        BipedalLocomotion::SystemYarpImplementation
#        BipedalLocomotion::JointTorqueControlCommands
#        PUBLIC_LINK_LIBRARIES
#        onnxruntime::onnxruntime
#        ${YARP_LIBRARIES}
#        CONFIGURE_PACKAGE_NAME pass_through_control_board_device) 
 
endif()
