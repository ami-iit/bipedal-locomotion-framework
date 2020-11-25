# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_FloatingBaseEstimators)

  add_bipedal_yarp_device(
    NAME FloatingBaseEstimatorDevice
    TYPE BipedalLocomotion::FloatingBaseEstimatorDevice
    SOURCES src/FloatingBaseEstimatorDevice.cpp
    PUBLIC_HEADERS include/BipedalLocomotion/FloatingBaseEstimatorDevice.h
    PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} BipedalLocomotion::YarpUtilities BipedalLocomotion::RobotInterfaceYarpImplementation BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::FloatingBaseEstimators
    INI FloatingBaseEstimatorDevice.ini)
endif()
