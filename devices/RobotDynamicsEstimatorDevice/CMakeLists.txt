# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_RobotDynamicsEstimator AND FRAMEWORK_COMPILE_ManifConversions)
    # Warning: the CONFIGURE_PACKAGE_NAME option should be different from the plugin NAME
    add_bipedal_yarp_device(
        NAME RobotDynamicsEstimatorDevice
        TYPE BipedalLocomotion::RobotDynamicsEstimatorDevice
        SOURCES src/RobotDynamicsEstimatorDevice.cpp
        PUBLIC_HEADERS include/BipedalLocomotion/RobotDynamicsEstimatorDevice.h
        PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} BipedalLocomotion::YarpUtilities
        BipedalLocomotion::RobotInterfaceYarpImplementation
        BipedalLocomotion::ParametersHandlerYarpImplementation
        BipedalLocomotion::RobotDynamicsEstimator
        BipedalLocomotion::ManifConversions
        BipedalLocomotion::VectorsCollection
        CONFIGURE_PACKAGE_NAME robot_dynamics_estimator_device)
endif()
