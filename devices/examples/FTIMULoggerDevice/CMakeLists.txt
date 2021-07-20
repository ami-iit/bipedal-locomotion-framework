# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_YarpUtilities)
# Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name
add_bipedal_yarp_device(
  NAME FTIMULoggerDevice
  TYPE BipedalLocomotion::FTIMULoggerDevice
  SOURCES src/FTIMULoggerDevice.cpp
  PUBLIC_HEADERS include/BipedalLocomotion/FTIMULoggerDevice.h
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES} BipedalLocomotion::YarpUtilities  BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::RobotInterfaceYarpImplementation BipedalLocomotion::matioCppConversions Eigen3::Eigen
  CONFIGURE_PACKAGE_NAME ft_imu_logger_test_device)
endif()

