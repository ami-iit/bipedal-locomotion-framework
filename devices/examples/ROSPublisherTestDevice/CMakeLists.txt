# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_YarpUtilities)
# Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name
add_bipedal_yarp_device(
  NAME ROSPublisherTestDevice
  TYPE BipedalLocomotion::ROSPublisherTestDevice
  SOURCES src/ROSPublisherTestDevice.cpp
  PUBLIC_HEADERS include/BipedalLocomotion/ROSPublisherTestDevice.h
  PUBLIC_LINK_LIBRARIES ${YARP_LIBRARIES} BipedalLocomotion::YarpUtilities  BipedalLocomotion::ParametersHandlerYarpImplementation
  CONFIGURE_PACKAGE_NAME ros_publisher_test_device)
endif()

