# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_VectorsCollectionWrapper)
  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name

  add_bipedal_yarp_device(
    NAME VectorsCollectionWrapper
    TYPE BipedalLocomotion::VectorsCollectionWrapper
    SOURCES src/VectorsCollectionWrapper.cpp
    PUBLIC_HEADERS include/BipedalLocomotion/VectorsCollectionWrapper.h
    PUBLIC_LINK_LIBRARIES
      YARP::YARP_os
      YARP::YARP_dev
      BipedalLocomotion::TextLogging
      BipedalLocomotion::YarpUtilities
      BipedalLocomotion::VectorsCollection
      BipedalLocomotion::ParametersHandlerYarpImplementation
      BipedalLocomotion::TextLoggingYarpImplementation
      BipedalLocomotion::SystemYarpImplementation
    CONFIGURE_PACKAGE_NAME yarp_vectors_collection_wrapper)
endif()

