# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
  NAME GenericContainer
  SOURCES GenericContainerTest.cpp
  LINKS BipedalLocomotion::GenericContainer iDynTree::idyntree-core)

if (FRAMEWORK_HAS_YARP)
    add_bipedal_test(
      NAME GenericContainerPlusYarp
      SOURCES GenericContainerTestPlusYarp.cpp
      LINKS BipedalLocomotion::GenericContainer iDynTree::idyntree-core YARP::YARP_os YARP::YARP_sig)
endif()
