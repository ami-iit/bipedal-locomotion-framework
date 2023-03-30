# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME GenericContainer
  SOURCES GenericContainerTest.cpp
  LINKS BipedalLocomotion::GenericContainer iDynTree::idyntree-core)

add_bipedal_test(
  NAME NamedTuple
  SOURCES NamedTupleTest.cpp
  LINKS BipedalLocomotion::GenericContainer)

if (FRAMEWORK_HAS_YARP)
    add_bipedal_test(
      NAME GenericContainerPlusYarp
      SOURCES GenericContainerTestPlusYarp.cpp
      LINKS BipedalLocomotion::GenericContainer iDynTree::idyntree-core YARP::YARP_os YARP::YARP_sig)
endif()
