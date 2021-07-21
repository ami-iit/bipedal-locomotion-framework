# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
  NAME CoMTaskTSID
  SOURCES CoMTaskTest.cpp
  LINKS BipedalLocomotion::TSID)

add_bipedal_test(
  NAME SO3TaskTSID
  SOURCES SO3TaskTest.cpp
  LINKS BipedalLocomotion::TSID BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME SE3TaskTSID
  SOURCES SE3TaskTest.cpp
  LINKS BipedalLocomotion::TSID BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME JointsTrackingTaskTSID
  SOURCES JointTrackingTaskTest.cpp
  LINKS BipedalLocomotion::TSID)

add_bipedal_test(
  NAME DynamicsTaskTSID
  SOURCES DynamicsTaskTest.cpp
  LINKS BipedalLocomotion::TSID)

add_bipedal_test(
  NAME FeasibleContactWrenchTaskTSID
  SOURCES FeasibleContactWrenchTaskTest.cpp
  LINKS BipedalLocomotion::TSID)

 add_bipedal_test(
  NAME QPFixedBaseTSID
  SOURCES QPFixedBaseTSIDTest.cpp
  LINKS BipedalLocomotion::TSID BipedalLocomotion::ManifConversions BipedalLocomotion::ContinuousDynamicalSystem)
