# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

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
    NAME R3TaskTSID
    SOURCES R3TaskTest.cpp
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
  NAME VariableRegularizationTaskTSID
  SOURCES VariableRegularizationTaskTest.cpp
  LINKS BipedalLocomotion::TSID)

add_bipedal_test(
  NAME AngularMomentumTaskTSID
  SOURCES AngularMomentumTaskTest.cpp
  LINKS BipedalLocomotion::TSID)


add_bipedal_test(
  NAME QPFixedBaseTSID
  SOURCES QPFixedBaseTSIDTest.cpp
  LINKS BipedalLocomotion::TSID BipedalLocomotion::ManifConversions BipedalLocomotion::ContinuousDynamicalSystem)
