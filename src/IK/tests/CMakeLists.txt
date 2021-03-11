# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
  NAME SE3TaskIK
  SOURCES SE3TaskTest.cpp
  LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME SO3TaskIK
  SOURCES SO3TaskTest.cpp
  LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions)

add_bipedal_test(
  NAME JointsTrackingTaskIK
  SOURCES JointTrackingTaskTest.cpp
  LINKS BipedalLocomotion::IK)

add_bipedal_test(
  NAME CoMTaskIK
  SOURCES CoMTaskTest.cpp
  LINKS BipedalLocomotion::IK)

add_bipedal_test(
  NAME QPInverseKinematics
  SOURCES QPInverseKinematicsTest.cpp
  LINKS BipedalLocomotion::IK BipedalLocomotion::ManifConversions)
