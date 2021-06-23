# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
    NAME ConvexHullHelper
    SOURCES ConvexHullHelperTest.cpp
    LINKS BipedalLocomotion::Planners)

add_bipedal_test(
  NAME TimeVaryingDCMPlanner
  SOURCES TimeVaryingDCMPlannerTest.cpp
  LINKS BipedalLocomotion::Planners BipedalLocomotion::Math)

add_bipedal_test(
  NAME Spline
  SOURCES SplineTest.cpp
  LINKS BipedalLocomotion::Planners)

add_bipedal_test(
  NAME SO3Planner
  SOURCES SO3PlannerTest.cpp
  LINKS BipedalLocomotion::Planners)

add_bipedal_test(
  NAME SwingFootPlanner
  SOURCES SwingFootPlannerTest.cpp
  LINKS BipedalLocomotion::Planners)
