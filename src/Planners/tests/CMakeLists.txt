# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
    NAME ConvexHullHelper
    SOURCES ConvexHullHelperTest.cpp
    LINKS BipedalLocomotion::Planners)

add_bipedal_test(
  NAME TimeVaryingDCMPlanner
  SOURCES TimeVaryingDCMPlannerTest.cpp
  LINKS BipedalLocomotion::Planners BipedalLocomotion::Math)

add_bipedal_test(
  NAME SO3Planner
  SOURCES SO3PlannerTest.cpp
  LINKS BipedalLocomotion::Planners)

add_bipedal_test(
  NAME SwingFootPlanner
  SOURCES SwingFootPlannerTest.cpp
  LINKS BipedalLocomotion::Planners)
