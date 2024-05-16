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

if (FRAMEWORK_COMPILE_Unicycle)

  include(FetchContent)
  FetchContent_Declare(
    ResolveRoboticsURICpp
    GIT_REPOSITORY https://github.com/ami-iit/resolve-robotics-uri-cpp
    GIT_TAG        v0.0.2
  )
  FetchContent_MakeAvailable(ResolveRoboticsURICpp)

  add_bipedal_test(
    NAME UnicycleTrajectoryPlanner
    SOURCES UnicycleTrajectoryPlannerTest.cpp
    LINKS BipedalLocomotion::Unicycle ResolveRoboticsURICpp::ResolveRoboticsURICpp
    )

endif()
