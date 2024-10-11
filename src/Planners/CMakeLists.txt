# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if (FRAMEWORK_COMPILE_Planners)

  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME Planners
    PUBLIC_HEADERS ${H_PREFIX}/ConvexHullHelper.h ${H_PREFIX}/DCMPlanner.h ${H_PREFIX}/TimeVaryingDCMPlanner.h
                   ${H_PREFIX}/SO3Planner.h ${H_PREFIX}/SO3Planner.tpp ${H_PREFIX}/SwingFootPlanner.h
                   ${H_PREFIX}/Spline.h ${H_PREFIX}/CubicSpline.h ${H_PREFIX}/QuinticSpline.h
    SOURCES src/ConvexHullHelper.cpp src/DCMPlanner.cpp  src/TimeVaryingDCMPlanner.cpp src/SwingFootPlanner.cpp
    PUBLIC_LINK_LIBRARIES Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts BipedalLocomotion::Math
    PRIVATE_LINK_LIBRARIES Qhull::qhull_r casadi iDynTree::idyntree-core BipedalLocomotion::Math BipedalLocomotion::TextLogging
    INSTALLATION_FOLDER Planners)

  add_subdirectory(tests)

endif()

if (FRAMEWORK_COMPILE_Unicycle)

  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME Unicycle
    PUBLIC_HEADERS ${H_PREFIX}/UnicycleTrajectoryPlanner.h ${H_PREFIX}/UnicycleTrajectoryGenerator.h ${H_PREFIX}/UnicycleUtilities.h
    SOURCES src/UnicycleTrajectoryPlanner.cpp src/UnicycleTrajectoryGenerator.cpp src/UnicycleUtilities.cpp
    PUBLIC_LINK_LIBRARIES BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts Eigen3::Eigen iDynTree::idyntree-modelio
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging UnicyclePlanner BipedalLocomotion::ContinuousDynamicalSystem BipedalLocomotion::ManifConversions
    INSTALLATION_FOLDER Planners)

endif()
