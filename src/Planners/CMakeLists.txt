# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if (FRAMEWORK_COMPILE_Planners)

  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME                   Planners
    PUBLIC_HEADERS         ${H_PREFIX}/ConvexHullHelper.h ${H_PREFIX}/DCMPlanner.h ${H_PREFIX}/TimeVaryingDCMPlanner.h ${H_PREFIX}/Spline.h ${H_PREFIX}/QuinticSpline.h ${H_PREFIX}/SO3Planner.h ${H_PREFIX}/SO3Planner.tpp ${H_PREFIX}/SwingFootPlanner.h ${H_PREFIX}/CubicSpline.h
    SOURCES                src/ConvexHullHelper.cpp  src/DCMPlanner.cpp src/TimeVaryingDCMPlanner.cpp src/QuinticSpline.cpp src/SwingFootPlanner.cpp src/CubicSpline.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts
    PRIVATE_LINK_LIBRARIES Qhull::qhullcpp Qhull::qhullstatic_r casadi BipedalLocomotion::Math BipedalLocomotion::TextLogging
    INSTALLATION_FOLDER    Planners)

  add_subdirectory(tests)

endif()
