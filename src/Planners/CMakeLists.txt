# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if (FRAMEWORK_COMPILE_Contact)
  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME                  Contact
    PUBLIC_HEADERS        ${H_PREFIX}/Contact.h ${H_PREFIX}/ContactList.h ${H_PREFIX}/ContactPhase.h ${H_PREFIX}/ContactPhaseList.h
    SOURCES               src/ContactList.cpp src/ContactPhase.cpp src/ContactPhaseList.cpp
    PUBLIC_LINK_LIBRARIES MANIF::manif BipedalLocomotion::Contacts
    INSTALLATION_FOLDER   Planners)

endif()

if (FRAMEWORK_COMPILE_Planners)

  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME                   Planners
    PUBLIC_HEADERS         ${H_PREFIX}/ConvexHullHelper.h ${H_PREFIX}/DCMPlanner.h ${H_PREFIX}/TimeVaryingDCMPlanner.h ${H_PREFIX}/QuinticSpline.h ${H_PREFIX}/SO3Planner.h ${H_PREFIX}/SO3Planner.tpp ${H_PREFIX}/SwingFootPlanner.h
    SOURCES                src/ConvexHullHelper.cpp  src/DCMPlanner.cpp src/TimeVaryingDCMPlanner.cpp src/QuinticSpline.cpp src/SwingFootPlanner.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contact
    PRIVATE_LINK_LIBRARIES Qhull::qhullcpp Qhull::qhullstatic_r casadi
    INSTALLATION_FOLDER    Planners)

  add_subdirectory(tests)

endif()
