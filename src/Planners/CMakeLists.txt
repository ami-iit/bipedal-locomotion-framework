# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if (FRAMEWORK_COMPILE_Planners)

  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME                  Contact
    PUBLIC_HEADERS        ${H_PREFIX}/Contact.h ${H_PREFIX}/ContactList.h ${H_PREFIX}/ContactPhase.h ${H_PREFIX}/ContactPhaseList.h
    SOURCES               src/ContactList.cpp src/ContactPhase.cpp src/ContactPhaseList.cpp
    PUBLIC_LINK_LIBRARIES iDynTree::idyntree-core
    INSTALLATION_FOLDER   Planners
    )

  add_bipedal_locomotion_library(
    NAME                   Planners
    PUBLIC_HEADERS         ${H_PREFIX}/ConvexHullHelper.h
    SOURCES                src/ConvexHullHelper.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
    PRIVATE_LINK_LIBRARIES Qhull::qhullcpp Qhull::qhullstatic_r
    INSTALLATION_FOLDER    Planners)

  add_subdirectory(tests)

endif()
