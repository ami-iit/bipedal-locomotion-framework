# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if (FRAMEWORK_COMPILE_Planners)

  set(H_PREFIX include/BipedalLocomotion/Planners)

  add_bipedal_locomotion_library(
    NAME                   Contact
    PUBLIC_HEADERS         ${H_PREFIX}/Contact.h ${H_PREFIX}/ContactList.h ${H_PREFIX}/ContactPhase.h ${H_PREFIX}/ContactPhaseList.h ${H_PREFIX}/ConvexHullHelper.h ${H_PREFIX}/DCMPlanner.h
    SOURCES                src/ContactList.cpp src/ContactPhase.cpp src/ContactPhaseList.cpp src/ConvexHullHelper.cpp src/DCMPlanner.cpp
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core BipedalLocomotion::System BipedalLocomotion::ParametersHandler
    PRIVATE_LINK_LIBRARIES Qhull::qhullcpp Qhull::qhullstatic_r
    INSTALLATION_FOLDER    Planners
    SUBDIRECTORIES         tests
    DEPENDS_ON_EIGEN_PRIVATE)

endif()
