# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_IK)

  set(H_PREFIX include/BipedalLocomotion/IK)

  add_bipedal_locomotion_library(
    NAME                   IK
    PUBLIC_HEADERS         ${H_PREFIX}/LinearTask.h ${H_PREFIX}/SE3Task.h ${H_PREFIX}/SO3Task.h ${H_PREFIX}/JointTrackingTask.h ${H_PREFIX}/CoMTask.h ${H_PREFIX}/IntegrationBasedIK.h ${H_PREFIX}/QPInverseKinematics.h
    SOURCES                src/LinearTask.cpp src/SE3Task.cpp src/SO3Task.cpp src/JointTrackingTask.cpp src/CoMTask.cpp src/IntegrationBasedIK.cpp src/QPInverseKinematics.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
                           BipedalLocomotion::ParametersHandler BipedalLocomotion::System
                           LieGroupControllers::LieGroupControllers
                           MANIF::manif
                           iDynTree::idyntree-high-level iDynTree::idyntree-model
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging OsqpEigen::OsqpEigen BipedalLocomotion::ManifConversions
    SUBDIRECTORIES         tests)

endif()
