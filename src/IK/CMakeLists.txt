# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_IK)

  set(H_PREFIX include/BipedalLocomotion/IK)

  add_bipedal_locomotion_library(
    NAME                   IK
    PUBLIC_HEADERS         ${H_PREFIX}/R3Task.h ${H_PREFIX}/SE3Task.h ${H_PREFIX}/SO3Task.h ${H_PREFIX}/JointTrackingTask.h ${H_PREFIX}/CoMTask.h ${H_PREFIX}/IntegrationBasedIK.h
                           ${H_PREFIX}/AngularMomentumTask.h ${H_PREFIX}/JointLimitsTask.h ${H_PREFIX}/QPFixedBaseInverseKinematics.h
                           ${H_PREFIX}/QPInverseKinematics.h ${H_PREFIX}/IKLinearTask.h ${H_PREFIX}/DistanceTask.h
    SOURCES                src/R3Task.cpp src/SE3Task.cpp src/SO3Task.cpp src/JointTrackingTask.cpp src/CoMTask.cpp src/AngularMomentumTask.cpp src/JointLimitsTask.cpp
                           src/QPInverseKinematics.cpp src/QPFixedBaseInverseKinematics.cpp src/IKLinearTask.cpp src/IntegrationBasedIK.cpp src/DistanceTask.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
                           BipedalLocomotion::ParametersHandler BipedalLocomotion::System
                           LieGroupControllers::LieGroupControllers
                           MANIF::manif
                           iDynTree::idyntree-high-level iDynTree::idyntree-model
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging OsqpEigen::OsqpEigen BipedalLocomotion::ManifConversions
    SUBDIRECTORIES         tests)

endif()
