# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_TSID)

  set(H_PREFIX include/BipedalLocomotion/TSID)

  add_bipedal_locomotion_library(
    NAME                   TSID
    PUBLIC_HEADERS         ${H_PREFIX}/TSIDLinearTask.h ${H_PREFIX}/SO3Task.h ${H_PREFIX}/SE3Task.h ${H_PREFIX}/R3Task.h ${H_PREFIX}/JointTrackingTask.h ${H_PREFIX}/CoMTask.h ${H_PREFIX}/AngularMomentumTask.h
                           ${H_PREFIX}/BaseDynamicsTask.h ${H_PREFIX}/JointDynamicsTask.h
                           ${H_PREFIX}/TaskSpaceInverseDynamics.h
                           ${H_PREFIX}/FeasibleContactWrenchTask.h
                           ${H_PREFIX}/VariableRegularizationTask.h
                           ${H_PREFIX}/QPFixedBaseTSID.h ${H_PREFIX}/QPTSID.h
    SOURCES                src/TSIDLinearTask.cpp src/SO3Task.cpp src/SE3Task.cpp src/JointTrackingTask.cpp src/CoMTask.cpp src/AngularMomentumTask.cpp src/R3Task.cpp
                           src/BaseDynamicsTask.cpp src/JointDynamicsTask.cpp
                           src/FeasibleContactWrenchTask.cpp
                           src/VariableRegularizationTask.cpp
                           src/QPFixedBaseTSID.cpp src/QPTSID.cpp src/TaskSpaceInverseDynamics.cpp
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
                           BipedalLocomotion::Contacts
                           BipedalLocomotion::ParametersHandler
                           BipedalLocomotion::System
                           BipedalLocomotion::Math
                           LieGroupControllers::LieGroupControllers
                           MANIF::manif
                           iDynTree::idyntree-high-level
                           iDynTree::idyntree-model
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::ManifConversions BipedalLocomotion::TextLogging OsqpEigen::OsqpEigen
    SUBDIRECTORIES         tests)

endif()
