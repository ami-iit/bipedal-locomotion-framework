# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::IK)

  set(H_PREFIX include/BipedalLocomotion/bindings/IK)

  add_bipedal_locomotion_python_module(
    NAME IKBindings
    SOURCES src/IntegrationBasedIK.cpp src/QPInverseKinematics.cpp src/CoMTask.cpp src/R3Task.cpp src/SE3Task.cpp src/SO3Task.cpp src/JointTrackingTask.cpp  src/JointLimitsTask.cpp src/AngularMomentumTask.cpp src/Module.cpp src/IKLinearTask.cpp src/DistanceTask.cpp src/GravityTask.cpp
    HEADERS ${H_PREFIX}/IntegrationBasedIK.h ${H_PREFIX}/QPInverseKinematics.h ${H_PREFIX}/CoMTask.h ${H_PREFIX}/R3Task.h ${H_PREFIX}/SE3Task.h ${H_PREFIX}/SO3Task.h ${H_PREFIX}/JointTrackingTask.h ${H_PREFIX}/JointLimitsTask.h ${H_PREFIX}/AngularMomentumTask.h ${H_PREFIX}/Module.h ${H_PREFIX}/IKLinearTask.h ${H_PREFIX}/DistanceTask.h ${H_PREFIX}/GravityTask.h
    LINK_LIBRARIES BipedalLocomotion::IK MANIF::manif
    TESTS tests/test_QP_inverse_kinematics.py
    TESTS_RUNTIME_CONDITIONS FRAMEWORK_USE_icub-models
    )

endif()
