# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::FloatingBaseEstimators)

  set(H_PREFIX include/BipedalLocomotion/bindings/FloatingBaseEstimators)

  add_bipedal_locomotion_python_module(
    NAME FloatingBaseEstimatorsBindings
    SOURCES src/LeggedOdometry.cpp src/FloatingBaseEstimators.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/LeggedOdometry.h ${H_PREFIX}/FloatingBaseEstimators.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::FloatingBaseEstimators
    TESTS tests/test_legged_odometry.py
    TESTS_RUNTIME_CONDITIONS FRAMEWORK_USE_icub-models)

endif()
