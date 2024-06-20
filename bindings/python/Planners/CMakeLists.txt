# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::Planners)

  set(H_PREFIX include/BipedalLocomotion/bindings/Planners)

  add_bipedal_locomotion_python_module(
    NAME PlannersBindings
    SOURCES
    src/DCMPlanner.cpp
    src/TimeVaryingDCMPlanner.cpp
    src/SwingFootPlanner.cpp
    src/Module.cpp
    src/Spline.cpp
    HEADERS
    ${H_PREFIX}/DCMPlanner.h
    ${H_PREFIX}/TimeVaryingDCMPlanner.h
    ${H_PREFIX}/SwingFootPlanner.h
    ${H_PREFIX}/Module.h
    ${H_PREFIX}/Spline.h
    LINK_LIBRARIES
    BipedalLocomotion::System
    BipedalLocomotion::Planners
    BipedalLocomotion::Contacts
    TESTS
    tests/test_swing_foot_planner.py
    tests/test_time_varying_dcm_planner.py
    )

endif()
