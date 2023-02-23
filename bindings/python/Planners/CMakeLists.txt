# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_Planners AND FRAMEWORK_COMPILE_Unicycle)

  set(H_PREFIX include/BipedalLocomotion/bindings/Planners)

  add_bipedal_locomotion_python_module(
    NAME PlannersBindings
    SOURCES
    src/DCMPlanner.cpp
    src/TimeVaryingDCMPlanner.cpp
    src/Spline.cpp
    src/SwingFootPlanner.cpp
    src/UnicyclePlanner.cpp
    src/Module.cpp
    HEADERS
    ${H_PREFIX}/DCMPlanner.h
    ${H_PREFIX}/TimeVaryingDCMPlanner.h
    ${H_PREFIX}/Spline.h
    ${H_PREFIX}/SwingFootPlanner.h
    ${H_PREFIX}/UnicyclePlanner.h
    ${H_PREFIX}/Module.h
    LINK_LIBRARIES
    BipedalLocomotion::System
    BipedalLocomotion::Planners
    BipedalLocomotion::Unicycle
    BipedalLocomotion::Contacts
    TESTS
    tests/test_quintic_spline.py
    tests/test_swing_foot_planner.py
    tests/test_time_verying_dcm_planner.py
    tests/test_unicycle_planner.py
    )

endif()
