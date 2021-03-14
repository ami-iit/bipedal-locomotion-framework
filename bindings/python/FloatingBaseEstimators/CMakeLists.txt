# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_FloatingBaseEstimators)

  set(H_PREFIX include/BipedalLocomotion/bindings/FloatingBaseEstimators)

  add_bipedal_locomotion_python_module(
    NAME FloatingBaseEstimatorsBindings
    SOURCES src/LeggedOdometry.cpp src/FloatingBaseEstimators.cpp src/Module.cpp
    PUBLIC_HEADERS ${H_PREFIX}/LeggedOdometry.h ${H_PREFIX}/FloatingBaseEstimators.h ${H_PREFIX}/Module.h
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::FloatingBaseEstimators
    INSTALLATION_FOLDER FloatingBaseEstimators
    TESTS tests/test_legged_odometry.py
    )

    configure_file(${CMAKE_CURRENT_SOURCE_DIR}/tests/model.urdf ${PROJECT_BINARY_DIR}/model.urdf COPYONLY)

endif()
