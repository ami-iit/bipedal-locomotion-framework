# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::RobotDynamicsEstimator)

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotDynamicsEstimator)

  add_bipedal_locomotion_python_module(
       NAME RobotDynamicsEstimatorBindings
       SOURCES src/RobotDynamicsEstimator.cpp src/Module.cpp
       HEADERS ${H_PREFIX}/RobotDynamicsEstimator.h ${H_PREFIX}/Module.h
       LINK_LIBRARIES BipedalLocomotion::RobotDynamicsEstimator
       )

endif()
