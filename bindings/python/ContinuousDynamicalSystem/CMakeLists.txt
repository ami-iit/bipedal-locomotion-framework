# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/BipedalLocomotion/bindings/ContinuousDynamicalSystem)

add_bipedal_locomotion_python_module(
  NAME ContinuousDynamicalSystemBindings
  SOURCES src/LinearTimeInvariantSystem.cpp src/FloatingBaseSystemKinematics.cpp src/Module.cpp
  HEADERS ${H_PREFIX}/DynamicalSystem.h ${H_PREFIX}/LinearTimeInvariantSystem.h ${H_PREFIX}/FloatingBaseSystemKinematics.h ${H_PREFIX}/Integrator.h
          ${H_PREFIX}/Module.h
  LINK_LIBRARIES BipedalLocomotion::ContinuousDynamicalSystem
  TESTS tests/test_linear_system.py tests/test_floating_base_system_kinematics.py
  )
