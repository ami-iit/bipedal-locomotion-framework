# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

set(H_PREFIX include/BipedalLocomotion/bindings/Math)

add_bipedal_locomotion_python_module(
  NAME MathBindings
  SOURCES src/Constants.cpp src/SchmittTrigger.cpp src/Module.cpp
  HEADERS ${H_PREFIX}/Constants.h ${H_PREFIX}/SchmittTrigger.h ${H_PREFIX}/Wrench.h ${H_PREFIX}/Module.h
  LINK_LIBRARIES BipedalLocomotion::Math
  )
