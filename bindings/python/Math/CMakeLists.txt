# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/BipedalLocomotion/bindings/Math)

add_bipedal_locomotion_python_module(
  NAME MathBindings
  SOURCES src/Constants.cpp src/Module.cpp
  PUBLIC_HEADERS ${H_PREFIX}/Constants.h ${H_PREFIX}/Module.h
  PRIVATE_LINK_LIBRARIES BipedalLocomotion::Math
  INSTALLATION_FOLDER Math
  )
