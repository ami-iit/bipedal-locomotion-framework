# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/BipedalLocomotion/bindings/TextLogging)

add_bipedal_locomotion_python_module(
  NAME TextLogging
  SOURCES
  src/Module.cpp
  src/TextLogging.cpp
  HEADERS
  ${H_PREFIX}/Module.h
  ${H_PREFIX}/TextLogging.h
  LINK_LIBRARIES
  BipedalLocomotion::TextLogging
  )
