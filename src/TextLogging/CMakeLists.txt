# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_locomotion_library(
  NAME                   TextLogging
  PUBLIC_HEADERS         include/BipedalLocomotion/TextLogging/Logger.h
  SOURCES                src/Logger.cpp
  PUBLIC_LINK_LIBRARIES  spdlog::spdlog)
