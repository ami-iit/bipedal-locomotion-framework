# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_locomotion_library(
  NAME                   TextLogging
  PUBLIC_HEADERS         include/BipedalLocomotion/TextLogging/Logger.h
                         include/BipedalLocomotion/TextLogging/LoggerBuilder.h
                         include/BipedalLocomotion/TextLogging/DefaultLogger.h
  SOURCES                src/Logger.cpp src/LoggerBuilder.cpp src/DefaultLogger.cpp
  PUBLIC_LINK_LIBRARIES  spdlog::spdlog
  SUBDIRECTORIES         YarpImplementation)
