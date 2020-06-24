# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_locomotion_library(
  NAME                  ParametersHandler
  PUBLIC_HEADERS        include/BipedalLocomotion/ParametersHandler/IParametersHandler.h include/BipedalLocomotion/ParametersHandler/StdImplementation.h
  SOURCES                src/StdImplementation.cpp
  PUBLIC_LINK_LIBRARIES BipedalLocomotion::GenericContainer
  SUBDIRECTORIES        tests YarpImplementation)
