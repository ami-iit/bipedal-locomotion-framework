# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/BipedalLocomotion/Math)

add_bipedal_locomotion_library(
  NAME                  Math
  PUBLIC_HEADERS        ${H_PREFIX}/AlgebraicRiccatiEquation.h
  SOURCES               src/AlgebraicRiccatiEquation.cpp
  PUBLIC_LINK_LIBRARIES Eigen3::Eigen
  SUBDIRECTORIES        tests)
