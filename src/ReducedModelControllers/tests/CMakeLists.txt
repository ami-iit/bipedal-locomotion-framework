# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
  NAME CentroidalMPC
  SOURCES CentroidalMPCTest.cpp
  LINKS BipedalLocomotion::ReducedModelControllers  BipedalLocomotion::Math BipedalLocomotion::ContinuousDynamicalSystem)
