# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME ContinousContactModel
  SOURCES ContinousContactModelTest.cpp
  LINKS BipedalLocomotion::ContactModels BipedalLocomotion::ParametersHandler Eigen3::Eigen)
