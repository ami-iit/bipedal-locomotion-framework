# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
  NAME ContinousContactModel
  SOURCES ContinousContactModelTest.cpp
  LINKS BipedalLocomotion::ContactModels BipedalLocomotion::ParametersHandlerYarpImplementation)
