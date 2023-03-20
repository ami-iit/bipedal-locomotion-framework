# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


add_bipedal_test(
  NAME CARE
  SOURCES CARETest.cpp
  LINKS BipedalLocomotion::Math)

add_bipedal_test(
  NAME FrictionCones
  SOURCES FrictionConesTest.cpp
  LINKS BipedalLocomotion::Math)

add_bipedal_test(
  NAME Wrench
  SOURCES WrenchTest.cpp
  LINKS BipedalLocomotion::Math)


add_bipedal_test(
  NAME SchmittTrigger
  SOURCES SchmittTriggerTest.cpp
  LINKS BipedalLocomotion::Math iDynTree::idyntree-estimation)
