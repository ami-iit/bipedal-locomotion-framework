# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
    NAME ContactList
    SOURCES ContactListTest.cpp
    LINKS BipedalLocomotion::Contacts Eigen3::Eigen)

add_bipedal_test(
  NAME ContactPhaseList
  SOURCES ContactPhaseListTest.cpp
  LINKS BipedalLocomotion::Contacts)

add_bipedal_test(
  NAME GlobalCoPEvaluator
  SOURCES GlobalCoPEvaluatorTest.cpp
  LINKS BipedalLocomotion::Contacts)


