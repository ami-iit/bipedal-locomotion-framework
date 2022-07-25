# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if (FRAMEWORK_COMPILE_ManifConversions)
    add_bipedal_test(
      NAME ManifConversionsTest
      SOURCES ManifConversionsTest.cpp
      LINKS BipedalLocomotion::ManifConversions BipedalLocomotion::CommonConversions)
endif()

if (FRAMEWORK_COMPILE_matioCppConversions)
    add_bipedal_test(
      NAME matioCppConversionsTest
      SOURCES matioCppConversionsTest.cpp
      LINKS BipedalLocomotion::matioCppConversions)
endif()

