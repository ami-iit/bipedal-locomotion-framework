# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


add_bipedal_test(
 NAME SchmittTriggerDetector
 SOURCES SchmittTriggerDetectorUnitTest.cpp
 LINKS BipedalLocomotion::ContactDetectors BipedalLocomotion::ParametersHandler)

add_bipedal_test(
 NAME FixedFootDetector
 SOURCES FixedFootDetectorUnitTest.cpp
 LINKS BipedalLocomotion::ContactDetectors BipedalLocomotion::ParametersHandler)
