# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
    NAME YarpUtilities
    SOURCES YarpUtilitiesTest.cpp
    LINKS BipedalLocomotion::YarpUtilities
    )

add_bipedal_test(
    NAME VectorsCollectionTest
    SOURCES VectorsCollectionTest.cpp
    LINKS BipedalLocomotion::YarpUtilities BipedalLocomotion::VectorsCollection
    DISABLE_VALGRIND_TEST
)
