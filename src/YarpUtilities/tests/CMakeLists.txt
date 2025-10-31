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
)

# Set different port ranges for the two tests to avoid address conflicts
set_property(TEST VectorsCollectionTestUnitTests APPEND PROPERTY ENVIRONMENT "YARP_PORT_RANGE=20000")
set_property(TEST memcheck_VectorsCollectionTestUnitTests APPEND PROPERTY ENVIRONMENT "YARP_PORT_RANGE=21000")
