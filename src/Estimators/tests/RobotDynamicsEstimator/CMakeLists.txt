# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_USE_icub-models AND FRAMEWORK_COMPILE_YarpImplementation)

    add_bipedal_test(NAME SubModelCreator
                     SOURCES SubModelCreatorTest.cpp
                     LINKS BipedalLocomotion::RobotDynamicsEstimator
                           BipedalLocomotion::ParametersHandler
                           BipedalLocomotion::ParametersHandlerYarpImplementation
                           icub-models::icub-models)

    add_bipedal_test(NAME SubModelKinDynWrapper
                     SOURCES SubModelKinDynWrapperTest.cpp
                     LINKS BipedalLocomotion::RobotDynamicsEstimator
                           BipedalLocomotion::ParametersHandler
                           BipedalLocomotion::ParametersHandlerYarpImplementation
                           BipedalLocomotion::ManifConversions
                           icub-models::icub-models
                           Eigen3::Eigen
                           MANIF::manif
                           )

    include_directories(${CMAKE_CURRENT_BINARY_DIR})
    configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

endif()