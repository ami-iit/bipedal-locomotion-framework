# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

add_bipedal_test(NAME ZeroVelocityStateDynamics
                SOURCES ZeroVelocityStateDynamicsTest.cpp
                LINKS BipedalLocomotion::RobotDynamicsEstimator
                BipedalLocomotion::ParametersHandler
                Eigen3::Eigen)

add_bipedal_test(NAME ConstantMeasurementModel
                SOURCES ConstantMeasurementModelTest.cpp
                LINKS BipedalLocomotion::RobotDynamicsEstimator
                BipedalLocomotion::ParametersHandler
                Eigen3::Eigen)

if(BUILD_TESTING)
    set(CUSTOM_MODEL_EXPECTED_MD5 33b903fe87939e2e4932564e45741047)
    if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/model.urdf")
    file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/model.urdf" CUSTOM_MODEL_CHECKSUM_VARIABLE)
    string(COMPARE EQUAL ${CUSTOM_MODEL_CHECKSUM_VARIABLE} ${CUSTOM_MODEL_EXPECTED_MD5} CUSTOM_MODEL_UPDATED)
    else()
    set(CUSTOM_MODEL_UPDATED FALSE)
    endif()

    if(NOT ${CUSTOM_MODEL_UPDATED})
    message(STATUS "Fetching CUSTOM model from github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/...")
    file(DOWNLOAD https://raw.githubusercontent.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/577c9d734ff0cb7128274b3880106da11678538c/urdf/four_joints_two_sensors.urdf
        ${CMAKE_CURRENT_BINARY_DIR}/model.urdf
        EXPECTED_MD5 ${CUSTOM_MODEL_EXPECTED_MD5})
    endif()
endif()

if(FRAMEWORK_COMPILE_YarpImplementation)
    add_bipedal_test(NAME SubModelCreator
        SOURCES SubModelCreatorTest.cpp
        LINKS BipedalLocomotion::RobotDynamicsEstimator
        BipedalLocomotion::ParametersHandler
        BipedalLocomotion::ParametersHandlerYarpImplementation)
endif()

if(FRAMEWORK_USE_icub-models)

    if (FRAMEWORK_COMPILE_ManifConversions)
        add_bipedal_test(NAME KinDynWrapper
            SOURCES KinDynWrapperTest.cpp
            LINKS BipedalLocomotion::RobotDynamicsEstimator
            BipedalLocomotion::ParametersHandler
            icub-models::icub-models
            BipedalLocomotion::ManifConversions)

        add_bipedal_test(NAME FrictionTorqueStateDynamics
            SOURCES FrictionTorqueStateDynamicsTest.cpp
            LINKS BipedalLocomotion::RobotDynamicsEstimator
            BipedalLocomotion::ParametersHandler
            icub-models::icub-models
            Eigen3::Eigen
            BipedalLocomotion::ManifConversions
            iDynTree::idyntree-model)

        add_bipedal_test(NAME GyroscopeMeasurementDynamics
            SOURCES GyroscopeMeasurementDynamicsTest.cpp
            LINKS BipedalLocomotion::RobotDynamicsEstimator
            BipedalLocomotion::ParametersHandler
            icub-models::icub-models
            Eigen3::Eigen
            BipedalLocomotion::ManifConversions
            iDynTree::idyntree-model)

        add_bipedal_test(NAME AccelerometerMeasurementDynamics
            SOURCES AccelerometerMeasurementDynamicsTest.cpp
            LINKS   BipedalLocomotion::RobotDynamicsEstimator
            BipedalLocomotion::ParametersHandler
            icub-models::icub-models
            Eigen3::Eigen
            MANIF::manif
            BipedalLocomotion::ManifConversions
            iDynTree::idyntree-model)

        add_bipedal_test(NAME JointVelocityStateDynamics
            SOURCES JointVelocityStateDynamicsTest.cpp
            LINKS BipedalLocomotion::RobotDynamicsEstimator
            BipedalLocomotion::ParametersHandler
            icub-models::icub-models
            Eigen3::Eigen
            MANIF::manif
            BipedalLocomotion::ManifConversions
            iDynTree::idyntree-model)
    endif()
endif()

if(FRAMEWORK_COMPILE_YarpImplementation AND FRAMEWORK_COMPILE_ManifConversions AND FRAMEWORK_COMPILE_matioCppConversions)
    if(BUILD_TESTING)
        set(DATASET_UPDATED FALSE)
        set(DATASET_EXPECTED_MD5 ac0c11c75616f71ccd4d8858f9d063d8)
        if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/dataset.mat")
            file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/dataset.mat" MAT_DATASET_CHECKSUM_VARIABLE)
            string(COMPARE EQUAL ${MAT_DATASET_CHECKSUM_VARIABLE} ${DATASET_EXPECTED_MD5} DATASET_UPDATED)
        else()
                set(DATASET_UPDATED FALSE)
        endif()

        if(NOT ${DATASET_UPDATED})
        message(STATUS "Fetching synthetic dataset from https://github.com/ami-iit/paper_...")
        file(DOWNLOAD https://github.com/ami-iit/paper_sorrentino_2024_icra_robot-dynamics-estimation/raw/main/python/dataset/synthetic_data/four_joints_two_ft_two_imu.mat?download=
            ${CMAKE_CURRENT_BINARY_DIR}/dataset.mat
            EXPECTED_MD5 ${DATASET_EXPECTED_MD5})
        endif()
    endif()

    add_bipedal_test(NAME RobotDynamicsEstimation
        SOURCES RobotDynamicsEstimationTest.cpp
        LINKS   BipedalLocomotion::RobotDynamicsEstimator
        BipedalLocomotion::ParametersHandler
        Eigen3::Eigen
        MANIF::manif
        BipedalLocomotion::ManifConversions
        iDynTree::idyntree-model
        BipedalLocomotion::ParametersHandlerYarpImplementation
        matioCpp::matioCpp
        BipedalLocomotion::matioCppConversions
    )
endif()
