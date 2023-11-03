# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_Estimators)
  set(H_PREFIX include/BipedalLocomotion/Estimators)
  add_bipedal_locomotion_library(
    NAME                   Estimators
    SOURCES                src/RecursiveLeastSquare.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/RecursiveLeastSquare.h
    SUBDIRECTORIES         tests/Estimators
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::System Eigen3::Eigen)
endif()

if(FRAMEWORK_COMPILE_FloatingBaseEstimators)
  set(H_PREFIX include/BipedalLocomotion/FloatingBaseEstimators)
  add_bipedal_locomotion_library(
    NAME                   FloatingBaseEstimators
    SOURCES                src/FloatingBaseEstimator.cpp src/InvariantEKFBaseEstimator.cpp src/LeggedOdometry.cpp src/BaseEstimatorFromFootIMU.cpp
    SUBDIRECTORIES         tests/FloatingBaseEstimators
    PUBLIC_HEADERS         ${H_PREFIX}/FloatingBaseEstimatorParams.h ${H_PREFIX}/FloatingBaseEstimatorIO.h ${H_PREFIX}/FloatingBaseEstimator.h ${H_PREFIX}/InvariantEKFBaseEstimator.h ${H_PREFIX}/LeggedOdometry.h ${H_PREFIX}/BaseEstimatorFromFootIMU.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions iDynTree::idyntree-high-level iDynTree::idyntree-core iDynTree::idyntree-model BipedalLocomotion::System Eigen3::Eigen BipedalLocomotion::Contacts iDynTree::idyntree-modelio-urdf BipedalLocomotion::TextLogging
                           MANIF::manif)
endif()

if(FRAMEWORK_COMPILE_RobotDynamicsEstimator)
  set(H_PREFIX include/BipedalLocomotion/RobotDynamicsEstimator)
  add_bipedal_locomotion_library(
    NAME                    RobotDynamicsEstimator
    SOURCES                 src/SubModel.cpp src/KinDynWrapper.cpp src/Dynamics.cpp src/ZeroVelocityStateDynamics.cpp src/JointVelocityStateDynamics.cpp
                            src/ConstantMeasurementModel.cpp src/AccelerometerMeasurementDynamics.cpp src/GyroscopeMeasurementDynamics.cpp
                            src/FrictionTorqueStateDynamics.cpp src/MotorCurrentMeasurementDynamics.cpp src/UkfModel.cpp
                            src/UkfState.cpp src/UkfMeasurement.cpp src/RobotDynamicsEstimator.cpp
    SUBDIRECTORIES          tests/RobotDynamicsEstimator
    PUBLIC_HEADERS          ${H_PREFIX}/SubModel.h ${H_PREFIX}/KinDynWrapper.h ${H_PREFIX}/Dynamics.h ${H_PREFIX}/ZeroVelocityStateDynamics.h
                            ${H_PREFIX}/JointVelocityStateDynamics.h ${H_PREFIX}/ConstantMeasurementModel.h ${H_PREFIX}/AccelerometerMeasurementDynamics.h
                            ${H_PREFIX}/GyroscopeMeasurementDynamics.h ${H_PREFIX}/FrictionTorqueStateDynamics.h ${H_PREFIX}/MotorCurrentMeasurementDynamics.h
                            ${H_PREFIX}/UkfModel.h ${H_PREFIX}/UkfState.h ${H_PREFIX}/UkfMeasurement.h ${H_PREFIX}/RobotDynamicsEstimator.h
    PUBLIC_LINK_LIBRARIES   BipedalLocomotion::ParametersHandler MANIF::manif BipedalLocomotion::System BipedalLocomotion::Math iDynTree::idyntree-high-level
                            iDynTree::idyntree-core iDynTree::idyntree-model iDynTree::idyntree-modelio Eigen3::Eigen BayesFilters::BayesFilters
                            BipedalLocomotion::ManifConversions
    PRIVATE_LINK_LIBRARIES  BipedalLocomotion::TextLogging
    )
endif()
