# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if (FRAMEWORK_COMPILE_ML)

  set(H_PREFIX include/BipedalLocomotion/ML)

  add_bipedal_locomotion_library(
    NAME ML
    PUBLIC_HEADERS ${H_PREFIX}/MANN.h ${H_PREFIX}/VelMANN.h
                   ${H_PREFIX}/MANNAutoregressive.h ${H_PREFIX}/VelMANNAutoregressive.h
                   ${H_PREFIX}/MANNTrajectoryGenerator.h ${H_PREFIX}/VelMANNTrajectoryGenerator.h
                   ${H_PREFIX}/MANNAutoregressiveInputBuilder.h ${H_PREFIX}/VelMANNAutoregressiveInputBuilder.h
    SOURCES        src/MANN.cpp src/VelMANN.cpp
                   src/MANNAutoregressive.cpp src/VelMANNAutoregressive.cpp
                   src/MANNTrajectoryGenerator.cpp src/VelMANNTrajectoryGenerator.cpp
                   src/MANNAutoregressiveInputBuilder.cpp src/VelMANNAutoregressiveInputBuilder.cpp
    PUBLIC_LINK_LIBRARIES Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System
                          BipedalLocomotion::ContinuousDynamicalSystem BipedalLocomotion::Contacts
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging onnxruntime::onnxruntime BipedalLocomotion::ManifConversions
    INSTALLATION_FOLDER ML)

  add_subdirectory(tests)

endif()
