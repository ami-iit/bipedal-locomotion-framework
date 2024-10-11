# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::ML)

  set(H_PREFIX include/BipedalLocomotion/bindings/ML)

  add_bipedal_locomotion_python_module(
    NAME MLBindings
    SOURCES src/Module.cpp src/MANN.cpp src/VelMANN.cpp
            src/MANNAutoregressive.cpp src/VelMANNAutoregressive.cpp
            src/MANNTrajectoryGenerator.cpp src/VelMANNTrajectoryGenerator.cpp
            src/MANNAutoregressiveInputBuilder.cpp src/VelMANNAutoregressiveInputBuilder.cpp
    HEADERS ${H_PREFIX}/Module.h ${H_PREFIX}/MANN.h ${H_PREFIX}/VelMANN.h
            ${H_PREFIX}/MANNAutoregressive.h ${H_PREFIX}/VelMANNAutoregressive.h
            ${H_PREFIX}/MANNTrajectoryGenerator.h ${H_PREFIX}/VelMANNTrajectoryGenerator.h
            ${H_PREFIX}/MANNAutoregressiveInputBuilder.h ${H_PREFIX}/VelMANNAutoregressiveInputBuilder.h
    LINK_LIBRARIES BipedalLocomotion::ML
    )

endif()
