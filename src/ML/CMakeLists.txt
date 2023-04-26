# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if (FRAMEWORK_COMPILE_ML)

  set(H_PREFIX include/BipedalLocomotion/ML)

  add_bipedal_locomotion_library(
    NAME ML
    PUBLIC_HEADERS ${H_PREFIX}/MANN.h
    SOURCES        src/MANN.cpp
    PUBLIC_LINK_LIBRARIES Eigen3::Eigen BipedalLocomotion::ParametersHandler BipedalLocomotion::System
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging onnxruntime::onnxruntime
    INSTALLATION_FOLDER ML)

  add_subdirectory(tests)

endif()
