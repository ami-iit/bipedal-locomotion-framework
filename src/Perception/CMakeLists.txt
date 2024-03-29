# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_Perception)
  if(FRAMEWORK_COMPILE_RealsenseCapture)
    set(H_PREFIX include/BipedalLocomotion/Perception/Capture)
    add_bipedal_locomotion_library(
      NAME                   PerceptionCapture
      SOURCES                src/RealSense.cpp
      PUBLIC_HEADERS         ${H_PREFIX}/RealSense.h
      PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging BipedalLocomotion::PerceptionInterface ${realsense2_LIBRARY}
      INSTALLATION_FOLDER    Perception/Capture)
  endif()

  set(H_PREFIX include/BipedalLocomotion/Perception/Features)
  add_bipedal_locomotion_library(
    NAME                   PerceptionFeatures
    SOURCES                src/ArucoDetector.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/ArucoDetector.h ${H_PREFIX}/PointCloudProcessor.h
    SUBDIRECTORIES         tests/Perception/Features
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::GenericContainer BipedalLocomotion::CommonConversions BipedalLocomotion::System ${OpenCV_LIBS} Eigen3::Eigen BipedalLocomotion::TextLogging
    INSTALLATION_FOLDER    Perception/Features)
endif()
