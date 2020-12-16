# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_Perception AND FRAMEWORK_COMPILE_RealsenseCapture)
  set(H_PREFIX include/BipedalLocomotion/Perception/Capture)
  add_bipedal_locomotion_library(
    NAME                   PerceptionCapture
    SOURCES                src/RealSense.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/RealSense.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::RobotInterface ${realsense2_LIBRARY})
endif()

