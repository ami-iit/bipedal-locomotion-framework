# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


# The following is required only for testing
if(BUILD_TESTING)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/FolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/FolderPath.h" @ONLY)

  # Get the onnx model of the network
  set(MANN_ONNX_EXPECTED_MD5 d66042fdcd270d20b86e65b674eaa3fa)
  if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/model.onnx")
    file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/model.onnx" ONNX_MODEL_CHECKSUM_VARIABLE)
    string(COMPARE EQUAL ${ONNX_MODEL_CHECKSUM_VARIABLE} ${MANN_ONNX_EXPECTED_MD5} MANN_ONNX_UPDATED)
  else()
    set(MANN_ONNX_UPDATED FALSE)
  endif()

  if(NOT ${MANN_ONNX_UPDATED})
    message(STATUS "Fetching MANN onnx model from huggingface.co/ami-iit/mann...")
    file(DOWNLOAD https://huggingface.co/ami-iit/mann/resolve/3a6fa8fe38d39deae540e4aca06063e9f2b53380/ergocubSN000_26j_49e.onnx
      ${CMAKE_CURRENT_BINARY_DIR}/model.onnx
      EXPECTED_MD5 ${MANN_ONNX_EXPECTED_MD5})
  endif()

   # Get the onnx model of the vel-based network
   set(VELMANN_ONNX_EXPECTED_MD5 6ca94766f764e37c59a6fa41700f4440)
   if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/model_velbased.onnx")
     file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/model_velbased.onnx" VELMANN_ONNX_MODEL_CHECKSUM_VARIABLE)
     string(COMPARE EQUAL ${VELMANN_ONNX_MODEL_CHECKSUM_VARIABLE} ${VELMANN_ONNX_EXPECTED_MD5} VELMANN_ONNX_UPDATED)
   else()
     set(VELMANN_ONNX_UPDATED FALSE)
   endif()

   if(NOT ${VELMANN_ONNX_UPDATED})
     message(STATUS "Fetching VelMANN onnx model from huggingface.co/ami-iit/mann...")
     file(DOWNLOAD https://huggingface.co/datasets/evelyd/paper_delia_2024_ral_physics-informed_trajectory_generation_dataset/resolve/main/onnx/training_subsampled_mirrored_10x_pi_20240514-173315_ep130.onnx
       ${CMAKE_CURRENT_BINARY_DIR}/model_velbased.onnx
       EXPECTED_MD5 ${VELMANN_ONNX_EXPECTED_MD5})
   endif()

  # Get the urdf model of the robot
  set(ERGOCUB_MODEL_EXPECTED_MD5 7d24f42cb415e660abc4bbc8a52d355f)
  if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/model.urdf")
    file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/model.urdf" ERGOCUB_MODEL_CHECKSUM_VARIABLE)
    string(COMPARE EQUAL ${ERGOCUB_MODEL_CHECKSUM_VARIABLE} ${ERGOCUB_MODEL_EXPECTED_MD5} ERGOCUB_MODEL_UPDATED)
  else()
    set(ERGOCUB_MODEL_UPDATED FALSE)
  endif()

  if(NOT ${ERGOCUB_MODEL_UPDATED})
    message(STATUS "Fetching ergoCubSN000 model from icub-tech-iit/ergocub-software...")
    file(DOWNLOAD https://raw.githubusercontent.com/icub-tech-iit/ergocub-software/f28733afcbbfcc99cbac13be530a6a072f832746/urdf/ergoCub/robots/ergoCubSN000/model.urdf
      ${CMAKE_CURRENT_BINARY_DIR}/model.urdf
      EXPECTED_MD5 ${ERGOCUB_MODEL_EXPECTED_MD5})
  endif()
endif()

add_bipedal_test(
  NAME MANN
  SOURCES MANNTest.cpp
  LINKS BipedalLocomotion::ML)

add_bipedal_test(
  NAME VelMANN
  SOURCES VelMANNTest.cpp
  LINKS BipedalLocomotion::ML)

add_bipedal_test(
  NAME MANNTrajectoryGenerator
  SOURCES MANNTrajectoryGeneratorTest.cpp
  LINKS BipedalLocomotion::ML)

add_bipedal_test(
  NAME VelMANNTrajectoryGenerator
  SOURCES VelMANNTrajectoryGeneratorTest.cpp
  LINKS BipedalLocomotion::ML)
