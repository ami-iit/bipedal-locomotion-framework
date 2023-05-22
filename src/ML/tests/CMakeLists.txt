# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/FolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/MANNModelFolderPath.h" @ONLY)

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



add_bipedal_test(
  NAME MANN
  SOURCES MANNTest.cpp
  LINKS BipedalLocomotion::ML)
