# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

include_directories(${CMAKE_CURRENT_BINARY_DIR})
configure_file("${CMAKE_CURRENT_SOURCE_DIR}/FolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/MANNModelFolderPath.h" @ONLY)

set(MANN_ONNX_EXPECTED_MD5 de9080f1b909368ffae044615d7646af)

# if (EXISTS "${CMAKE_CURRENT_BINARY_DIR}/model.onnx")
#   file(MD5 "${CMAKE_CURRENT_BINARY_DIR}/model.onnx" ONNX_MODEL_CHECKSUM_VARIABLE)
#   string(COMPARE EQUAL ${ONNX_MODEL_CHECKSUM_VARIABLE} ${MANN_ONNX_EXPECTED_MD5} MANN_ONNX_UPDATED)
# else()
#   set(MANN_ONNX_UPDATED FALSE)
# endif()

# if(NOT ${MANN_ONNX_UPDATED})
#   message(STATUS "Fetching MANN onnx model from ami-iit/mann-pytorch...")
#   file(DOWNLOAD https://raw.githubusercontent.com/ami-iit/mann-pytorch/7386952b61440c19459bbe680b818faf7f461ede/models/storage_20220909-131438/models/model_49.onnx
#     ${CMAKE_CURRENT_BINARY_DIR}/model.onnx
#     EXPECTED_MD5 ${MANN_ONNX_EXPECTED_MD5})
# endif()



add_bipedal_test(
  NAME MANN
  SOURCES MANNTest.cpp
  LINKS BipedalLocomotion::ML)
