# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_tests)
  include_directories(${CMAKE_CURRENT_BINARY_DIR})
  configure_file("${CMAKE_CURRENT_SOURCE_DIR}/ConfigFolderPath.h.in" "${CMAKE_CURRENT_BINARY_DIR}/ConfigFolderPath.h" @ONLY)

  add_library(ParametersHandlerTestMain ParametersHandlerTestMain.cpp)
  target_link_libraries(ParametersHandlerTestMain PRIVATE Catch2::Catch2)

  add_executable(ParametersHandlerTest ParametersHandlerTest.cpp)
  target_link_libraries(ParametersHandlerTest ParametersHandlerTestMain Catch2::Catch2 BipedalLocomotionControllers::ParametersHandler)
  target_compile_definitions(ParametersHandlerTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
  target_compile_features(ParametersHandlerTest PUBLIC cxx_std_17)

  add_test(NAME ParametersHandlerTest COMMAND ParametersHandlerTest)

  if(BIPEDAL_LOCOMOTION_CONTROLLERS_RUN_Valgrind_tests)
    add_test(NAME memcheck_ParametersHandlerTest COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:ParametersHandlerTest>)
  endif()

  if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_YarpImplementation)
    add_library(ParametersHandlerYarpTestMain ParametersHandlerYarpTestMain.cpp)
    target_link_libraries(ParametersHandlerYarpTestMain PRIVATE Catch2::Catch2)

    add_executable(ParametersHandlerYarpTest ParametersHandlerYarpTest.cpp)
    target_link_libraries(ParametersHandlerYarpTest ParametersHandlerYarpTestMain Catch2::Catch2 BipedalLocomotionControllers::ParametersHandlerYarpImplementation)
    target_compile_definitions(ParametersHandlerYarpTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

    add_test(NAME ParametersHandlerYarpTest COMMAND ParametersHandlerYarpTest)

    if(BIPEDAL_LOCOMOTION_CONTROLLERS_RUN_Valgrind_tests)
      add_test(NAME memcheck_ParametersHandlerYarpTest COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:ParametersHandlerYarpTest>)
    endif()

  endif()
endif()
