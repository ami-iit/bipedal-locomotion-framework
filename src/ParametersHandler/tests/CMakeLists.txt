# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_tests)
  add_library(ParametersHandlerTestMain ParametersHandlerTestMain.cpp)
  target_link_libraries(ParametersHandlerTestMain PRIVATE Catch2::Catch2)

  add_executable(ParametersHandlerTest ParametersHandlerTest.cpp)
  target_link_libraries(ParametersHandlerTest ParametersHandlerTestMain Catch2::Catch2 BipedalLocomotionControllers::ParametersHandler)
  target_compile_definitions(ParametersHandlerTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
  target_compile_features(ParametersHandlerTest PUBLIC cxx_std_17)

  add_test(NAME ParametersHandlerTest COMMAND ParametersHandlerTest)
endif()
