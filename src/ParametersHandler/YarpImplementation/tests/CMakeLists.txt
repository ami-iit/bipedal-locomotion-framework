# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_tests)
  add_library(ParametesHandlerYarpTestMain ParametesHandlerYarpTestMain.cpp)
  target_link_libraries(ParametesHandlerYarpTestMain PRIVATE Catch2::Catch2)

  add_executable(ParametesHandlerYarpTest ParametesHandlerYarpTest.cpp)
  target_link_libraries(ParametesHandlerYarpTest ParametesHandlerYarpTestMain Catch2::Catch2 BipedalLocomotionControllers::ParametersHandlerYarpImplementation)
  target_compile_definitions(ParametesHandlerYarpTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

  add_test(NAME ParametesHandlerYarpTest COMMAND ParametesHandlerYarpTest)
endif()
