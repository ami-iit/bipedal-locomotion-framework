# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_tests)

  ## Variable Handler
  add_library(ContinousContactModelTestMain ContinousContactModelTestMain.cpp)
  target_link_libraries(ContinousContactModelTestMain PRIVATE Catch2::Catch2)

  add_executable(ContinousContactModelTest ContinousContactModelTest.cpp)
  target_link_libraries(ContinousContactModelTest ContinousContactModelTestMain
    Catch2::Catch2 BipedalLocomotionControllers::ContactModels)
  target_compile_definitions(ContinousContactModelTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

  add_test(NAME ContinousContactModelTest COMMAND ContinousContactModelTest)

endif()
