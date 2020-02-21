# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_tests)
  add_library(YarpUtilitiesTestMain YarpUtilitiesTestMain.cpp)
  target_link_libraries(YarpUtilitiesTestMain PRIVATE Catch2::Catch2)

  add_executable(YarpUtilitiesTest YarpUtilitiesTest.cpp)
  target_link_libraries(YarpUtilitiesTest YarpUtilitiesTestMain Catch2::Catch2 BipedalLocomotionControllers::YarpUtilities)
  target_compile_definitions(YarpUtilitiesTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

  add_test(NAME YarpUtilitiesTest COMMAND YarpUtilitiesTest)

  if(BIPEDAL_LOCOMOTION_CONTROLLERS_RUN_Valgrind_tests)
    add_test(NAME memcheck_YarpUtilitiesTest COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:YarpUtilitiesTest>)
  endif()

endif()
