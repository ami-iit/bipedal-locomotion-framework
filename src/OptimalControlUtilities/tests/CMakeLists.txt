# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_tests)

  ## Variable Handler
  add_library(VariableHandlerTestMain VariableHandlerTestMain.cpp)
  target_link_libraries(VariableHandlerTestMain PRIVATE Catch2::Catch2)

  add_executable(VariableHandlerTest VariableHandlerTest.cpp)
  target_link_libraries(VariableHandlerTest VariableHandlerTestMain Catch2::Catch2 BipedalLocomotionControllers::OptimalControlUtilities)
  target_compile_definitions(VariableHandlerTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

  add_test(NAME VariableHandlerTest COMMAND VariableHandlerTest)

  ## Control problem elements
  add_library(ControlProblemTestMain ControlProblemTestMain.cpp)
  target_link_libraries(ControlProblemTestMain PRIVATE Catch2::Catch2)

  add_executable(ControlProblemTest ControlProblemTest.cpp)
  target_link_libraries(ControlProblemTest ControlProblemTestMain Catch2::Catch2 BipedalLocomotionControllers::OptimalControlUtilities)
  target_compile_definitions(ControlProblemTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

  add_test(NAME ControlProblemTest COMMAND ControlProblemTest)

  ## Optimization problem elements
  add_library(OptimizationProblemTestMain OptimizationProblemTestMain.cpp)
  target_link_libraries(OptimizationProblemTestMain PRIVATE Catch2::Catch2)

  add_executable(OptimizationProblemTest OptimizationProblemTest.cpp)
  target_link_libraries(OptimizationProblemTest OptimizationProblemTestMain Catch2::Catch2 BipedalLocomotionControllers::OptimalControlUtilities)
  target_compile_definitions(OptimizationProblemTest PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)

  add_test(NAME OptimizationProblemTest COMMAND OptimizationProblemTest)

endif()
