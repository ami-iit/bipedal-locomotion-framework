# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

include(BipedalLocomotionFrameworkFindDependencies)

################################################################################
# Find all packages

find_package(iDynTree 1.1.0 REQUIRED) #Right now, all the packages built in the framework
                                      #depend directly or indirectly from iDynTree and Eigen
                                      #(which is an iDynTree dependency by the way)
find_package(Eigen3 3.2.92 REQUIRED)

find_package(spdlog REQUIRED)

find_package(YARP QUIET)
checkandset_dependency(YARP)

find_package(Qhull 8.0.0 QUIET)
checkandset_dependency(Qhull)

find_package(casadi QUIET)
checkandset_dependency(casadi)

find_package(cppad QUIET)
checkandset_dependency(cppad)

find_package(manif QUIET)
checkandset_dependency(manif)

find_package(OsqpEigen QUIET)
checkandset_dependency(OsqpEigen)

find_package(Python3 3.6 COMPONENTS Interpreter Development QUIET)
checkandset_dependency(Python3 MINIMUM_VERSION 3.6 COMPONENTS Interpreter Development)

find_package(pybind11 2.2 CONFIG QUIET)
checkandset_dependency(pybind11)

find_package(matioCpp QUIET)
checkandset_dependency(matioCpp)

find_package(LieGroupControllers QUIET)
checkandset_dependency(LieGroupControllers)

find_package(OpenCV QUIET)
checkandset_dependency(OpenCV)

find_package(PCL QUIET)
checkandset_dependency(PCL)

find_package(realsense2 QUIET)
checkandset_dependency(realsense2)

find_package(Catch2 QUIET)
checkandset_dependency(Catch2)

find_package(VALGRIND QUIET)
checkandset_dependency(VALGRIND)

framework_dependent_option(FRAMEWORK_COMPILE_tests
  "Compile tests?" ON
  "FRAMEWORK_USE_Catch2;BUILD_TESTING" OFF)

framework_dependent_option(FRAMEWORK_RUN_Valgrind_tests
  "Run Valgrind tests?" OFF
  "FRAMEWORK_COMPILE_tests;VALGRIND_FOUND" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_YarpUtilities
  "Compile YarpHelper library?" ON
  "FRAMEWORK_USE_YARP" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_YarpImplementation
  "Compile All the YARP implementations?" ON
  "FRAMEWORK_COMPILE_YarpUtilities" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Estimators
  "Compile Estimators library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Contact
  "Compile Contact libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Planners
  "Compile Planners libraries?" ON
  "FRAMEWORK_USE_Qhull;FRAMEWORK_USE_casadi;FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ContactModels
  "Compile ContactModels library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_System
  "Compile System library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ContinuousDynamicalSystem
  "Compile System ContinuousDynamicalSystem?" ON
  "FRAMEWORK_COMPILE_ContactModels" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_AutoDiffCppAD
  "Compile CppAD-Eigen wrapper?" ON
  "FRAMEWORK_USE_cppad" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RobotInterface
  "Compile RobotInterface libraries?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_FloatingBaseEstimators
  "Compile FloatingBaseEstimators libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ManifConversions
  "Compile manif Conversions libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_matioCppConversions
  "Compile matioCpp Conversions libraries?" ON
  "FRAMEWORK_USE_matioCpp" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_TSID
  "Compile TSID library?" ON
  "FRAMEWORK_COMPILE_System;FRAMEWORK_USE_LieGroupControllers;FRAMEWORK_COMPILE_ManifConversions;FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_IK
  "Compile IK library?" ON
  "FRAMEWORK_COMPILE_System;FRAMEWORK_USE_LieGroupControllers;FRAMEWORK_COMPILE_ManifConversions;FRAMEWORK_USE_manif;FRAMEWORK_USE_OsqpEigen" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointPositionTrackingApplication
  "Compile joint-position-tracking application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Planners;FRAMEWORK_COMPILE_RobotInterface" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_PYTHON_BINDINGS
  "Do you want to generate and compile the Python bindings?" ON
  "FRAMEWORK_USE_Python3;FRAMEWORK_USE_pybind11;FRAMEWORK_COMPILE_Planners;FRAMEWORK_COMPILE_System;FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_YarpImplementation" OFF)

framework_dependent_option(FRAMEWORK_TEST_PYTHON_BINDINGS
  "Do you want to test the Python bindings?" ON
  "FRAMEWORK_COMPILE_tests;FRAMEWORK_COMPILE_PYTHON_BINDINGS" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_MasImuTest
  "Compile test on the MAS IMU?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_matioCppConversions" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointTrajectoryPlayer
  "Compile joint-trajectory-player application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Planners;FRAMEWORK_COMPILE_RobotInterface;FRAMEWORK_COMPILE_matioCppConversions;FRAMEWORK_USE_matioCpp;FRAMEWORK_USE_YARP" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Perception
  "Compile Perception libraries?" ON
  "FRAMEWORK_USE_OpenCV;FRAMEWORK_USE_PCL" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_PerceptionInterface
  "Compile PerceptionInterface libraries?" ON
  "FRAMEWORK_COMPILE_Perception" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RealsenseCapture
  "Compile Realsense related software?" ON
  "FRAMEWORK_COMPILE_PerceptionInterface;FRAMEWORK_USE_realsense2" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RealSenseTestApplication
  "Compile realsense-test application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Perception;FRAMEWORK_COMPILE_RealsenseCapture;FRAMEWORK_COMPILE_PerceptionInterface" OFF)
