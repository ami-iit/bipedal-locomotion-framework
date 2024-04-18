# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME IntegratorLinearSystemTest
  SOURCES IntegratorLinearSystem.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem
        BipedalLocomotion::TestUtils Eigen3::Eigen)


add_bipedal_test(
  NAME IntegratorFloatingBaseSystemKinematicsTest
  SOURCES IntegratorFloatingBaseSystemKinematics.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem
        BipedalLocomotion::TestUtils Eigen3::Eigen)

add_bipedal_test(
  NAME FirstOrderSmootherTest
  SOURCES FirstOrderSmoother.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem
        BipedalLocomotion::TestUtils Eigen3::Eigen)

add_bipedal_test(
  NAME MultiStateWeightProviderTest
  SOURCES MultiStateWeightProvider.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem
        BipedalLocomotion::TestUtils Eigen3::Eigen)

add_bipedal_test(
  NAME ButterworthLowPassFilterTest
  SOURCES ButterworthLowPassFilter.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem
        Eigen3::Eigen)
