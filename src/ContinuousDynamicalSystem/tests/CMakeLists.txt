# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_test(
  NAME IntegratorLinearSystemTest
  SOURCES IntegratorLinearSystem.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem Eigen3::Eigen)


add_bipedal_test(
  NAME IntegratorFloatingBaseSystemKinematicsTest
  SOURCES IntegratorFloatingBaseSystemKinematics.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem Eigen3::Eigen)

add_bipedal_test(
  NAME FirstOrderSmootherTest
  SOURCES FirstOrderSmoother.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem Eigen3::Eigen)

add_bipedal_test(
  NAME MultiStateWeightProviderTest
  SOURCES MultiStateWeightProvider.cpp
  LINKS BipedalLocomotion::ContinuousDynamicalSystem Eigen3::Eigen)
