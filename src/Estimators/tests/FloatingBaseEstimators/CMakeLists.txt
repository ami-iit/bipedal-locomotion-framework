# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


if(FRAMEWORK_USE_icub-models)

  add_bipedal_test(
    NAME BareBonesBaseEstimator
    SOURCES FloatingBaseEstimatorTest.cpp
    LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions Eigen3::Eigen iDynTree::idyntree-modelio icub-models::icub-models)


  add_bipedal_test(
    NAME InvariantEKFBaseEstimatorTest
    SOURCES InvariantEKFBaseEstimatorTest.cpp
    LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions
          Eigen3::Eigen
          MANIF::manif
          icub-models::icub-models)

  add_bipedal_test(
    NAME BaseEstimatorFromFootIMU
    SOURCES BaseEstimatorFromFootIMUTest.cpp
    LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions
          Eigen3::Eigen
          iDynTree::idyntree-modelio-urdf
          icub-models::icub-models)

endif()
