# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if(FRAMEWORK_USE_icub-models)

  add_bipedal_test(
    NAME BareBonesBaseEstimator
    SOURCES FloatingBaseEstimatorTest.cpp
    LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions Eigen3::Eigen iDynTree::idyntree-modelio-urdf icub-models::icub-models)


  add_bipedal_test(
    NAME InvariantEKFBaseEstimatorTest
    SOURCES InvariantEKFBaseEstimatorTest.cpp
    LINKS BipedalLocomotion::FloatingBaseEstimators BipedalLocomotion::ParametersHandler BipedalLocomotion::ManifConversions
          Eigen3::Eigen
          MANIF::manif
          icub-models::icub-models)

endif()
