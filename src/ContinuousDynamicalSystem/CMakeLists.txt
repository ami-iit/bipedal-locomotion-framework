# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


if(FRAMEWORK_COMPILE_ContinuousDynamicalSystem)

  set(H_PREFIX include/BipedalLocomotion/ContinuousDynamicalSystem)

  # set target name
  add_bipedal_locomotion_library(
    NAME                   ContinuousDynamicalSystem
    PUBLIC_HEADERS         ${H_PREFIX}/DynamicalSystem.h ${H_PREFIX}/LinearTimeInvariantSystem.h
                           ${H_PREFIX}/FloatingBaseSystemKinematics.h ${H_PREFIX}/FloatingBaseDynamicsWithCompliantContacts.h ${H_PREFIX}/FixedBaseDynamics.h ${H_PREFIX}/FirstOrderSmoother.h
                           ${H_PREFIX}/CentroidalDynamics.h
                           ${H_PREFIX}/LieGroupDynamics.h ${H_PREFIX}/SO3Dynamics.h
                           ${H_PREFIX}/Integrator.h  ${H_PREFIX}/FixedStepIntegrator.h ${H_PREFIX}/ForwardEuler.h ${H_PREFIX}/RK4.h
                           ${H_PREFIX}/CompliantContactWrench.h
                           ${H_PREFIX}/MultiStateWeightProvider.h
                           ${H_PREFIX}/ButterworthLowPassFilter.h
    PRIVATE_HEADERS        ${H_PREFIX}/impl/traits.h
    SOURCES                src/LinearTimeInvariantSystem.cpp src/FloatingBaseSystemKinematics.cpp src/CompliantContactWrench.cpp
                           src/FloatingBaseDynamicsWithCompliantContacts.cpp src/FixedBaseDynamics.cpp src/CentroidalDynamics.cpp
                           src/FirstOrderSmoother.cpp src/MultiStateWeightProvider.cpp
                           src/ButterworthLowPassFilter.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::ContactModels BipedalLocomotion::System
                           iDynTree::idyntree-high-level iDynTree::idyntree-model
                           Eigen3::Eigen BipedalLocomotion::TextLogging BipedalLocomotion::Math BipedalLocomotion::Contacts
                           BipedalLocomotion::GenericContainer
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::CommonConversions
    SUBDIRECTORIES         tests
    )

endif()
