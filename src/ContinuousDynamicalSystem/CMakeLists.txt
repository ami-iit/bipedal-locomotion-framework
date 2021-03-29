# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if(FRAMEWORK_COMPILE_ContinuousDynamicalSystem)

  set(H_PREFIX include/BipedalLocomotion/ContinuousDynamicalSystem)

  # set target name
  add_bipedal_locomotion_library(
    NAME                   ContinuousDynamicalSystem
    PUBLIC_HEADERS         ${H_PREFIX}/DynamicalSystem.h ${H_PREFIX}/LinearTimeInvariantSystem.h
                           ${H_PREFIX}/FloatingBaseSystemKinematics.h ${H_PREFIX}/FloatingBaseDynamicsWithCompliantContacts.h ${H_PREFIX}/FixedBaseDynamics.h
                           ${H_PREFIX}/Integrator.h  ${H_PREFIX}/FixedStepIntegrator.h ${H_PREFIX}/ForwardEuler.h
                           ${H_PREFIX}/CompliantContactWrench.h ${H_PREFIX}/CentroidalDynamics.h
    PRIVATE_HEADERS        ${H_PREFIX}/impl/traits.h
    SOURCES                src/LinearTimeInvariantSystem.cpp src/FloatingBaseSystemKinematics.cpp src/CompliantContactWrench.cpp src/FloatingBaseDynamicsWithCompliantContacts.cpp src/FixedBaseDynamics.cpp src/CentroidalDynamics.cpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::ContactModels
                           iDynTree::idyntree-high-level iDynTree::idyntree-model
                           Eigen3::Eigen BipedalLocomotion::TextLogging BipedalLocomotion::Math BipedalLocomotion::Contacts
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::CommonConversions
    SUBDIRECTORIES         tests
    )

endif()
