# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_WholeBodyControllers)

  add_bipedal_component(
    NAME                   WholeBodyControllers
    SOURCES                src/MomentumBasedControlHelper.cpp src/MomentumBasedControl.cpp
    PUBLIC_HEADERS         include/BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.h include/BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.tpp include/BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControl.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotionControllers::ParametersHandler  BipedalLocomotionControllers::OptimalControlUtilities OsqpEigen::OsqpEigen ${iDynTree_LIBRARIES})

endif()
