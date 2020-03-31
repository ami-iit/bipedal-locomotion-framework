# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_Simulator)

  add_bipedal_component(
    NAME                   Simulator
    SOURCES                src/Simulator.cpp src/Visualizer.cpp
    PUBLIC_HEADERS         include/BipedalLocomotionControllers/Simulator/Simulator.h include/BipedalLocomotionControllers/Simulator/Integrator.h include/BipedalLocomotionControllers/Simulator/Integrator.tpp include/BipedalLocomotionControllers/Simulator/Visualizer.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotionControllers::ParametersHandler BipedalLocomotionControllers::ContactModels iDynTree::idyntree-core)

endif()
