# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_ContactModels)

    add_bipedal_component(
      NAME                   ContactModels
      SOURCES                src/ContactModel.cpp src/ContinuousContactModel.cpp
      PUBLIC_HEADERS         include/BipedalLocomotionControllers/ContactModels/ContactModel.h include/BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h
      PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core BipedalLocomotionControllers::ParametersHandler
      SUBDIRECTORIES         tests)

endif()
