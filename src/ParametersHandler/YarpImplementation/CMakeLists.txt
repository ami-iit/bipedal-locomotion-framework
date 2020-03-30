# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_YarpImplementation)

  add_bipedal_component(
    NAME                   ParametersHandlerYarpImplementation
    SOURCES                src/YarpImplementation.cpp
    PUBLIC_HEADERS         include/BipedalLocomotionControllers/ParametersHandler/YarpImplementation.h include/BipedalLocomotionControllers/ParametersHandler/YarpImplementation.tpp
    PUBLIC_LINK_LIBRARIES  BipedalLocomotionControllers::ParametersHandler BipedalLocomotionControllers::YarpUtilities YARP::YARP_os
    INSTALLATION_FOLDER ParametersHandler)

endif()
