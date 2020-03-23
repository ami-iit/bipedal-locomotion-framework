# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
if(BIPEDAL_LOCOMOTION_CONTROLLERS_COMPILE_YarpUtilities)

  add_bipedal_component(
    NAME                   YarpUtilities
    SOURCES                src/Helper.cpp
    PUBLIC_HEADERS         include/BipedalLocomotionControllers/YarpUtilities/Helper.h include/BipedalLocomotionControllers/YarpUtilities/Helper.tpp
    PUBLIC_LINK_LIBRARIES  ${YARP_LIBRARIES} ${iDynTree_LIBRARIES} BipedalLocomotionControllers::GenericContainer
    SUBDIRECTORIES         tests)

endif()
