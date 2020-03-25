# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
add_bipedal_component(
    NAME                   GenericContainer
    PUBLIC_HEADERS         include/BipedalLocomotionControllers/GenericContainer/Vector.h include/BipedalLocomotionControllers/GenericVector/TemplateHelpers.h
    PUBLIC_LINK_LIBRARIES ${iDynTree_LIBRARIES}
    SUBDIRECTORIES   tests
    IS_INTERFACE     ON)


