# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
add_bipedal_locomotion_library(
    NAME                   GenericContainer
    PUBLIC_HEADERS         include/BipedalLocomotion/GenericContainer/Vector.h include/BipedalLocomotion/GenericContainer/TemplateHelpers.h
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core Eigen3::Eigen
    SUBDIRECTORIES   tests
    IS_INTERFACE)


