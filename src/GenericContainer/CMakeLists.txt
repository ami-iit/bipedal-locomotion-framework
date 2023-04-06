# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# set target name

set(H_PREFIX include/BipedalLocomotion/GenericContainer)

add_bipedal_locomotion_library(
    NAME                   GenericContainer
    PUBLIC_HEADERS         ${H_PREFIX}/Vector.h ${H_PREFIX}/TemplateHelpers.h ${H_PREFIX}/NamedTuple.h
    PUBLIC_LINK_LIBRARIES  iDynTree::idyntree-core Eigen3::Eigen
    SUBDIRECTORIES         tests
    IS_INTERFACE)


