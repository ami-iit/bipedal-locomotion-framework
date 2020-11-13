# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_bipedal_locomotion_library(
    NAME                   CommonConversions
    IS_INTERFACE
    PUBLIC_HEADERS         include/BipedalLocomotion/Conversions/CommonConversions.h
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen
    INSTALLATION_FOLDER    Conversions
    SUBDIRECTORIES         tests
    )

if (FRAMEWORK_COMPILE_ManifConversions)
    add_bipedal_locomotion_library(
        NAME                   ManifConversions
        IS_INTERFACE
        PUBLIC_HEADERS         include/BipedalLocomotion/Conversions/ManifConversions.h
        PUBLIC_LINK_LIBRARIES  MANIF::manif iDynTree::idyntree-core BipedalLocomotion::CommonConversions
        INSTALLATION_FOLDER    Conversions
        )
endif()

if (FRAMEWORK_COMPILE_matioCppConversions)
    add_bipedal_locomotion_library(
        NAME                   matioCppConversions
        IS_INTERFACE
        PUBLIC_HEADERS         include/BipedalLocomotion/Conversions/matioCppConversions.h
        PUBLIC_LINK_LIBRARIES  Eigen3::Eigen matioCpp::matioCpp BipedalLocomotion::GenericContainer BipedalLocomotion::CommonConversions
        INSTALLATION_FOLDER    Conversions
        )
endif()


