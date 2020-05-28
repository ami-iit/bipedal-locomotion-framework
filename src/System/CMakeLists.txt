# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# set target name
add_bipedal_locomotion_library(
    NAME           System
    PUBLIC_HEADERS include/BipedalLocomotion/System/Advanceable.h
    IS_INTERFACE #This will be removed when adding also sources.
    )
