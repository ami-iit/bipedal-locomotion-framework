# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


set(H_PREFIX include/BipedalLocomotion/bindings/type_caster)

add_bipedal_locomotion_python_module(
    NAME TypeCaster
    HEADERS ${H_PREFIX}/swig.h
    LINK_LIBRARIES iDynTree::idyntree-high-level)
