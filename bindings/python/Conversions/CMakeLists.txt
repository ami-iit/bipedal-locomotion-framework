# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::ManifConversions)

  set(H_PREFIX include/BipedalLocomotion/bindings/Conversions)

  add_bipedal_locomotion_python_module(
    NAME ManifConversionsBindings
    SOURCES src/ManifConversions.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/ManifConversions.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::ManifConversions
    TESTS tests/test_manif_conversions.py)

endif()
