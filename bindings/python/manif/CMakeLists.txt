# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_USE_manif)

  set(H_PREFIX include/BipedalLocomotion/bindings/manif)

  add_bipedal_locomotion_python_module(
    NAME manifBindings
    SOURCES src/BaseTypes.cpp src/Module.cpp
    PUBLIC_HEADERS ${H_PREFIX}/BaseTypes.h ${H_PREFIX}/Module.h
    PRIVATE_LINK_LIBRARIES MANIF::manif
    INSTALLATION_FOLDER manif
    TESTS tests/test_base_types.py
    )

endif()
