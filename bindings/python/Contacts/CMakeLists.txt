# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::Contacts AND TARGET BipedalLocomotion::ContactDetectors)

  set(H_PREFIX include/BipedalLocomotion/bindings/Contacts)

  add_bipedal_locomotion_python_module(
    NAME ContactsBindings
    SOURCES src/Contacts.cpp src/ContactDetectors.cpp src/GlobalCoPEvaluator.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/Contacts.h ${H_PREFIX}/ContactDetectors.h ${H_PREFIX}/GlobalCoPEvaluator.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::Contacts BipedalLocomotion::ContactDetectors
    TESTS tests/test_contact.py tests/test_fixed_foot_detector.py tests/test_schmitt_trigger_detector.py)

endif()
