# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_Contact)

  set(H_PREFIX include/BipedalLocomotion/bindings/Contacts)

  add_bipedal_locomotion_python_module(
    NAME ContactsBindings
    SOURCES src/Contacts.cpp src/ContactDetectors.cpp  src/Module.cpp
    PUBLIC_HEADERS ${H_PREFIX}/Contacts.h ${H_PREFIX}/ContactDetectors.h ${H_PREFIX}/Module.h
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::Contacts BipedalLocomotion::ContactDetectors
    INSTALLATION_FOLDER Contacts
    TESTS tests/test_contact.py tests/test_schmitt_trigger_detector.py)

endif()
