# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


if (FRAMEWORK_COMPILE_Contact)
  set(H_PREFIX include/BipedalLocomotion/Contacts)
  add_bipedal_locomotion_library(
    NAME                  Contacts
    SUBDIRECTORIES        tests/Contacts
    PUBLIC_HEADERS        ${H_PREFIX}/Contact.h ${H_PREFIX}/ContactList.h ${H_PREFIX}/ContactPhase.h ${H_PREFIX}/ContactPhaseList.h ${H_PREFIX}/ContactListJsonParser.h
                          ${H_PREFIX}/GlobalCoPEvaluator.h
    SOURCES               src/Contact.cpp src/ContactList.cpp src/ContactPhase.cpp src/ContactPhaseList.cpp src/ContactListJsonParser.cpp
                          src/GlobalCoPEvaluator.cpp
    PUBLIC_LINK_LIBRARIES MANIF::manif BipedalLocomotion::Math BipedalLocomotion::TextLogging
    PRIVATE_LINK_LIBRARIES nlohmann_json::nlohmann_json
    INSTALLATION_FOLDER   Contacts)

  set(H_PREFIX include/BipedalLocomotion/ContactDetectors)
    add_bipedal_locomotion_library(
    NAME                   ContactDetectors
    SUBDIRECTORIES         tests/ContactDetectors
    SOURCES                src/ContactDetector.cpp src/SchmittTriggerDetector.cpp src/FixedFootDetector.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/ContactDetector.h ${H_PREFIX}/SchmittTriggerDetector.h ${H_PREFIX}/FixedFootDetector.h
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::TextLogging
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts)
endif()
