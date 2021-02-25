# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if (FRAMEWORK_COMPILE_Contact)
  set(H_PREFIX include/BipedalLocomotion/Contacts)
  add_bipedal_locomotion_library(
    NAME                  Contacts
    SUBDIRECTORIES        tests/Contacts
    PUBLIC_HEADERS        ${H_PREFIX}/Contact.h ${H_PREFIX}/ContactList.h ${H_PREFIX}/ContactPhase.h ${H_PREFIX}/ContactPhaseList.h
    SOURCES               src/Contact.cpp src/ContactList.cpp src/ContactPhase.cpp src/ContactPhaseList.cpp
    PUBLIC_LINK_LIBRARIES MANIF::manif
    INSTALLATION_FOLDER   Contacts)
    
  set(H_PREFIX include/BipedalLocomotion/ContactDetectors)
    add_bipedal_locomotion_library(
    NAME                   ContactDetectors
    SUBDIRECTORIES         tests/ContactDetectors
    SOURCES                src/ContactDetector.cpp src/SchmittTriggerDetector.cpp src/ContactBayesFilter.cpp ${H_PREFIX}/ContactBayesFilter.h
    PUBLIC_HEADERS         ${H_PREFIX}/ContactDetector.h ${H_PREFIX}/SchmittTriggerDetector.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::System BipedalLocomotion::Contacts iDynTree::idyntree-core iDynTree::idyntree-model iDynTree::idyntree-solid-shapes)
endif()

