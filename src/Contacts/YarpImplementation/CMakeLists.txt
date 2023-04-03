# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


if(FRAMEWORK_COMPILE_YarpImplementation)

  set(H_PREFIX include/BipedalLocomotion/Contacts)

  add_bipedal_locomotion_library(
    NAME                   ContactsYarpImplementation
    SOURCES                src/PlannedContactYarpMsg.cpp src/ContactListYarpMsg.cpp
                           src/ContactListMapYarpMsg.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/PlannedContactYarpMsg.h
                           ${H_PREFIX}/ContactListYarpMsg.h
                           ${H_PREFIX}/ContactListMapYarpMsg.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::Contacts YARP::YARP_os
    INSTALLATION_FOLDER    Contacts)

endif()
