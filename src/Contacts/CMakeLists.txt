# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


if (FRAMEWORK_COMPILE_Contact)
  set(H_PREFIX include/BipedalLocomotion/Contacts)

  add_bipedal_locomotion_library(
    NAME                  Contacts
    IS_INTERFACE
    PUBLIC_HEADERS        ${H_PREFIX}/Contact.h 
    PUBLIC_LINK_LIBRARIES MANIF::manif
    INSTALLATION_FOLDER   Contacts)

endif()


