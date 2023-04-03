# Copyright (C) 2023 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


if(FRAMEWORK_COMPILE_YarpImplementation)

  add_bipedal_test(
    NAME ContactsYarpMsg
    SOURCES ContactsYarpMsgTest.cpp
    LINKS BipedalLocomotion::ContactsYarpImplementation YARP::YARP_init)

endif()
