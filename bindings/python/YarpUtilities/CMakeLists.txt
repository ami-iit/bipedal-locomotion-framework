# Copyright (C) 2022 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(TARGET BipedalLocomotion::YarpUtilities)

  set(H_PREFIX include/BipedalLocomotion/bindings/YarpUtilities)

  add_bipedal_locomotion_python_module(
    NAME YarpUtilitiesBindings
    SOURCES src/VectorsCollection.cpp src/Module.cpp
    HEADERS ${H_PREFIX}/VectorsCollection.h ${H_PREFIX}/BufferedPort.h ${H_PREFIX}/Module.h
    LINK_LIBRARIES BipedalLocomotion::VectorsCollection ${YARP_LIBRARIES} BipedalLocomotion::YarpUtilities
    )

endif()
