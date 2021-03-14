# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_RobotInterface AND FRAMEWORK_COMPILE_YarpImplementation)

  set(H_PREFIX include/BipedalLocomotion/bindings/RobotInterface)

  add_bipedal_locomotion_python_module(
    NAME RobotInterfaceBindings
    SOURCES src/Polydriver.cpp src/RobotControl.cpp src/SensorBridge.cpp src/Module.cpp
    PUBLIC_HEADERS ${H_PREFIX}/Polydriver.h ${H_PREFIX}/RobotControl.h ${H_PREFIX}/SensorBridge.h ${H_PREFIX}/Module.h
    PRIVATE_LINK_LIBRARIES BipedalLocomotion::RobotInterface BipedalLocomotion::RobotInterfaceYarpImplementation
    INSTALLATION_FOLDER RobotInterface
    )

endif()
