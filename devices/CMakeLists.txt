# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_subdirectory(FloatingBaseEstimatorDevice)
add_subdirectory(YarpRobotLoggerDevice)

if (FRAMEWORK_COMPILE_example_devices)
  add_subdirectory(examples)
endif()

