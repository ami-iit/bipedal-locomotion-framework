# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_subdirectory(FloatingBaseEstimatorDevice)
add_subdirectory(RobotDynamicsEstimatorDevice)
add_subdirectory(YarpRobotLoggerDevice)
add_subdirectory(VectorsCollectionWrapper)

if (FRAMEWORK_COMPILE_example_devices)
  add_subdirectory(examples)
endif()

