# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

set(H_PREFIX include/BipedalLocomotion/DCMWalkingPipeline)

find_package(LieGroupControllers REQUIRED)
find_package(WalkingControllers REQUIRED)
find_package(OsqpEigen REQUIRED)


yarp_add_idl(DCMWalkingPipeline_GEN_SRC
  thrifts/BipedalLocomotion/DCMWalkingPipeline/Commands.thrift)

add_bipedal_locomotion_library(
    NAME                   DCMWalkingPipeline
    SOURCES                src/Pipeline.cpp src/Homing.cpp
    PUBLIC_HEADERS         ${H_PREFIX}/Pipeline.h ${H_PREFIX}/Homing.h
    PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::Planners Eigen3::Eigen LieGroupControllers::LieGroupControllers WalkingControllers::WholeBodyControllers)

  add_executable(DCMWalkingApplication
    src/main.cpp src/Module.cpp
    ${H_PREFIX}/Module.h
    ${DCMWalkingPipeline_GEN_SRC})

  target_link_libraries(DCMWalkingApplication
    BipedalLocomotion::DCMWalkingPipeline
    BipedalLocomotion::ParametersHandlerYarpImplementation
    BipedalLocomotion::RobotInterfaceYarpImplementation
    YARP::YARP_dev)
