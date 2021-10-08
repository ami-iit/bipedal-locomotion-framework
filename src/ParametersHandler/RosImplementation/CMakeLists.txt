# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_RosImplementation)

  add_bipedal_locomotion_library(
      NAME                   ParametersHandlerRosImplementation
      SOURCES                src/RosImplementation.cpp
      PUBLIC_HEADERS         include/BipedalLocomotion/ParametersHandler/RosImplementation.h include/BipedalLocomotion/ParametersHandler/RosImplementation.tpp
      PUBLIC_LINK_LIBRARIES  BipedalLocomotion::ParametersHandler BipedalLocomotion::TextLogging ${roscpp_LIBRARIES}
      ADDITIONAL_INCLUDE_DIRECTORIES ${roscpp_INCLUDE_DIRS}
      INSTALLATION_FOLDER ParametersHandler)

endif()
