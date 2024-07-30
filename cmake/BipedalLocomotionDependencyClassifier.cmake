# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.


# Check if a package is used and set some cmake variables
function(dependency_classifier package)
  set(options PUBLIC)
  set(singleValueArgs MINIMUM_VERSION IS_USED)
  set(multiValueArgs COMPONENTS)
  cmake_parse_arguments(DC "${options}" "${singleValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PREFIX "FRAMEWORK")

  if(NOT DEFINED DC_IS_USED)
    message(FATAL_ERROR "dependency_classifier function. The IS_USED variable must be specified for the package ${package}")
  endif()

  if(${DC_IS_USED})

    set(dependency "${package}")

    if (DC_MINIMUM_VERSION)
      set(dependency "${dependency} ${DC_MINIMUM_VERSION}")
    endif()

    if (DC_COMPONENTS)
      # DC_COMPONENTS contains the components separated by ";" we now replace ";" with " " this is
      # required to correctly parse the string
      string(REPLACE ";" " " DC_COMPONENTS_STR "${DC_COMPONENTS}")
      set(dependency "${dependency} COMPONENTS ${DC_COMPONENTS_STR}")
    endif()

    if(DC_PUBLIC)
      set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_PublicDependencies "${dependency}")
    else()
      set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_PrivateDependencies "${dependency}")
    endif()
  endif()

endfunction()

