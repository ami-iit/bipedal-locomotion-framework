# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.


# Check if a package is used and set some cmake variables
function(dependency_classifier package)
  set(options PUBLIC)
  set(singleValueArgs MINIMUM_VERSION)
  cmake_parse_arguments(DC "${options}" "${singleValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PREFIX "FRAMEWORK")

  # FRAMEWORK_USE_${package}
  if(${PREFIX}_USE_${package})
    if(DC_PUBLIC)
      if(DC_MINIMUM_VERSION)
        set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_PublicDependencies "${package} ${DC_MINIMUM_VERSION}")
      else()
        set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_PublicDependencies ${package})
      endif()
    else()
      if(DC_MINIMUM_VERSION)
        set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_PrivateDependencies "${package} ${DC_MINIMUM_VERSION}")
      else()
        set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_PrivateDependencies ${package})
      endif()
    endif()
  endif()

endfunction()
