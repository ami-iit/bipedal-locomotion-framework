# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

# This module checks if all the dependencies are installed and if the
# dependencies to build some parts are satisfied.
# For every dependency, it creates the following variables:
#
# FRAMEWORK_USE_${Package}: Can be disabled by the user if he doesn't want to use that
#                      dependency.
# FRAMEWORK_HAS_${Package}: Internal flag. It should be used to check if a part of
#                      FRAMEWORK should be built. It is on if FRAMEWORK_USE_${Package} is
#                      on and either the package was found or will be built.
# FRAMEWORK_BUILD_${Package}: Internal flag. Used to check if FRAMEWORK has to build an
#                        external package.
# FRAMEWORK_BUILD_DEPS_${Package}: Internal flag. Used to check if dependencies
#                             required to build the package are available.
# FRAMEWORK_HAS_SYSTEM_${Package}: Internal flag. Used to check if the package is
#                             available on the system.
# FRAMEWORK_USE_SYSTEM_${Package}: This flag is shown only for packages in the
#                             extern folder that were also found on the system
#                             (TRUE by default). If this flag is enabled, the
#                             system installed library will be used instead of
#                             the version shipped within the framework.


include(CMakeDependentOption)

# Check if a package is installed and set some cmake variables
macro(checkandset_dependency package)

  set(singleValueArgs MINIMUM_VERSION)
  set(multiValueArgs COMPONENTS)
  cmake_parse_arguments(CSD "${options}" "${singleValueArgs}" "${multiValueArgs}" ${ARGN})

  set(PREFIX "FRAMEWORK")

  string(TOUPPER ${package} PKG)

  # FRAMEWORK_HAS_SYSTEM_${package}
  if(${package}_FOUND OR ${PKG}_FOUND)
    set(${PREFIX}_HAS_SYSTEM_${package} TRUE)
  else()
    set(${PREFIX}_HAS_SYSTEM_${package} FALSE)
  endif()

  # FRAMEWORK_USE_${package}
  option(${PREFIX}_USE_${package} "Use package ${package}" ${${PREFIX}_HAS_SYSTEM_${package}})
  if (${PREFIX}_HAS_SYSTEM_${package})
      mark_as_advanced(${PREFIX}_USE_${package})
  elseif (${PREFIX}_USE_${package})
    if (CSD_COMPONENTS)
      find_package(${package} ${CSD_MINIMUM_VERSION} COMPONENTS ${CSD_COMPONENTS} REQUIRED)
    else ()
      find_package(${package} ${CSD_MINIMUM_VERSION} REQUIRED)
    endif ()
  endif()

  # FRAMEWORK_USE_SYSTEM_${package}
  set(${PREFIX}_USE_SYSTEM_${package} ${${PREFIX}_USE_${package}} CACHE INTERNAL "Use system-installed ${package}, rather than a private copy (recommended)" FORCE)
  if(NOT "${package}" STREQUAL "${PKG}")
    unset(${PREFIX}_USE_SYSTEM_${PKG} CACHE)
  endif()

  # FRAMEWORK_HAS_${package}
  if(${${PREFIX}_HAS_SYSTEM_${package}})
    set(${PREFIX}_HAS_${package} ${${PREFIX}_USE_${package}})
  else()
    set(${PREFIX}_HAS_${package} FALSE)
  endif()

endmacro()

macro(FRAMEWORK_DEPENDENT_OPTION _option _doc _default _deps _force)

  if(DEFINED ${_option})
    get_property(_option_strings_set CACHE ${_option} PROPERTY STRINGS SET)
    if(_option_strings_set)
      # If the user thinks he is smarter than the machine, he deserves an error
      get_property(_option_strings CACHE ${_option} PROPERTY STRINGS)
      list(GET _option_strings 0 _option_strings_first)
      string(REGEX REPLACE ".+\"(.+)\".+" "\\1" _option_strings_first "${_option_strings_first}")
      list(LENGTH _option_strings _option_strings_length)
      math(EXPR _option_strings_last_index "${_option_strings_length} - 1")
      list(GET _option_strings ${_option_strings_last_index} _option_strings_last)
      if("${${_option}}" STREQUAL "${_option_strings_last}")
        message(SEND_ERROR "That was a trick, you cannot outsmart me! I will never let you win! ${_option} stays OFF until I say so! \"${_option_strings_first}\" is needed to enable ${_option}. Now stop bothering me, and install your dependencies, if you really want to enable this option.")
      endif()
      unset(${_option} CACHE)
    endif()
  endif()

  cmake_dependent_option(${_option} "${_doc}" ${_default} "${_deps}" ${_force})

  unset(_missing_deps)
  foreach(_dep ${_deps})
    string(REGEX REPLACE " +" ";" _depx "${_dep}")
    if(NOT (${_depx}))
      list(APPEND _missing_deps "${_dep}")
    endif()
  endforeach()

  if(DEFINED _missing_deps)
    set(${_option}_disable_reason " (dependencies unsatisfied: \"${_missing_deps}\")")
    # Set a value that can be visualized on ccmake and on cmake-gui, but
    # still evaluates to false
    set(${_option} "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND" CACHE STRING "${_option_doc}" FORCE)
    string(REPLACE ";" "\;" _missing_deps "${_missing_deps}")
    set_property(CACHE ${_option}
                PROPERTY STRINGS "OFF - Dependencies unsatisfied: '${_missing_deps}' - ${_option}-NOTFOUND"
                                 "OFF - You can try as much as you want, but '${_missing_deps}' is needed to enable ${_option} - ${_option}-NOTFOUND"
                                 "OFF - Are you crazy or what? '${_missing_deps}' is needed to enable ${_option} - ${_option}-NOTFOUND"
                                 "OFF - Didn't I already tell you that '${_missing_deps}' is needed to enable ${_option}? - ${_option}-NOTFOUND"
                                 "OFF - Stop it! - ${_option}-NOTFOUND"
                                 "OFF - This is insane! Leave me alone! - ${_option}-NOTFOUND"
                                 "ON - All right, you win. The option is enabled. Are you happy now? You just broke the build.")
    # Set non-cache variable that will override the value in current scope
    # For parent scopes, the "-NOTFOUND ensures that the variable still
    # evaluates to false
    set(${_option} ${_force})
  endif()

endmacro()



################################################################################
# Find all packages

find_package(iDynTree 0.11.105 REQUIRED) #Right now, all the packages built in the framework
                                         #depend directly or indirectly from iDynTree and Eigen
                                         #(which is an iDynTree dependency by the way)
find_package(Eigen3 3.2.92 REQUIRED)

find_package(YARP QUIET)
checkandset_dependency(YARP)

find_package(Qhull 8.0.0 QUIET)
checkandset_dependency(Qhull)

find_package(casadi QUIET)
checkandset_dependency(casadi)

find_package(cppad QUIET)
checkandset_dependency(cppad)

find_package(manif QUIET)
checkandset_dependency(manif)

find_package(Python3 3.6 COMPONENTS Interpreter Development QUIET)
checkandset_dependency(Python3 MINIMUM_VERSION 3.6 COMPONENTS Interpreter Development)

find_package(pybind11 2.2 CONFIG QUIET)
checkandset_dependency(pybind11)

find_package(pytest QUIET)
checkandset_dependency(pytest)

find_package(matioCpp QUIET)
checkandset_dependency(matioCpp)

framework_dependent_option(FRAMEWORK_COMPILE_YarpUtilities
  "Compile YarpHelper library?" ON
  "FRAMEWORK_USE_YARP" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_YarpImplementation
  "Compile All the YARP implementations?" ON
  "FRAMEWORK_COMPILE_YarpUtilities" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Estimators
  "Compile Estimators library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Contact
  "Compile Contact libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_Planners
  "Compile Planners libraries?" ON
  "FRAMEWORK_USE_Qhull;FRAMEWORK_USE_casadi;FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ContactModels
  "Compile ContactModels library?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_System
  "Compile System library?" ON
  "FRAMEWORK_COMPILE_ContactModels" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_AutoDiffCppAD
  "Compile CppAD-Eigen wrapper?" ON
  "FRAMEWORK_USE_cppad" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_RobotInterface
  "Compile RobotInterface libraries?" ON
  "" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_FloatingBaseEstimators
  "Compile FloatingBaseEstimators libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_ManifConversions
  "Compile manif Conversions libraries?" ON
  "FRAMEWORK_USE_manif" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_matioCppConversions
  "Compile matioCpp Conversions libraries?" ON
  "FRAMEWORK_USE_matioCpp" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_JointPositionTrackingApplication
  "Compile joint-position-tracking application?" ON
  "FRAMEWORK_COMPILE_YarpImplementation;FRAMEWORK_COMPILE_Planners;FRAMEWORK_COMPILE_RobotInterface" OFF)

framework_dependent_option(FRAMEWORK_COMPILE_PYTHON_BINDINGS
  "Do you want to generate and compile the Python bindings?" ON
  "FRAMEWORK_USE_Python3;FRAMEWORK_USE_pybind11" OFF)

framework_dependent_option(FRAMEWORK_TEST_PYTHON_BINDINGS
  "Do you want to test the Python bindings?" ON
  "FRAMEWORK_COMPILE_tests;FRAMEWORK_COMPILE_PYTHON_BINDINGS;FRAMEWORK_USE_pytest" OFF)
