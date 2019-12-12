# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

cmake_minimum_required(VERSION 3.5)
set(CMAKE_CXX_STANDARD 14)

## MAIN project
project(BipedalLocomotionControllers
  VERSION 0.0.1)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

# Check BipedalLocomotionControllers dependencies, find necessary libraries.
include(BipedalLocomotionControllersFindDependencies)

#set default build type to "Release" in single-config generators
if(NOT CMAKE_CONFIGURATION_TYPES)
    if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release" CACHE STRING
        "Choose the type of build, recommanded options are: Debug or Release" FORCE)
    endif()
    set(ROBOTOLOGY_BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${ROBOTOLOGY_BUILD_TYPES})
endif()

# Defines the CMAKE_INSTALL_LIBDIR, CMAKE_INSTALL_BINDIR and many other useful macros.
# See https://cmake.org/cmake/help/latest/module/GNUInstallDirs.html
include(GNUInstallDirs)

option(ENABLE_RPATH "Enable RPATH for this library" ON)
mark_as_advanced(ENABLE_RPATH)
include(AddInstallRPATHSupport)
add_install_rpath_support(BIN_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_BINDIR}"
  LIB_DIRS "${CMAKE_INSTALL_PREFIX}/${CMAKE_INSTALL_LIBDIR}"
  INSTALL_NAME_DIR "${CMAKE_INSTALL_PREFIX}"
  DEPENDS ENABLE_RPATH
  USE_LINK_PATH)

add_subdirectory(src)

include(InstallBasicPackageFiles)
install_basic_package_files(${PROJECT_NAME}
                            NAMESPACE BipedalLocomotionControllers::
                            VERSION ${BipedalLocomotionControllers_VERSION}
                            COMPATIBILITY AnyNewerVersion
                            TARGETS_PROPERTY BipedalLocomotionControllers_TARGETS
                            VARS_PREFIX ${PROJECT_NAME}
                            NO_CHECK_REQUIRED_COMPONENTS_MACRO)


include(AddUninstallTarget)
