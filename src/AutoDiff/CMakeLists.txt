# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_AutoDiffCppAD)

  add_bipedal_locomotion_library(
    NAME                   AutoDiffCppAD
    PUBLIC_HEADERS         include/BipedalLocomotion/AutoDiff/CppAD.h
    PUBLIC_LINK_LIBRARIES  Eigen3::Eigen cppad
    IS_INTERFACE
    SUBDIRECTORIES         tests)

endif()
