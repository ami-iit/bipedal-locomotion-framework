# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

if(FRAMEWORK_COMPILE_Math)

  set(H_PREFIX include/BipedalLocomotion/Math)

  add_bipedal_locomotion_library(
    NAME                  Math
    PUBLIC_HEADERS        ${H_PREFIX}/CARE.h ${H_PREFIX}/Constants.h
                          ${H_PREFIX}/LinearizedFrictionCone.h ${H_PREFIX}/ContactWrenchCone.h
                          ${H_PREFIX}/Wrench.h ${H_PREFIX}/SchmittTrigger.h ${H_PREFIX}/QuadraticBezierCurve.h
                          ${H_PREFIX}/Spline.h ${H_PREFIX}/ZeroOrderSpline.h  ${H_PREFIX}/LinearSpline.h ${H_PREFIX}/CubicSpline.h ${H_PREFIX}/QuinticSpline.h
    SOURCES               src/CARE.cpp  src/LinearizedFrictionCone.cpp src/ContactWrenchCone.cpp
                          src/SchmittTrigger.cpp src/QuadraticBezierCurve.cpp
    PUBLIC_LINK_LIBRARIES Eigen3::Eigen BipedalLocomotion::ParametersHandler
                          BipedalLocomotion::TextLogging BipedalLocomotion::System MANIF::manif
    SUBDIRECTORIES        tests)

endif()
