# Copyright (C) 2021 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

import os
import sys

import cmake_build_extension
import setuptools

if "CIBUILDWHEEL" in os.environ and os.environ["CIBUILDWHEEL"] == "1":
    CIBW_CMAKE_OPTIONS = [
        # Prevent CMake to use lib64 on Debian Stretch
        "-DCMAKE_INSTALL_LIBDIR=lib",
        # Prevent CMake finding the osqp shipped with the CasADI wheel
        "-Dosqp_DIR=/usr/local/lib/cmake/osqp",
    ]
else:
    CIBW_CMAKE_OPTIONS = []


setuptools.setup(
    cmdclass=dict(build_ext=cmake_build_extension.BuildExtension),
    ext_modules=[
        cmake_build_extension.CMakeExtension(
            name="BlfCMakeProject",
            install_prefix="bipedal_locomotion_framework",
            cmake_depends_on=[
                "idyntree",
                "pybind11",
                "ycm_cmake_modules",
                "yarp",
                "manifpy",
                "casadi",
            ],
            disable_editable=True,
            cmake_configure_options=[
                f"-DPython3_EXECUTABLE:PATH={sys.executable}",
                "-DFRAMEWORK_PACKAGE_FOR_PYPI:BOOL=ON",
                "-DBUILD_SHARED_LIBS:BOOL=OFF",
                # *_USE_*
                "-DFRAMEWORK_USE_Catch2:BOOL=OFF",
                "-DFRAMEWORK_USE_LieGroupControllers:BOOL=OFF",
                "-DFRAMEWORK_USE_OpenCV:BOOL=OFF",
                "-DFRAMEWORK_USE_OsqpEigen:BOOL=ON",
                "-DFRAMEWORK_USE_PCL:BOOL=OFF",
                "-DFRAMEWORK_USE_Python3:BOOL=ON",
                "-DFRAMEWORK_USE_Qhull:BOOL=ON",
                "-DFRAMEWORK_USE_VALGRIND:BOOL=OFF",
                "-DFRAMEWORK_USE_YARP:BOOL=ON",
                "-DFRAMEWORK_USE_casadi:BOOL=ON",
                "-DFRAMEWORK_USE_cppad:BOOL=OFF",
                "-DFRAMEWORK_USE_manif:BOOL=ON",
                "-DFRAMEWORK_USE_matioCpp:BOOL=OFF",
                "-DFRAMEWORK_USE_nlohmann_json:BOOL=ON",
                "-DFRAMEWORK_USE_pybind11:BOOL=ON",
                "-DFRAMEWORK_USE_realsense2:BOOL=OFF",
                "-DFRAMEWORK_USE_tomlplusplus:BOOL=OFF",
                # *_COMPILE_*
                "-DFRAMEWORK_COMPILE_Contact:BOOL=ON",
                "-DFRAMEWORK_COMPILE_Math:BOOL=ON",
                "-DFRAMEWORK_COMPILE_PYTHON_BINDINGS:BOOL=ON",
                "-DFRAMEWORK_COMPILE_Planners:BOOL=ON",
                "-DFRAMEWORK_COMPILE_RobotInterface:BOOL=ON",
                "-DFRAMEWORK_COMPILE_System:BOOL=ON",
                "-DFRAMEWORK_COMPILE_YarpImplementation:BOOL=ON",
                "-DFRAMEWORK_COMPILE_YarpUtilities:BOOL=ON",
                "-DFRAMEWORK_COMPILE_JointPositionTrackingApplication:BOOL=OFF",
            ]
            + CIBW_CMAKE_OPTIONS,
        )
    ],
)
