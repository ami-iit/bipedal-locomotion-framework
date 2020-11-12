# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

option(FRAMEWORK_USES_PYTHON "Do you want to create the Python bindings" FALSE)

if(FRAMEWORK_USES_PYTHON)

    # Find the necessary Python libraries
    find_package(Python3 COMPONENTS Interpreter Development REQUIRED)
    message(STATUS "Using Python: ${Python3_EXECUTABLE}")

    # Install the resulting Python package for the active interpreter
    if(NOT PYTHON_INSTDIR)
        set(PYTHON_INSTDIR ${Python3_SITELIB}/bipedal_locomotion)
    endif()

    # Require pybind11
    if(NOT FRAMEWORK_USE_pybind11)
        message(FATAL_ERROR "pybind11 not found")
    endif()

    # Folder of the Python package within the build tree.
    # It is used for the Python tests.
    set(BIPEDAL_LOCOMOTION_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/bipedal_locomotion")

    # Add the bindings directory
    add_subdirectory(python)

    # Create the __init__.py file
    file(GENERATE
         OUTPUT "${BIPEDAL_LOCOMOTION_PYTHON_PACKAGE}/__init__.py"
         CONTENT "from . import bindings")

     # Install the __init__.py file
    install(FILES "${BIPEDAL_LOCOMOTION_PYTHON_PACKAGE}/__init__.py"
            DESTINATION ${PYTHON_INSTDIR})

endif()
