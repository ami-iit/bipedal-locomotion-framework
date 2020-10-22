# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

option(FRAMEWORK_USES_PYTHON "Do you want to create the Python bindings" FALSE)

if(FRAMEWORK_USES_PYTHON)
    find_package(Python3 COMPONENTS Interpreter QUIET)
    if (Python3_FOUND)
        execute_process(COMMAND ${Python3_EXECUTABLE}
            -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
            OUTPUT_VARIABLE _PYTHON_INSTDIR)
        string(STRIP ${_PYTHON_INSTDIR} FRAMEWORK_PYTHON_INSTALL_DIR)
        # # Bindings are installed in `BipedalLocomotion` subdirectory.
        # list(APPEND FRAMEWORK_BINARY_DIRS "${FRAMEWORK_PYTHON_INSTALL_DIR}/BipedalLocomotion")
    endif()
endif()

# Setup installation directory for Python bindings.
if (FRAMEWORK_USES_PYTHON)
  set(PYTHON_INSTDIR ${FRAMEWORK_PYTHON_INSTALL_DIR}/BipedalLocomotion)
endif()

find_package(pybind11 QUIET)
if (FRAMEWORK_USES_PYTHON)
  if (${pybind11_FOUND})
        add_subdirectory(python)
    else()
       MESSAGE(FATAL_ERROR "pybind11 not found, impossible to generate Bipedal-locomotion-framework bindings.")
    endif()
endif()

# Install main __init__.py file.
if (FRAMEWORK_USES_PYTHON)
    if (WIN32)
        set(NEW_LINE "\n\r")
    else()
        set(NEW_LINE "\n")
    endif()
    # Clear the file first if it exists.
    file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "")
    # If pybind is enabled, add the corresponding import.
    if (${FRAMEWORK_USES_PYTHON})
        file(APPEND ${CMAKE_CURRENT_BINARY_DIR}/__init__.py "from . import pybind${NEW_LINE}")
    endif()

    install(FILES "${CMAKE_CURRENT_BINARY_DIR}/__init__.py"
             DESTINATION ${PYTHON_INSTDIR})
endif()
