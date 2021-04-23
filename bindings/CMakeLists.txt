# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

if(FRAMEWORK_COMPILE_PYTHON_BINDINGS)

    option(FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES
        "Do you want BLF to detect and use the active site-package directory? (it could be a system dir)"
        FALSE)

    # Install the resulting Python package for the active interpreter
    if(FRAMEWORK_DETECT_ACTIVE_PYTHON_SITEPACKAGES)
        set(PYTHON_INSTDIR ${Python3_SITELIB}/bipedal_locomotion_framework)
    elseif(FRAMEWORK_PACKAGE_FOR_PYPI)
        set(PYTHON_INSTDIR ${CMAKE_INSTALL_PREFIX})
    else()
     execute_process(COMMAND ${Python3_EXECUTABLE} -c "from distutils import sysconfig; print(sysconfig.get_python_lib(1,0,prefix=''))"
                     OUTPUT_VARIABLE _PYTHON_INSTDIR)
     string(STRIP ${_PYTHON_INSTDIR} _PYTHON_INSTDIR_CLEAN)
     set(PYTHON_INSTDIR ${_PYTHON_INSTDIR_CLEAN}/bipedal_locomotion_framework)
    endif()

    # Folder of the Python package within the build tree.
    # It is used for the Python tests.
    set(BLF_PYTHON_PACKAGE "${CMAKE_BINARY_DIR}/bipedal_locomotion_framework")

    # Add the bindings directory
    add_subdirectory(python)

    # Create the __init__.py file
    file(GENERATE
         OUTPUT "${BLF_PYTHON_PACKAGE}/__init__.py"
         CONTENT "from . import bindings")

     # Install the __init__.py file
    install(FILES "${BLF_PYTHON_PACKAGE}/__init__.py"
            DESTINATION ${PYTHON_INSTDIR})

endif()
