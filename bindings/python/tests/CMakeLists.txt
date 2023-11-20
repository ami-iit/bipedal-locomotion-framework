# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# Run pytest from the build folder. CMake creates a build/bipedal_locomotion_framework
# folder with the build-tree Python package. We use it as working directory so that
# the tests always load the Python package from the build-tree even if blf is installed
# elsewhere.

get_property(bindings_tests GLOBAL PROPERTY BipedalLocomotionFrameworkBindings_TESTS)

add_test(NAME PythonBindingsUnitTest
    COMMAND ${Python3_EXECUTABLE} -m pytest ${bindings_tests}
    WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
)
