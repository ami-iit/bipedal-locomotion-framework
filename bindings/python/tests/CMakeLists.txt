# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Run pytest from the build folder. CMake creates a build/bipedal_locomotion
# folder with the build-tree Python package.
add_test(NAME PythonBindingsUnitTest
  COMMAND ${Python3_EXECUTABLE} -m pytest ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}
)
