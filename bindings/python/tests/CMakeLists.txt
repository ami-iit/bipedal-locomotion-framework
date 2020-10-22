# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

add_test (NAME pybind11_BipdealLocomotion_test
  COMMAND ${Python3_EXECUTABLE} -m unittest discover -s ${CMAKE_CURRENT_SOURCE_DIR}
  WORKING_DIRECTORY ${PROJECT_BINARY_DIR}/bindings/python
)
