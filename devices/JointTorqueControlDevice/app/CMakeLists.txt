# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

# Get list of models
subdirlist(subdirs ${CMAKE_CURRENT_SOURCE_DIR}/robots/)
# Install each model
foreach (dir ${subdirs})
  yarp_install(DIRECTORY robots/${dir} DESTINATION ${YARP_ROBOTS_INSTALL_DIR})
endforeach ()
