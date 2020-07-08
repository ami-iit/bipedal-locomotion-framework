# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# List the subdirectory
# http://stackoverflow.com/questions/7787823/cmake-how-to-get-the-name-of-all-subdirectories-of-a-directory
macro(SUBDIRLIST result curdir)
  file(GLOB children RELATIVE ${curdir} ${curdir}/*)
  set(dirlist "")
  foreach(child ${children})
    if(IS_DIRECTORY ${curdir}/${child})
      list(APPEND dirlist ${child})
    endif()
  endforeach()
  set(${result} ${dirlist})
endmacro()

option(FRAMEWORK_INSTALL_ALL_ROBOTS_INI "Enable installation of THE ini files for all robots" ON)
set(ROBOT_NAME "$ENV{YARP_ROBOT_NAME}" CACHE STRING "Name of your robot")

if (FRAMEWORK_USE_YARP)
    yarp_configure_external_installation(BipedalLocomotionFramework)
endif()

macro(install_ini_files parent_dir)
  if (FRAMEWORK_USE_YARP)
    # Get list of models
    if(FRAMEWORK_INSTALL_ALL_ROBOTS_INI)
      subdirlist(robots ${parent_dir}/robots/)

      # Install each model
      foreach (robot ${robots})
        file(GLOB scripts ${parent_dir}/robots/${robot}/*.ini)
        yarp_install(FILES ${scripts} DESTINATION ${BIPEDALLOCOMOTIONFRAMEWORK_ROBOTS_INSTALL_DIR}/${robot})

        subdirlist(subdirs ${parent_dir}/robots/${robot}/)
        foreach (subdir ${subdirs})
          yarp_install(DIRECTORY ${parent_dir}/robots/${robot}/${subdir} DESTINATION ${BIPEDALLOCOMOTIONFRAMEWORK_ROBOTS_INSTALL_DIR}/${robot})
        endforeach ()
      endforeach ()
    else()
      if(ROBOT_NAME)
        if(IS_DIRECTORY "${parent_dir}/robots/${ROBOT_NAME}")

          file(GLOB scripts ${parent_dir}/robots/${ROBOT_NAME}/*.ini)
          yarp_install(FILES ${scripts} DESTINATION ${BIPEDALLOCOMOTIONFRAMEWORK_ROBOTS_INSTALL_DIR}/${ROBOT_NAME})

          subdirlist(subdirs ${parent_dir}/robots/${ROBOT_NAME}/)
          foreach (subdir ${subdirs})
            yarp_install(DIRECTORY robots/${ROBOT_NAME}/${subdir} DESTINATION ${BIPEDALLOCOMOTIONFRAMEWORK_ROBOTS_INSTALL_DIR}/${ROBOT_NAME})
          endforeach ()
        endif()
      endif()
    endif()
  endif()
endmacro()

