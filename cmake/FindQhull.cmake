# SPDX-FileCopyrightText: 2023 Istituto Italiano di Tecnologia (IIT)
# SPDX-License-Identifier: BSD-3-Clause

#[=======================================================================[.rst:
FindQhull
-----------

This is just a simple compatibility script that ensures that Qhull
provided by apt on Ubuntu 20.04 works fine with blf. It is not meant
to work fine in general (it only defines the imported targets required by blf).
Please remove as soon as Ubuntu 20.04 apt compatibiliy is dropped.

#]=======================================================================]

include(FindPackageHandleStandardArgs)

# First of all, try if Qhull installs a config file
find_package(Qhull CONFIG)

if(Qhull_FOUND)
  find_package_handle_standard_args(Qhull CONFIG_MODE)
else()
  find_path(qhull_INCLUDE_DIR libqhullcpp/Qhull.h)
  mark_as_advanced(qhull_INCLUDE_DIR)
  find_library(qhull_r_LIBRARY qhull_r)
  mark_as_advanced(qhull_r_LIBRARY)
  # This may fail, but that is ok as C++ symbols of qhull
  # for some older distributions are contained in qhull_r
  find_library(qhullcpp_LIBRARY qhullcpp)
  mark_as_advanced(qhullcpp_LIBRARY)

  find_package_handle_standard_args(Qhull DEFAULT_MSG qhull_INCLUDE_DIR qhull_r_LIBRARY)

  if(Qhull_FOUND)
    if(NOT TARGET Qhull::qhull_r)
      add_library(Qhull::qhull_r UNKNOWN IMPORTED)
      set_target_properties(Qhull::qhull_r PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${qhull_INCLUDE_DIR}")
      set_property(TARGET Qhull::qhull_r PROPERTY
        IMPORTED_LOCATION "${qhull_r_LIBRARY}")
    endif()
    if(NOT TARGET Qhull::qhullcpp)
      add_library(Qhull::qhullcpp UNKNOWN IMPORTED)
      set_target_properties(Qhull::qhullcpp PROPERTIES
        INTERFACE_INCLUDE_DIRECTORIES "${qhull_INCLUDE_DIR}")
      if(qhullcpp_LIBRARY)
        set_property(TARGET Qhull::qhullcpp PROPERTY
          IMPORTED_LOCATION "${qhullcpp_LIBRARY}")
      endif()
    endif()
  endif()
endif()
