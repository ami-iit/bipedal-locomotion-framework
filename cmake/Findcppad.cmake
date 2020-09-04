#.rst:
# Findcppad
# ---------
#
# Try to locate the cppad library
#
# On non Windows systems, use pkg-config to try to locate the library,
# if this fails then try to locate the library in the directory pointed
# by the cppad_DIR environment variable.
#
# On Windows systems, it is not supported yet
#
# Create the following variables::
#
#  cppad_INCLUDE_DIRS - Directories to include to use cppad
#  cppad_LIBRARIES    - Default library to link against to use cppad
#  cppad_DEFINITIONS  - Flags to be added to linker's options
#  cppad_LINK_FLAGS   - Flags to be added to linker's options
#  cppad_FOUND        - If false, don't try to use cppad

#=============================================================================

if(NOT WIN32)
  # On non Windows systems we use PkgConfig to find cppad
  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)

    if(cppad_FIND_VERSION)
      if(cppad_FIND_VERSION_EXACT)
        pkg_check_modules(_PC_cppad QUIET cppad=${cppad_FIND_VERSION})
      else()
        pkg_check_modules(_PC_cppad QUIET cppad>=${cppad_FIND_VERSION})
      endif()
    else()
      pkg_check_modules(_PC_cppad QUIET cppad)
    endif()

    if(_PC_cppad_FOUND)
      set(cppad_INCLUDE_DIRS ${_PC_cppad_INCLUDE_DIRS} CACHE PATH "cppad include directory")
      set(cppad_DEFINITIONS ${_PC_cppad_CFLAGS} CACHE STRING "Additional compiler flags for cppad")

      find_library(${_PC_cppad_LIBRARIES}_PATH
        NAMES ${_PC_cppad_LIBRARIES}
        PATHS ${_PC_cppad_LIBRARY_DIRS})

      set(cppad_LIBRARIES ${${_PC_cppad_LIBRARIES}_PATH} CACHE PATH "cppad libraries" FORCE)

    else()
      set(cppad_DEFINITIONS "")
    endif()

  endif()

  set(cppad_LINK_FLAGS "")

  # If pkg-config fails, try to find the package using cppad_DIR
  if(NOT _PC_cppad_FOUND)
    set(cppad_DIR_TEST $ENV{cppad_DIR})
    if(cppad_DIR_TEST)
      set(cppad_DIR $ENV{cppad_DIR} CACHE PATH "Path to cppad build directory")
    else()
      set(cppad_DIR /usr            CACHE PATH "Path to cppad build directory")
    endif()

    find_path(cppad_INCLUDE_DIRS
      NAMES cppad.hpp PATH_SUFFIXES cppad PATHS ${cppad_DIR}/include/cppad)

    find_library(cppad_LIBRARIES cppad ${cppad_DIR}/lib ${cppad_DIR}/lib/cppad)

    set(cppad_DEFINITIONS "")
    set(cppad_LINK_FLAGS "")
  endif()

# Windows platforms
else()

  find_package(PkgConfig QUIET)
  if(PKG_CONFIG_FOUND)

    if(cppad_FIND_VERSION)
      if(cppad_FIND_VERSION_EXACT)
        pkg_check_modules(_PC_cppad QUIET cppad=${cppad_FIND_VERSION})
      else()
        pkg_check_modules(_PC_cppad QUIET cppad>=${cppad_FIND_VERSION})
      endif()
    else()
      pkg_check_modules(_PC_cppad QUIET cppad)
    endif()

    if(_PC_cppad_FOUND)
      set(cppad_INCLUDE_DIRS ${_PC_cppad_INCLUDE_DIRS} CACHE PATH "cppad include directory")
      set(cppad_DEFINITIONS ${_PC_cppad_CFLAGS} CACHE STRING "Additional compiler flags for cppad")

      find_library(${_PC_cppad_LIBRARIES}_PATH
        NAMES ${_PC_cppad_LIBRARIES}
        PATHS ${_PC_cppad_LIBRARY_DIRS})

      set(cppad_LIBRARIES ${${_PC_cppad_LIBRARIES}_PATH} CACHE PATH "cppad libraries" FORCE)

    else()
      set(cppad_DEFINITIONS "")
    endif()

  endif()

  set(cppad_LINK_FLAGS "")

  # If pkg-config fails, try to find the package using cppad_DIR
  if(NOT _PC_cppad_FOUND)
    set(cppad_DIR $ENV{cppad_DIR} CACHE PATH "Path to cppad build directory")

    find_path(cppad_INCLUDE_DIRS
      NAMES cppad.hpp PATH_SUFFIXES cppad PATHS ${cppad_DIR}/include/cppad)

    find_library(cppad_LIBRARIES
      NAMES cppad cppad_lib
      PATHS ${cppad_DIR}/lib ${cppad_DIR}/lib/cppad)

    set(cppad_DEFINITIONS "")
    set(cppad_LINK_FLAGS "")
  endif()

endif()


mark_as_advanced(cppad_INCLUDE_DIRS
                 cppad_LIBRARIES
                 cppad_DEFINITIONS
                 cppad_LINK_FLAGS)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(cppad DEFAULT_MSG cppad_LIBRARIES)

if(NOT cppad_FOUND)
  return()
endif()

if(NOT TARGET cppad)
  add_library(cppad UNKNOWN IMPORTED)
  set_target_properties(cppad PROPERTIES IMPORTED_LOCATION ${cppad_LIBRARIES})
  set_target_properties(cppad PROPERTIES INTERFACE_INCLUDE_DIRECTORIES "${cppad_INCLUDE_DIRS}")
endif()
