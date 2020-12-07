# Find pytest.
#
# This module defines:
#
#    pytest_EXECUTABLE: the pytest executable.
#    pytest_FOUND: if false, do not try to use pytest.
#
# If you have valgrind installed in a non-standard place, you can define
# pytest_ROOT to tell cmake where it is.

if(pytest_FOUND)
    return()
endif()

if(pytest_ROOT)
    set(EXTRA_PYTEST_PATH "${pytest_ROOT}")
else()
    set(EXTRA_PYTEST_PATH)
endif()

# Look for a command called pytest
find_program(pytest_EXECUTABLE
             NAMES pytest
             PATH ${EXTRA_PYTEST_PATH}
             DOC "Path to pytest executable")
string(STRIP "${pytest_EXECUTABLE}" pytest_EXECUTABLE)

include(FindPackageHandleStandardArgs)

# Handle standard arguments to find_package like REQUIRED and QUIET
find_package_handle_standard_args(pytest
                                  DEFAULT_MSG
                                  pytest_EXECUTABLE)
