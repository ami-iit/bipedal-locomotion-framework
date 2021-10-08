# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

if (FRAMEWORK_RUN_Valgrind_tests)
    set(CTEST_MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
    set(MEMORYCHECK_COMMAND ${VALGRIND_PROGRAM})
    if (APPLE)
        set(MEMORYCHECK_SUPPRESSIONS "--suppressions=${PROJECT_SOURCE_DIR}/cmake/valgrind-macos.supp")
    elseif (UNIX AND NOT APPLE)
        set(MEMORYCHECK_SUPPRESSIONS "--suppressions=${PROJECT_SOURCE_DIR}/cmake/valgrind-linux.supp")
    else ()
        set(MEMORYCHECK_SUPPRESSIONS "")
    endif ()
    set(MEMORYCHECK_COMMAND_OPTIONS "--leak-check=full --gen-suppressions=all --error-exitcode=1 ${MEMORYCHECK_SUPPRESSIONS}"  CACHE STRING "Options to pass to the memory checker")
    mark_as_advanced(MEMORYCHECK_COMMAND_OPTIONS)
    set(MEMCHECK_COMMAND_COMPLETE "${MEMORYCHECK_COMMAND} ${MEMORYCHECK_COMMAND_OPTIONS}")
    separate_arguments(MEMCHECK_COMMAND_COMPLETE)
endif()

if (FRAMEWORK_COMPILE_tests)
    configure_file(cmake/Catch2Main.cpp.in ${CMAKE_BINARY_DIR}/Testing/Catch2Main.cpp)
    add_library(CatchTestMain ${CMAKE_BINARY_DIR}/Testing/Catch2Main.cpp)
    target_link_libraries(CatchTestMain PUBLIC Catch2::Catch2)
endif()

if (FRAMEWORK_COMPILE_Rostests)
    configure_file(cmake/Catch2RosMain.cpp.in ${CMAKE_BINARY_DIR}/Testing/Catch2RosMain.cpp)
    configure_file(cmake/RosTest.sh.in ${CMAKE_BINARY_DIR}/Testing/RosTest.sh)
    add_library(RosCatchTestMain ${CMAKE_BINARY_DIR}/Testing/Catch2RosMain.cpp)
    target_link_libraries(RosCatchTestMain PUBLIC Catch2::Catch2 ${roscpp_LIBRARIES})
    include_directories(${roscpp_INCLUDE_DIRS})
  endif()


function(add_bipedal_test)

    if(FRAMEWORK_COMPILE_tests)

      set(options )
      set(oneValueArgs NAME)
      set(multiValueArgs SOURCES LINKS)

      set(prefix "bipedal")

      cmake_parse_arguments(${prefix}
          "${options}"
          "${oneValueArgs}"
          "${multiValueArgs}"
          ${ARGN})

      set(name ${${prefix}_NAME})
      set(unit_test_files ${${prefix}_SOURCES})

      set(targetname ${name}UnitTests)
      add_executable(${targetname}
          "${unit_test_files}")

      target_link_libraries(${targetname} PRIVATE CatchTestMain ${${prefix}_LINKS})
      target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS)
      target_compile_features(${targetname} PUBLIC cxx_std_17)
      target_compile_definitions(${targetname} PRIVATE -D_USE_MATH_DEFINES)

      add_test(NAME ${targetname} COMMAND ${targetname})

      if(FRAMEWORK_RUN_Valgrind_tests)
        add_test(NAME memcheck_${targetname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${targetname}>)
      endif()

    endif()

endfunction()
