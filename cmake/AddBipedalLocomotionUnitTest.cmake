# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
#
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license. See the accompanying LICENSE file for details.

# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

# Fetch catch2 only if the testing are built
if (BUILD_TESTING)
  find_package(Catch2 3.0.1 QUIET)
  cmake_dependent_option(USE_SYSTEM_Catch2 "Use system Catch2" ON "Catch2_FOUND" OFF)
  if(NOT USE_SYSTEM_Catch2)
    include(FetchContent)
    FetchContent_Declare(Catch2
      GIT_REPOSITORY https://github.com/catchorg/Catch2.git
      GIT_TAG        v3.0.1)

    FetchContent_GetProperties(Catch2)
    if(NOT Catch2_POPULATED)
      message(STATUS "Fetching Catch2...")
      FetchContent_MakeAvailable(Catch2)
    endif()
  endif()
endif()



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

function(add_bipedal_test)

    if(BUILD_TESTING)

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

      target_link_libraries(${targetname} PRIVATE Catch2::Catch2WithMain ${${prefix}_LINKS})
      target_compile_definitions(${targetname} PRIVATE CATCH_CONFIG_FAST_COMPILE CATCH_CONFIG_DISABLE_MATCHERS CATCH_CONFIG_ENABLE_CHRONO_STRINGMAKER)
      target_compile_features(${targetname} PUBLIC cxx_std_17)
      target_compile_definitions(${targetname} PRIVATE -D_USE_MATH_DEFINES)

      add_test(NAME ${targetname} COMMAND ${targetname})

      if(FRAMEWORK_RUN_MemoryAllocationMonitor_tests)
        # When possible use `path_list_prepend` to permit to set other LD_PRELOAD
        if (CMAKE_MINIMUM_REQUIRED_VERSION VERSION_GREATER_EQUAL 3.22)
          message(AUTHOR_WARNING "cmake_minimum_required is now 3.22, cleanup the if following this message command.")
        endif()
        if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
          set_tests_properties(${targetname} PROPERTIES
            ENVIRONMENT_MODIFICATION "LD_PRELOAD=path_list_prepend:$<TARGET_FILE:BipedalLocomotion::MemoryAllocationMonitorPreload>")
        else()
          set_tests_properties(${targetname} PROPERTIES
            ENVIRONMENT "LD_PRELOAD=$<TARGET_FILE:BipedalLocomotion::MemoryAllocationMonitorPreload>")
        endif()
      endif()

      if(FRAMEWORK_RUN_Valgrind_tests)
        add_test(NAME memcheck_${targetname} COMMAND ${MEMCHECK_COMMAND_COMPLETE} $<TARGET_FILE:${targetname}>)

        if(FRAMEWORK_RUN_MemoryAllocationMonitor_tests)
          if (CMAKE_VERSION VERSION_GREATER_EQUAL 3.22)
            set_tests_properties(memcheck_${targetname} PROPERTIES
              ENVIRONMENT_MODIFICATION "LD_PRELOAD=path_list_prepend:$<TARGET_FILE:BipedalLocomotion::MemoryAllocationMonitorPreload>")
          else()
            set_tests_properties(memcheck_${targetname} PROPERTIES
              ENVIRONMENT "LD_PRELOAD=$<TARGET_FILE:BipedalLocomotion::MemoryAllocationMonitorPreload>")
          endif()
        endif()
      endif()

    endif()

endfunction()
