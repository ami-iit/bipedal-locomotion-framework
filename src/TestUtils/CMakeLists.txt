set(H_PREFIX include/BipedalLocomotion/TestUtils)

if(FRAMEWORK_RUN_MemoryAllocationMonitor_tests)
    set(TestUtils_SOURCES MemoryAllocationMonitor.cpp)
else()
    set(TestUtils_SOURCES MemoryAllocationMonitorDummy.cpp)
endif()

add_bipedal_locomotion_library(
    NAME                   TestUtils
    PUBLIC_HEADERS         ${H_PREFIX}/MemoryAllocationMonitor.h
    SOURCES                ${TestUtils_SOURCES}
    SUBDIRECTORIES         tests
    SKIP_INSTALL)

if(FRAMEWORK_RUN_MemoryAllocationMonitor_tests)
    # Small library that just contain malloc and 
    # other function reimplementaton, that s just passed via LD_PRELOAD
    # The library type is hardcoded to be SHARED as otherwise it is
    # not possible to LD_PRELOAD the library
    add_library(MemoryAllocationMonitorPreload
                SHARED MemoryAllocationMonitorRedefinedMemoryFunctions.c)
    add_library(BipedalLocomotion::MemoryAllocationMonitorPreload 
                ALIAS MemoryAllocationMonitorPreload)
    set_target_properties(MemoryAllocationMonitorPreload PROPERTIES
              OUTPUT_NAME "${PROJECT_NAME}MemoryAllocationMonitorPreload")

    # Link BipedaLocomotion::TestUtils as it contains the definitions of the 
    # g_blf_mam_** global variables
    target_link_libraries(MemoryAllocationMonitorPreload PRIVATE BipedalLocomotion::TestUtils ${CMAKE_DL_LIBS})
endif()
