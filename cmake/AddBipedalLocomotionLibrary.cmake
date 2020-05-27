# Copyright (C) 2019 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.

function(add_bipedal_locomotion_library)

  set(options IS_INTERFACE DEPENDS_ON_EIGEN_PRIVATE)
  set(oneValueArgs NAME INSTALLATION_FOLDER)
  set(multiValueArgs
    SOURCES
    PUBLIC_HEADERS
    PRIVATE_HEADERS
    PUBLIC_LINK_LIBRARIES
    PRIVATE_LINK_LIBRARIES
    SUBDIRECTORIES)

  set(prefix "bipedal_component")

  cmake_parse_arguments(${prefix}
    "${options}"
    "${oneValueArgs}"
    "${multiValueArgs}"
    ${ARGN})

  set(name ${${prefix}_NAME})
  set(installation_folder ${${prefix}_INSTALLATION_FOLDER})
  set(is_interface ${${prefix}_IS_INTERFACE})
  set(depends_on_eigen_private ${${prefix}_DEPENDS_ON_EIGEN_PRIVATE})
  set(sources ${${prefix}_SOURCES})
  set(public_headers ${${prefix}_PUBLIC_HEADERS})
  set(private_headers ${${prefix}_PRIVATE_HEADERS})
  set(public_link_libraries ${${prefix}_PUBLIC_LINK_LIBRARIES})
  set(private_link_libraries ${${prefix}_PRIVATE_LINK_LIBRARIES})
  set(subdirectories ${${prefix}_SUBDIRECTORIES})

  if(NOT installation_folder)
    set(installation_folder ${name})
  endif()

  # The building process is different if the component is an INTERFACE
  if(is_interface)

    add_library(${name} INTERFACE)

    # Add C++17 features
    target_compile_features(${name} INTERFACE cxx_std_17)

    target_link_libraries(${name} INTERFACE ${public_link_libraries})

    # Specify include directories for both compilation and installation process.
    # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
    # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html
    # creating-relocatable-packages
    target_include_directories(${name} INTERFACE "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

    # Specify installation targets, typology and destination folders.
    install(TARGETS        ${name}
      EXPORT               ${PROJECT_NAME}
      COMPONENT            runtime)

    install(FILES ${public_headers} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotion/${installation_folder}")
    install(FILES ${private_headers} DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotion/${installation_folder}/impl")

  else()

    # add an executable to the project using the specified source files.
    add_library(${name} ${sources} ${public_headers} ${private_headers})

    # Add C++17 features
    target_compile_features(${name} PUBLIC cxx_std_17)

    target_link_libraries(${name} PUBLIC ${public_link_libraries})
    target_link_libraries(${name} PRIVATE ${private_link_libraries})

    set_target_properties(${name} PROPERTIES
      OUTPUT_NAME "${PROJECT_NAME}${name}"
      VERSION ${BipedalLocomotionFramework_VERSION}
      PUBLIC_HEADER "${public_headers}"
      PRIVATE_HEADER "${private_headers}")

    # Specify include directories for both compilation and installation process.
    # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
    # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html
    #creating-relocatable-packages
    target_include_directories(${name} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

    # Specify installation targets, typology and destination folders.
    install(TARGETS    ${name}
      EXPORT           ${PROJECT_NAME}
      COMPONENT        runtime
      LIBRARY          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                                    COMPONENT shlib
      ARCHIVE          DESTINATION "${CMAKE_INSTALL_LIBDIR}"                                                    COMPONENT lib
      RUNTIME          DESTINATION "${CMAKE_INSTALL_BINDIR}"                                                    COMPONENT bin
      PUBLIC_HEADER    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotion/${installation_folder}"       COMPONENT dev
      PRIVATE_HEADER   DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/BipedalLocomotion/${installation_folder}/impl"  COMPONENT dev)

  endif()

  get_property(umbrella_includes_list GLOBAL PROPERTY umbrella_includes)
  foreach(header ${public_headers})
    get_filename_component(extension ${header} LAST_EXT)
    if ((extension STREQUAL ".h") OR (extension STREQUAL ".hpp"))
      get_filename_component(header_name ${header} NAME)
      set(include_command "#include <BipedalLocomotion/${installation_folder}/${header_name}>")
      set(umbrella_includes_list "${umbrella_includes_list}\n${include_command}")
    endif()
  endforeach()
  set_property(GLOBAL PROPERTY umbrella_includes "${umbrella_includes_list}")

  # add alias
  add_library(BipedalLocomotion::${name} ALIAS ${name})

  if (depends_on_eigen_private)
    target_include_directories(${name} PRIVATE SYSTEM ${EIGEN3_INCLUDE_DIR})
  endif()

  # Add all subdirectories
  foreach(subdir ${subdirectories})
    add_subdirectory(${subdir})
  endforeach()

  set_property(GLOBAL APPEND PROPERTY BipedalLocomotionFramework_TARGETS ${name})

  message(STATUS "Created target ${name} for export ${PROJECT_NAME}.")

endfunction()
