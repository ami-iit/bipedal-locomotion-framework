# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

framework_dependent_option(FRAMEWORK_COMPILE_example_devices
  "Compile example devices?" ON
  "BUILD_DEVICE_EXAMPLES" OFF)


function(add_bipedal_yarp_device)
  set(options )
  set(oneValueArgs NAME)
  set(multiValueArgs
    SOURCES
    PUBLIC_HEADERS
    PRIVATE_HEADERS
    PUBLIC_LINK_LIBRARIES
    PRIVATE_LINK_LIBRARIES
    TYPE
    CONFIGURE_PACKAGE_NAME)

  set(prefix "bipedal_yarp")

  cmake_parse_arguments(${prefix}
          "${options}"
          "${oneValueArgs}"
          "${multiValueArgs}"
          ${ARGN})

  set(name ${${prefix}_NAME})
  set(type ${${prefix}_TYPE})
  set(ini_package ${${prefix}_CONFIGURE_PACKAGE_NAME})
  set(sources ${${prefix}_SOURCES})
  set(public_headers ${${prefix}_PUBLIC_HEADERS})
  set(public_link_libraries ${${prefix}_PUBLIC_LINK_LIBRARIES})
  set(private_link_libraries ${${prefix}_PRIVATE_LINK_LIBRARIES})

  set(YARP_FORCE_DYNAMIC_PLUGINS ON)
  # Warning: the <package> option of yarp_configure_plugins_installation should be different from the plugin name
  yarp_configure_plugins_installation(${ini_package})

  yarp_prepare_plugin(${name} CATEGORY device
                              TYPE ${type}
                              INCLUDE ${public_headers}
                              OPTION ENABLE_${name}
                              DEFAULT ON)

  if(NOT SKIP_${name})

    yarp_add_plugin(${name} ${sources} ${public_headers})

    target_link_libraries(${name} PUBLIC ${public_link_libraries})
    target_link_libraries(${name} PRIVATE ${private_link_libraries})
    target_compile_features(${name} PUBLIC cxx_std_17)

    # Specify include directories for both compilation and installation process.
    # The $<INSTALL_PREFIX> generator expression is useful to ensure to create
    # relocatable configuration files, see https://cmake.org/cmake/help/latest/manual/cmake-packages.7.html#creating-relocatable-packages
    target_include_directories(${name} PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
      "$<INSTALL_INTERFACE:$<INSTALL_PREFIX>/${CMAKE_INSTALL_INCLUDEDIR}>")

    # Specify installation targets, typology and destination folders.
    yarp_install(TARGETS ${name}
      COMPONENT runtime
      LIBRARY DESTINATION ${YARP_DYNAMIC_PLUGINS_INSTALL_DIR}
      ARCHIVE DESTINATION ${YARP_STATIC_PLUGINS_INSTALL_DIR}
      YARP_INI DESTINATION ${YARP_PLUGIN_MANIFESTS_INSTALL_DIR})

    add_subdirectory(app)

    message(STATUS "Created device ${name} for export ${PROJECT_NAME}.")

  endif()

endfunction()
