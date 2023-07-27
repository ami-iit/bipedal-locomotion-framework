# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
# Authors: Giulio Romualdi

if(FRAMEWORK_COMPILE_BalancingPositionControlApplication)

  install(FILES script/blf-balancing-torque-control.py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    DESTINATION "${CMAKE_INSTALL_BINDIR}")

  install(DIRECTORY balancing_torque_control
    DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/python")


  install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

endif()
