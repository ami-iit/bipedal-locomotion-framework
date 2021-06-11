# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

if(FRAMEWORK_COMPILE_CalibrationDeltaUpdaterApplication)

  install(FILES script/blf_calibration_delta_updater.py
    DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/python")

endif()
