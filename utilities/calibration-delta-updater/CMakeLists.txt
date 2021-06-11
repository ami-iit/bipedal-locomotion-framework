# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v2.1 or any later version.
# Authors: Giulio Romualdi

if(FRAMEWORK_COMPILE_CalibrationDeltaUpdaterApplication)

  install(FILES script/blf-calibration-delta-updater.py
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    DESTINATION "${CMAKE_INSTALL_BINDIR}")

endif()
