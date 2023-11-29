# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.
# Authors: Giulio Romualdi

if(FRAMEWORK_COMPILE_JointPositionTrackingApplication)

  add_bipedal_locomotion_application(
    NAME joint-position-tracking
    SOURCES src/Main.cpp src/Module.cpp
    HEADERS include/BipedalLocomotion/JointPositionTracking/Module.h
    LINK_LIBRARIES   YARP::YARP_dev BipedalLocomotion::Math BipedalLocomotion::ParametersHandlerYarpImplementation BipedalLocomotion::RobotInterfaceYarpImplementation
    )

  install_ini_files(${CMAKE_CURRENT_SOURCE_DIR}/config)

  install(FILES script/blf_joint_position_tracking_plot_dataset.py
    DESTINATION "${CMAKE_INSTALL_PREFIX}/share/BipedalLocomotionFramework/python")

  install(FILES script/blf-joint-position-tracking-script.sh
    PERMISSIONS OWNER_EXECUTE OWNER_WRITE OWNER_READ
    DESTINATION "${CMAKE_INSTALL_BINDIR}")

endif()
