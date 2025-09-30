# Copyright (C) 2025 Istituto Italiano di Tecnologia (IIT).

if(FRAMEWORK_COMPILE_JointLevelControllers)

  add_bipedal_test(
    NAME PositionToCurrentController
    SOURCES PositionToCurrentControllerTest.cpp
    LINKS BipedalLocomotion::JointLevelControllers)

  add_bipedal_test(
    NAME PositionToTorqueController
    SOURCES PositionToTorqueControllerTest.cpp
    LINKS BipedalLocomotion::JointLevelControllers)

  add_bipedal_test(
    NAME EMAWithLimits
    SOURCES EMAWithLimitsTest.cpp
    LINKS BipedalLocomotion::JointLevelControllers)

endif()
