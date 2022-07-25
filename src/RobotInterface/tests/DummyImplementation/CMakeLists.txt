# Copyright (C) 2020 Istituto Italiano di Tecnologia (IIT). All rights reserved.
# This software may be modified and distributed under the terms of the
# BSD-3-Clause license.

add_bipedal_test(
  NAME DummySensorBridge
  SOURCES DummySensorBridge.h DummySensorBridgeTest.cpp
  LINKS BipedalLocomotion::RobotInterface)
