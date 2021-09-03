/**
 * @file DummySensorBridgeTest.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// Catch2
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <catch2/catch.hpp>

#include "DummySensorBridge.h"
#include <iostream>
#include <vector>
using namespace BipedalLocomotion::RobotInterface;
using namespace BipedalLocomotion::ParametersHandler;
using namespace BipedalLocomotion;

bool populateConfig(std::weak_ptr<IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        return false;
    }

    auto optionsGroup = std::make_shared<StdImplementation>();
    optionsGroup->setParameter("streamLinearAccelerometerMeasurements", true);
    handle->setGroup("Options", optionsGroup);

    auto sourcesGroup = std::make_shared<StdImplementation>();
    sourcesGroup->setParameter("LinearAccelerometers", std::vector<std::string>{"dummy"});
    handle->setGroup("Sources", sourcesGroup);

    return true;
}

TEST_CASE("Test Sensor Bridge Interface")
{
    std::shared_ptr<StdImplementation> originalHandler = std::make_shared<StdImplementation>();
    IParametersHandler::shared_ptr parameterHandler = originalHandler;

    // Populate the input configuration to be passed to the estimator
    REQUIRE(populateConfig(parameterHandler));

    std::shared_ptr<DummySensorBridge> dummyBridge = std::make_shared<DummySensorBridge>();
    REQUIRE(dummyBridge->initialize(parameterHandler));

    double accRecvTime;
    Eigen::Vector3d acc;
    std::string someName{"dummy"};
    REQUIRE(dummyBridge->getLinearAccelerometerMeasurement(someName, acc, accRecvTime));
    REQUIRE(acc(2) == -9.8);
    REQUIRE(accRecvTime == 10.0);
}
