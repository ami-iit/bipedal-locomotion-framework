/**
 * @file UnicyclePlannerTest.cpp
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <fstream>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryPlanner.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ParametersHandler;

#include <manif/manif.h>

void saveData(const UnicycleTrajectoryPlannerOutput& output, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    file << output.ContactPhaseList.toString() << std::endl;

    file.close();
}

std::shared_ptr<IParametersHandler> params()
{
    // Set the non-default parameters of the planner
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();

    handler->setParameter("referencePosition", Eigen::Vector2d(0.1, 0.0));
    handler->setParameter("saturationFactors", Eigen::Vector2d(0.7, 0.7));
    handler->setParameter("leftZMPDelta", Eigen::Vector2d(0.0, 0.0));
    handler->setParameter("rightZMPDelta", Eigen::Vector2d(0.0, 0.0));
    handler->setParameter("mergePointRatios", Eigen::Vector2d(0.4, 0.4));
    handler->setParameter("leftContactFrameName", "l_sole");
    handler->setParameter("rightContactFrameName", "r_sole");

    return handler;
}

TEST_CASE("UnicyclePlannerTest")
{
    const auto handler = params();

    bool saveDataTofile = false;

    BipedalLocomotion::Planners::UnicycleTrajectoryPlanner planner;

    REQUIRE(planner.initialize(handler));

    UnicycleTrajectoryPlannerInput input
        = UnicycleTrajectoryPlannerInput::generateDummyUnicycleTrajectoryPlannerInput();

    UnicycleTrajectoryPlannerOutput output;

    REQUIRE(planner.setInput(input));
    REQUIRE(planner.advance());
    REQUIRE(planner.isOutputValid());

    output = planner.getOutput();

    REQUIRE(output.ContactPhaseList.size() == 1);

    if (saveDataTofile)
    {
        saveData(output, "UnicycleTrajectoryPlannerTestOutput.txt");
    }
}
