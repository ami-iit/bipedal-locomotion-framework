/**
 * @file UnicyclePlannerTest.cpp
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */
#include <fstream>

#include <chrono>

// Catch2
#include <catch2/catch_test_macros.hpp>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::ParametersHandler;


#include <manif/manif.h>


bool approxEqual(const manif::SE3d& lhs, const manif::SE3d& rhs, const double& translationTol = 1e-1, const double& rotationTol = 1e2)
{
    Eigen::Matrix3d rotationDiffMat = (lhs.rotation().matrix() * rhs.rotation().inverse().matrix()).eval();
    Eigen::AngleAxisd rotationDiffAA(rotationDiffMat);
    return (lhs.translation() - rhs.translation()).norm() < translationTol && rotationDiffAA.angle() < rotationTol;
}

void saveData(const UnicyclePlannerOutput& output, const std::string& filename)
{
    std::ofstream file(filename);
    if (!file.is_open()) {
        std::cerr << "Error: Unable to open file " << filename << " for writing." << std::endl;
        return;
    }

    file << "Left_foot_position_x Left_foot_position_y Left_foot_position_z  Left_foot_quat_x Left_foot_quat_y Left_foot_quat_z Left_foot_quat_w"
            "Right_foot_position_x Right_foot_position_y Right_foot_position_z Right_foot_quat_x Right_foot_quat_y Right_foot_quat_z Right_foot_quat_w \n";

    for (size_t i = 0; i < output.left.size(); ++i)
    {
        const auto& left_pose = output.left[i].pose;
        const auto& right_pose = output.right[i].pose;

        file << left_pose.coeffs().transpose() << " "
             << right_pose.coeffs().transpose() << "\n";
    }

    file.close();
}




std::shared_ptr<IParametersHandler> params(const double& dT)
{
    // Set the non-default parameters of the planner
    std::shared_ptr<IParametersHandler> handler = std::make_shared<StdImplementation>();

    handler->setParameter("sampling_time", dT);
    handler->setParameter("minStepDuration", 0.7);
    handler->setParameter("maxStepDuration", 1.31);
    handler->setParameter("nominalDuration", 1.0);
    handler->setParameter("maxStepLength", 0.5);
    handler->setParameter("minStepLength", 0.01);
    handler->setParameter("minWidth", 0.12);
    handler->setParameter("nominalWidth", 0.17);
    handler->setParameter("minAngleVariation", 5.0);
    handler->setParameter("maxAngleVariation", 18.0);
    handler->setParameter("switchOverSwingRatio", 0.3);
    handler->setParameter("positionWeight", 100.0);

    return handler;
}

TEST_CASE("UnicyclePlannerTest")
{
    const double dT = 0.01;
    const auto handler = params(dT);

    bool saveDataTofile = false;

    UnicyclePlanner planner;

    REQUIRE(planner.initialize(handler));

    UnicyclePlannerInput input(
        {UnicycleKnot({0.0, 0.0}, {0.0, 0.0}, 0.0),
        UnicycleKnot({0.5, 0.0}, {0.0, 0.0}, 1.0),
        UnicycleKnot({1.0, 0.0}, {0.0, 0.0}, 1.5)},
        20.0,
        std::nullopt,
        std::nullopt,
        0.0);
    UnicyclePlannerOutput output;

    REQUIRE(planner.setInput(input));
    REQUIRE(planner.advance());
    output = planner.getOutput();

    REQUIRE(output.left.size() > 2);
    REQUIRE(output.left.size() == output.right.size() );

    // require last contact of output.left to have x pos = last UnicycleKnot - referencePosition

    const auto& lastLeftContact = output.left.rbegin()->pose;
    REQUIRE(approxEqual(lastLeftContact, manif::SE3d({input.knots.back().x - 0.1, input.knots.back().y, 0.0}, Eigen::Quaterniond::Identity())));

    if (saveDataTofile)
    {
        saveData(output, "UnicyclePlannerTestOutput.txt");
    }

}
