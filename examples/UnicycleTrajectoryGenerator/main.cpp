#include "BipedalLocomotion/Contacts/ContactPhaseList.h"
#include "BipedalLocomotion/System/StdClock.h"
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryGenerator.h>

#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/System/YarpClock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <Eigen/src/Core/Matrix.h>
#include <chrono>
#include <iDynTree/Position.h>
#include <iDynTree/Transform.h>
#include <manif/impl/se3/SE3.h>
#include <memory>

#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

#include <yarp/os/Network.h>
#include <yarp/os/RFModule.h>

using namespace BipedalLocomotion;

// define some parameters of the unicycle trajectory generator
std::shared_ptr<ParametersHandler::IParametersHandler> getUnicycleParametersHandler(double dt)
{
    std::shared_ptr<ParametersHandler::IParametersHandler> handler
        = std::make_shared<ParametersHandler::StdImplementation>();

    handler->setParameter("dt", dt);
    handler->setParameter("referencePosition", Eigen::Vector2d(0.1, 0.0));
    handler->setParameter("saturationFactors", Eigen::Vector2d(0.7, 0.7));
    handler->setParameter("leftZMPDelta", Eigen::Vector2d(0.0, 0.0));
    handler->setParameter("rightZMPDelta", Eigen::Vector2d(0.0, 0.0));
    handler->setParameter("mergePointRatios", Eigen::Vector2d(0.4, 0.4));
    handler->setParameter("leftContactFrameName", "l_sole");
    handler->setParameter("rightContactFrameName", "r_sole");

    return handler;
}

// define some parameters of the swing foot planner
std::shared_ptr<ParametersHandler::IParametersHandler>
getSwingFootPlannerParametersHandler(double dt)
{
    std::shared_ptr<ParametersHandler::IParametersHandler> handler
        = std::make_shared<ParametersHandler::StdImplementation>();

    auto dtChrono = std::chrono::milliseconds(static_cast<int>(dt * 1000));

    handler->setParameter("sampling_time", dtChrono);
    handler->setParameter("step_height", 0.035);
    handler->setParameter("foot_apex_time", 0.5);
    handler->setParameter("foot_landing_velocity", 0.0);
    handler->setParameter("foot_landing_acceleration", 0.0);

    return handler;
}

int main(int argc, char* argv[])
{
    constexpr auto errorPrefix = "[main]";

    // set the Bipedal Locomotion clock factory
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::StdClockFactory>());

    // sample time [s] of the unicycle trajectory generator
    auto dt = 0.002;
    std::chrono::milliseconds dtChrono = std::chrono::milliseconds(static_cast<int>(dt * 1000));

    // initialize the unicyle trajectory generator
    Planners::UnicycleTrajectoryGenerator unicycleTrajectoryGenerator;
    unicycleTrajectoryGenerator.initialize(getUnicycleParametersHandler(dt));

    // initialize the input of the unicycle trajectory generator
    Planners::UnicycleTrajectoryGeneratorInput input
        = Planners::UnicycleTrajectoryGeneratorInput::generateDummyUnicycleTrajectoryGeneratorInput();

    input.w_H_leftFoot = iDynTree::Transform::Identity();
    input.w_H_leftFoot.setPosition({0.0, 0.1, 0.0});

    input.w_H_rightFoot = iDynTree::Transform::Identity();
    input.w_H_rightFoot.setPosition({0.0, -0.1, 0.0});

    // initialize the output of the unicycle trajectory generator
    Planners::UnicycleTrajectoryGeneratorOutput output;

    // initialize the swing foot planner
    Planners::SwingFootPlanner leftFootPlanner;
    leftFootPlanner.initialize(getSwingFootPlannerParametersHandler(dt));
    leftFootPlanner.setTime(std::chrono::nanoseconds::zero());
    Planners::SwingFootPlanner rightFootPlanner;
    rightFootPlanner.initialize(getSwingFootPlannerParametersHandler(dt));
    rightFootPlanner.setTime(std::chrono::nanoseconds::zero());

    // initialize loop variables
    bool isFirstIteration = true;
    manif::SE3d w_H_left;
    manif::SE3d w_H_right;

    // initial time of the simulation
    auto timeStart = BipedalLocomotion::clock().now();

    // final time of the simulation
    auto timeEnd = timeStart + std::chrono::seconds(20);

    while (BipedalLocomotion::clock().now() < timeEnd)
    {
        auto currentTime = BipedalLocomotion::clock().now();
        log()->info("[main] Current time: {}",
                    std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timeStart));

        if (!isFirstIteration)
        {
            // set linear vlocity in x-direction
            input.plannerInput[0] = 0.1;

            // update the feet pose
            input.w_H_leftFoot.setPosition(iDynTree::Position(w_H_left.translation()));
            input.w_H_rightFoot.setPosition(iDynTree::Position(w_H_right.translation()));

            log()->info("[main] Left foot position: {}", w_H_left.translation().transpose());
            log()->info("[main] Right foot position: {}", w_H_right.translation().transpose());

            input.w_H_leftFoot.setRotation(iDynTree::Rotation(w_H_left.rotation()));
            input.w_H_rightFoot.setRotation(iDynTree::Rotation(w_H_right.rotation()));
        }

        unicycleTrajectoryGenerator.setInput(input);

        if (!unicycleTrajectoryGenerator.advance())
        {
            log()->error("[main] Unable to advance the unicycle trajectory generator.");
            return EXIT_FAILURE;
        }

        output = unicycleTrajectoryGenerator.getOutput();

        if (!unicycleTrajectoryGenerator.isOutputValid())
        {
            log()->error("[main] The output of the unicycle trajectory generator is not valid.");
            return EXIT_FAILURE;
        }

        // set the contact list of the swing foot planners
        if (!leftFootPlanner.setContactList(output.ContactPhaseList.lists().at("left_foot")))
        {
            log()->error("[main] Unable to set the contact list of the left foot planner.");
            return EXIT_FAILURE;
        }
        if (!rightFootPlanner.setContactList(output.ContactPhaseList.lists().at("right_foot")))
        {
            log()->error("[main] Unable to set the contact list of the right foot planner.");
            return EXIT_FAILURE;
        }

        // advance the swing foot planners
        if (!leftFootPlanner.advance())
        {
            log()->error("[main] Unable to advance the left foot planner.");
            return EXIT_FAILURE;
        }
        if (!rightFootPlanner.advance())
        {
            log()->error("[main] Unable to advance the right foot planner.");
            return EXIT_FAILURE;
        }

        // get the feet pose from the swing foot planners
        w_H_left = leftFootPlanner.getOutput().transform;
        w_H_right = rightFootPlanner.getOutput().transform;

        // get the DCM trajectory from the unicycle trajectory generator
        auto dcmPosition = output.dcmTrajectory.dcmPosition;
        auto dcmVelocity = output.dcmTrajectory.dcmVelocity;

        Eigen::VectorXd Xdcm;
        Xdcm.resize(dcmPosition.size());

        Eigen::VectorXd Ydcm;
        Ydcm.resize(dcmPosition.size());

        for (size_t i = 0; i < dcmPosition.size(); i++)
        {
            Xdcm(i) = dcmPosition[i][0];
        }

        for (size_t i = 0; i < dcmPosition.size(); i++)
        {
            Ydcm(i) = dcmPosition[i][1];
        }

        // log()->info("[main] DCM x: {}", Xdcm.transpose());
        // log()->info("[main] DCM y: {}", Ydcm.transpose());

        BipedalLocomotion::clock().sleepUntil(currentTime + dtChrono);

        isFirstIteration = false;
    }

    return EXIT_SUCCESS;
}