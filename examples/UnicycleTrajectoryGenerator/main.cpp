#include <BipedalLocomotion/ParametersHandler/StdImplementation.h>
#include <BipedalLocomotion/Planners/SwingFootPlanner.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryGenerator.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryPlanner.h>
#include <BipedalLocomotion/System/Clock.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/Model.h>
#include <iDynTree/ModelLoader.h>

#include <chrono>
#include <matioCpp/matioCpp.h>
#include <vector>

#include <FolderPath.h>

using namespace BipedalLocomotion;

// define some parameters of the unicycle trajectory generator
std::shared_ptr<ParametersHandler::IParametersHandler> getUnicycleParametersHandler(double dt)
{
    std::shared_ptr<ParametersHandler::IParametersHandler> handler
        = std::make_shared<ParametersHandler::StdImplementation>();

    handler->setParameter("dt", dt);
    handler->setParameter("planner_advance_time_in_s", 0.18);
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

    bool saveResults = false;
    if (argc > 1)
    {
        if (std::string(argv[1]) == "--save")
        {
            // results are saved in a mat file located in the directory
            // where the executable is run
            saveResults = true;
        }
    }

    // matio file
    matioCpp::File resultFile;
    if (saveResults)
    {
        resultFile = matioCpp::File::Create("UnicycleTrajectoryGeneratorResults.mat");
    }
    std::vector<Eigen::Vector3d> positionLeftFoot;
    std::vector<Eigen::Vector3d> positionRightFoot;
    std::vector<Eigen::Vector3d> positionCOM;
    std::vector<Eigen::Vector3d> velocityCOM;
    std::vector<Eigen::Vector2d> DCMposition;
    std::vector<Eigen::Vector2d> DCMvelocity;
    std::vector<double> time;
    size_t i = 0;

    // set the Bipedal Locomotion clock factory
    BipedalLocomotion::System::ClockBuilder::setFactory(
        std::make_shared<BipedalLocomotion::System::StdClockFactory>());

    // load robot model
    auto jointsList
        = std::vector<std::string>{"l_hip_pitch",      "l_hip_roll",       "l_hip_yaw",
                                   "l_knee",           "l_ankle_pitch",    "l_ankle_roll",
                                   "r_hip_pitch",      "r_hip_roll",       "r_hip_yaw",
                                   "r_knee",           "r_ankle_pitch",    "r_ankle_roll",
                                   "torso_pitch",      "torso_roll",       "torso_yaw",
                                   "neck_pitch",       "neck_roll",        "neck_yaw",
                                   "l_shoulder_pitch", "l_shoulder_roll",  "l_shoulder_yaw",
                                   "l_elbow",          "r_shoulder_pitch", "r_shoulder_roll",
                                   "r_shoulder_yaw",   "r_elbow"};
    iDynTree::ModelLoader ml;
    ml.loadReducedModelFromFile(getRobotModelPath(), jointsList);

    // sample time [s] of the unicycle trajectory generator
    auto dt = 0.002;
    std::chrono::milliseconds dtChrono = std::chrono::milliseconds(static_cast<int>(dt * 1000));

    // initialize the unicyle trajectory generator
    Planners::UnicycleTrajectoryGenerator unicycleTrajectoryGenerator;
    unicycleTrajectoryGenerator.initialize(getUnicycleParametersHandler(dt));
    unicycleTrajectoryGenerator.setRobotContactFrames(ml.model());

    // initialize the input of the unicycle trajectory generator
    Planners::UnicycleTrajectoryGeneratorInput input
        = Planners::UnicycleTrajectoryGeneratorInput::generateDummyUnicycleTrajectoryGeneratorInput();

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

        // set linear velocity in x-direction
        if ((currentTime - timeStart) < (timeEnd - timeStart) / 6)
        {
            input.plannerInput[0] = 0.0;
        } else if ((currentTime - timeStart) > 4 * (timeEnd - timeStart) / 6)
        {
            input.plannerInput[0] = 0.0;
        } else
        {
            input.plannerInput[0] = 0.1;
        }

        if (!isFirstIteration)
        {
            // print the feet pose
            log()->info("[main] Left foot position: {}", w_H_left.translation().transpose());
            log()->info("[main] Right foot position: {}", w_H_right.translation().transpose());
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
        if (!leftFootPlanner.setContactList(output.contactPhaseList.lists().at("left_foot")))
        {
            log()->error("[main] Unable to set the contact list of the left foot planner.");
            return EXIT_FAILURE;
        }
        if (!rightFootPlanner.setContactList(output.contactPhaseList.lists().at("right_foot")))
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

        if (saveResults)
        {
            positionLeftFoot.push_back(w_H_left.translation());
            positionRightFoot.push_back(w_H_right.translation());
            positionCOM.push_back(output.comTrajectory.position.front());
            velocityCOM.push_back(output.comTrajectory.velocity.front());
            DCMposition.push_back(output.dcmTrajectory.position.front());
            DCMvelocity.push_back(output.dcmTrajectory.velocity.front());

            i++;
            time.push_back(
                std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - timeStart)
                    .count());
        }
        // get the DCM trajectory from the unicycle trajectory generator
        auto dcmPosition = output.dcmTrajectory.position;
        auto dcmVelocity = output.dcmTrajectory.velocity;

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

    // save the results to a mat file
    if (saveResults)
    {
        auto toMatioTimeVector = matioCpp::make_variable("time", time);
        resultFile.write(toMatioTimeVector);

        Eigen::Matrix3Xd footPositionMatrix(3, positionLeftFoot.size());
        for (size_t i = 0; i < positionLeftFoot.size(); i++)
        {
            footPositionMatrix.col(i) = positionLeftFoot[i];
        }
        auto toMatioEigenVec = matioCpp::make_variable("LeftFootPosition", footPositionMatrix);
        resultFile.write(toMatioEigenVec);

        for (size_t i = 0; i < positionRightFoot.size(); i++)
        {
            footPositionMatrix.col(i) = positionRightFoot[i];
        }
        toMatioEigenVec = matioCpp::make_variable("RightFootPosition", footPositionMatrix);
        resultFile.write(toMatioEigenVec);

        Eigen::Matrix3Xd comPositionMatrix(3, positionCOM.size());
        for (size_t i = 0; i < positionCOM.size(); i++)
        {
            comPositionMatrix.col(i) = positionCOM[i];
        }

        toMatioEigenVec = matioCpp::make_variable("COMPosition", comPositionMatrix);
        resultFile.write(toMatioEigenVec);

        Eigen::Matrix3Xd comVelocityMatrix(3, velocityCOM.size());
        for (size_t i = 0; i < velocityCOM.size(); i++)
        {
            comVelocityMatrix.col(i) = velocityCOM[i];
        }
        toMatioEigenVec = matioCpp::make_variable("COMVelocity", comVelocityMatrix);
        resultFile.write(toMatioEigenVec);

        Eigen::Matrix2Xd dcmPositionMatrix(2, DCMposition.size());
        for (size_t i = 0; i < DCMposition.size(); i++)
        {
            dcmPositionMatrix.col(i) = DCMposition[i];
        }
        toMatioEigenVec = matioCpp::make_variable("DCMPosition", dcmPositionMatrix);
        resultFile.write(toMatioEigenVec);

        Eigen::Matrix2Xd dcmVelocityMatrix(2, DCMvelocity.size());
        for (size_t i = 0; i < DCMvelocity.size(); i++)
        {
            dcmVelocityMatrix.col(i) = DCMvelocity[i];
        }
        toMatioEigenVec = matioCpp::make_variable("DCMVelocity", dcmVelocityMatrix);
        resultFile.write(toMatioEigenVec);
    }

    return EXIT_SUCCESS;
}
