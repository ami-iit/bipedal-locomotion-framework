/**
 * @file UnicycleTrajectoryGenerator.h
 * @authors Lorenzo Moretti, Stefano Dafarra, Giulio Romualdi
 * @copyright 2024 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_GENERATOR_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_GENERATOR_H

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Planners/UnicycleTrajectoryPlanner.h>
#include <BipedalLocomotion/System/Advanceable.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

#include <iDynTree/VectorDynSize.h>

#include <CoMHeightTrajectoryGenerator.h>
#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>

#include <chrono>
#include <memory>
namespace BipedalLocomotion::Planners
{
class UnicycleTrajectoryGenerator;
struct UnicycleTrajectoryGeneratorInput;
struct UnicycleTrajectoryGeneratorOutput;
struct UnicycleTrajectoryGeneratorParameters;
} // namespace BipedalLocomotion::Planners

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorInput
{
    /**
     * UnicycleTrajectoryPlannerInput implements the input to the planner. Depending on the type of
     * unicycle controller used the plannerInput is a 2d-vector or a 3d-vector. For instance, if
     * unicycle controller is set to UnicycleController::PERSON_FOLLOWING, the plannerInput is a
     * vector of size 2 that represents a reference position (i.e., [x, y]). Instead, if it is set
     * to UnicycleController::DIRECT, the plannerInput is a vector of size 3 that representes a
     * velocity command (i.e., [xdot, ydot, wz]).
     */
    Eigen::VectorXd plannerInput; /**< Input to the unicycle planner */

    static UnicycleTrajectoryGeneratorInput generateDummyUnicycleTrajectoryGeneratorInput();
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorOutput
    : public BipedalLocomotion::Planners::UnicycleTrajectoryPlannerOutput
{
    bool isValid = true; /**< True if the output is valid, false otherwise. */
    BipedalLocomotion::Contacts::ContactPhaseList contactPhaseList; /**< Contact phase list */
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorParameters
{
    std::chrono::nanoseconds dt; /**< Sampling time of the planner */
    size_t plannerAdvanceTimeSteps; /**< Number of time steps that the planner should be called in
                                         advance */
    int leftContactFrameIndex; /**< Index of the left contact frame */
    int rightContactFrameIndex; /**< Index of the right contact frame */
};

/**
 * UnicycleTrajectoryGenerator is a class that generates reference trajectories for
 * humanoid robots.
 * Every time that the advance() member function is called, the Generator:
 *   1. First checks if it is time to generate a new trajectory. In this case, it deploys the
        BipedalLocomotion::Planners::UnicycleTrajectoryPlanner to plan this new trajectory.
 *   2. Then, it checks if it is time to merge the current trajectory with the last one
        computed by the UnicycleTrajectoryPlanner. In this case, it merges the two trajectories,
        which become the current one.
 *   3. Finally, it unrolls the current trajectory (i.e., it advances it over time).
 * The getOutput() member function returns the current trajectory, which includes the CoM, DCM,
 * and footstep ones.
 * The Generator requires the user to set the robot model using the setRobotModel() member function,
 * before invoking the initialize() member function, which configures the Generator.
 * As input, which is set by the setInput() member function, the Generator requires an instance of
 * the UnicycleTrajectoryGeneratorInput struct.
 */
class BipedalLocomotion::Planners::UnicycleTrajectoryGenerator final
    : public System::Advanceable<UnicycleTrajectoryGeneratorInput, UnicycleTrajectoryGeneratorOutput>
{
public:
    UnicycleTrajectoryGenerator();

    virtual ~UnicycleTrajectoryGenerator();

    /**
     * Set the robot contact frames.
     * It should be called after the initialize() function.
     * It checks if the contact frames names parsed by the initialize() function exist.
     * If yes, it sets the related contact frames indexes and returns true.
     * Otherwise, it sets the Impl::FSM::State back to NotInitialized and returns false.
     * @param model iDynTree::Model of the robot considered.
     */
    bool setRobotContactFrames(const iDynTree::Model& model);

    // clang-format off
    /**
     * Initialize the planner.
     *
     * @note The following parameters are required by the class:
     * |           Name              |      Type      |      Default      |     Example     |                     Description                           |
     * | :-------------------------: | :------------: | :---------------: | :-------------: | :-------------------------------------------------------: |
     * | `planner_advance_time_in_s` |     double     |       0.08        |       -         |     The time in advance at which the planner is called    |
     * |           `dt`              |     double     |      0.002        |       -         |     The sampling time of the trajectory generator         |
     * | `leftContactFrameName`      |     string     |         -         |    "l_sole"     |     Name of the left foot contact frame                   |
     * | `rightContactFrameName`     |     string     |         -         |    "r_sole"     |     Name of the right foot contact frame                  |
     *
     * Implicitely, the class needs also all the parameters required by the Bipedalocotion::Planner::UnicyclePlanner class.
    // clang-format on

     * @param handler Pointer to the parameter handler.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * @brief Get the output of the planner.
     * @return Output of the planner.
     */
    const UnicycleTrajectoryGeneratorOutput& getOutput() const override;

    /**
     * @brief Check if the output is valid.
     * @return True if the output is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * @brief Set the input of the planner.
     * @param input Input of the planner.
     * @return True in case of success, false otherwise.
     */
    bool setInput(const UnicycleTrajectoryGeneratorInput& input) override;

    /**
     * @brief Advance the planner.
     * @return True in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * @brief Get the sampling time of the planner.
     * @return Sampling time of the planner.
     */
    std::chrono::nanoseconds getSamplingTime() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;

    /**
     * @brief Generate the first trajectory at initialization phase.
     * @return True in case of success, false otherwise.
     */
    bool generateFirstTrajectory();
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_GENERATOR_H
