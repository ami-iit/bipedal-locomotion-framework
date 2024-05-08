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

    manif::SE3d w_H_leftFoot; /**< Left foot pose in the world frame */

    manif::SE3d w_H_rightFoot; /**< Right foot pose in the world frame */

    static UnicycleTrajectoryGeneratorInput generateDummyUnicycleTrajectoryGeneratorInput();
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorOutput
    : public BipedalLocomotion::Planners::UnicycleTrajectoryPlannerOutput
{
    bool isValid = true; /**< True if the output is valid, false otherwise. */
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorParameters
{
    std::chrono::nanoseconds dt; /**< Sampling time of the planner */
    size_t plannerAdvanceTimeSteps; /**< Number of time steps that the planner should be called in
                                         advance */
    int leftContactFrameIndex; /**< Index of the left contact frame */
    int rightContactFrameIndex; /**< Index of the right contact frame */
};

class BipedalLocomotion::Planners::UnicycleTrajectoryGenerator final
    : public System::Advanceable<UnicycleTrajectoryGeneratorInput, UnicycleTrajectoryGeneratorOutput>
{
public:
    UnicycleTrajectoryGenerator();

    virtual ~UnicycleTrajectoryGenerator();

    /**
     * Set the robot model.
     * @param model model of the robot considered.
     */
    bool setRobotModel(const iDynTree::Model& model);

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

    const UnicycleTrajectoryGeneratorOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool setInput(const UnicycleTrajectoryGeneratorInput& input) override;

    bool advance() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;

    UnicycleController
    getUnicycleControllerFromString(const std::string& unicycleControllerAsString);

    bool generateFirstTrajectory();
};

namespace BipedalLocomotion::Planners::Utilities
{
template <typename T>
bool appendVectorToDeque(const std::vector<T>& input,
                         std::deque<T>& output,
                         const size_t& initPoint)
{
    if (initPoint > output.size())
    {
        BipedalLocomotion::log()->error("[Utilities::appendVectorToDeque] The init point has to "
                                        "be less or equal to the size of the output deque.");
        return false;
    }

    // resize the deque
    output.resize(input.size() + initPoint);

    // Advances the iterator it by initPoint positions
    typename std::deque<T>::iterator it = output.begin();
    std::advance(it, initPoint);

    // copy the vector into the deque from the initPoint position
    std::copy(input.begin(), input.end(), it);

    return true;
}

template <typename T>
void populateVectorFromDeque(const std::deque<T>& deque, std::vector<T>& vector)
{

    vector.clear();

    vector.insert(vector.end(), deque.begin(), deque.end());
}

}; // namespace BipedalLocomotion::Planners::Utilities

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_GENERATOR_H
