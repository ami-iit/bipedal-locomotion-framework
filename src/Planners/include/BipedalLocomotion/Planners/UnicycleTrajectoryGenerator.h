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
#include <BipedalLocomotion/Planners/UnicyclePlanner.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/src/Core/Matrix.h>
#include <iDynTree/VectorDynSize.h>

#include <memory>

#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>

#include <CoMHeightTrajectoryGenerator.h>

namespace BipedalLocomotion::Planners
{
class UnicycleTrajectoryGenerator;
struct UnicycleTrajectoryGeneratorInput;
struct UnicycleTrajectoryGeneratorOutput;
struct UnicycleTrajectoryGeneratorParameters;
} // namespace BipedalLocomotion::Planners

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorInput
{
    /*
    if UnicycleController::PERSON_FOLLOWING, the plannerInput is a vector of size 2 (i.e., [x, y])
    if UnicycleController::DIRECT, the plannerInput is a vector of size 3 (i.e., [xdot, ydot, wz])
    */
    Eigen::VectorXd plannerInput; // The input to the unicycle planner.

    DCMInitialState dcmInitialState; // The initial state of the DCM trajectory generator.

    double initTime; // The initial time of the trajectory.

    iDynTree::Transform measuredTransform; // The measured transform of the last foot that touched
                                           // the floor.

    static UnicycleTrajectoryGeneratorInput generateDummyUnicycleTrajectoryGeneratorInput();
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorOutput
    : public BipedalLocomotion::Planners::UnicyclePlannerOutput
{
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryGeneratorParameters
{
    double dt; // The sampling time of the planner.
};

class BipedalLocomotion::Planners::UnicycleTrajectoryGenerator final
    : public System::Advanceable<UnicycleTrajectoryGeneratorInput, UnicycleTrajectoryGeneratorOutput>
{
public:
    UnicycleTrajectoryGenerator();

    virtual ~UnicycleTrajectoryGenerator();

    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    const UnicycleTrajectoryGeneratorOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool setInput(const UnicycleTrajectoryGeneratorInput& input) override;

    /*
    The advance method should be called only in DoubleSupport phase.
    */
    bool advance() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;

    UnicycleController
    getUnicycleControllerFromString(const std::string& unicycleControllerAsString);

    bool generateFirstTrajectory();
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_GENERATOR_H
