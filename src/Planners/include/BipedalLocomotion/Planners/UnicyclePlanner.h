/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <Eigen/src/Core/Matrix.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <memory>

#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>

#include <CoMHeightTrajectoryGenerator.h>

namespace BipedalLocomotion::Planners
{
struct UnicycleKnot;
class UnicyclePlanner;
struct UnicyclePlannerInput;
struct UnicyclePlannerOutput;
} // namespace BipedalLocomotion::Planners

struct BipedalLocomotion::Planners::UnicyclePlannerInput
{
    /*
    if UnicycleController::PERSON_FOLLOWING, the plannerInput is a vector of size 2 (i.e., [x, y])
    if UnicycleController::DIRECT, the plannerInput is a vector of size 3 (i.e., [xdot, ydot, wz])
    */
    Eigen::VectorXd plannerInput; // The input to the unicycle planner .

    DCMInitialState dcmInitialState; // The initial state of the DCM trajectory generator.

    bool correctLeft; // Whether to correct the left foot pose.

    double initTime; // The initial time of the trajectory.

    iDynTree::Transform measuredTransform; // The measured transform of the foot.

    static UnicyclePlannerInput generateDummyUnicyclePlannerInput();
};

struct BipedalLocomotion::Planners::UnicyclePlannerOutput
{
    struct COMHeightTrajectory
    {
        std::vector<double> comHeightPosition;
        std::vector<double> comHeightVelocity;
        std::vector<double> comHeightAcceleration;
    };

    struct DCMTrajectory
    {
        std::vector<Eigen::Vector2d> dcmPosition;
        std::vector<Eigen::Vector2d> dcmVelocity;
    };

    COMHeightTrajectory comHeightTrajectory; // The CoM height trajectory;

    DCMTrajectory dcmTrajectory; // The DCM trajectory;

    Contacts::ContactPhaseList ContactPhaseList; // The list of foot contact phases;
};

class BipedalLocomotion::Planners::UnicyclePlanner final
    : public System::Advanceable<UnicyclePlannerInput, UnicyclePlannerOutput>
{
public:
    UnicyclePlanner();

    virtual ~UnicyclePlanner();

    // clang-format off

    /**
     * Initialize the planner.
     *
     * @note The following parameters are required by the class:
     *
     * |          Name          |      Type      |      Default      | Mandatory |                    Description                     |
     * | :--------------------: | :------------: | :---------------: | :-------: | :------------------------------------------------: |
     * |    `sampling_time`     |     double     |         -         |    Yes    |          The sampling time of the planner          |
     * |     `unicycleGain`     |     double     |       10.0        |    No     |      The main gain of the unicycle controller      |
     * | `slowWhenTurningGain`  |     double     |        0.0        |    No     |     The turnin gain of the unicycle controller     |
     * |  `referencePosition`   | list of double |   (0.10, 0.00)    |    No     | The reference position of the unicycle controller  |
     * |      `timeWeight`      |     double     |        1.0        |    No     |         The time weight of the OC problem          |
     * |    `positionWeight`    |     double     |        1.0        |    No     |       The position weight of the OC problem        |
     * |   `minStepDuration`    |     double     |         -         |    Yes    |           The minimum duration of a step           |
     * |   `maxStepDuration`    |     double     |         -         |    Yes    |           The maximum duration of a step           |
     * |   `nominalDuration`    |     double     |         -         |    Yes    |           The nominal duration of a step           |
     * |    `minStepLength`     |     double     |         -         |    Yes    |            The minimum length of a step            |
     * |    `maxStepLength`     |     double     |         -         |    Yes    |            The maximum length of a step            |
     * |       `minWidth`       |     double     |         -         |    Yes    |             The minimum feet distance              |
     * |     `nominalWidth`     |     double     |         -         |    Yes    |             The nominal feet distance              |
     * |  `minAngleVariation`   |     double     |         -         |    Yes    |           The minimum unicycle rotation            |
     * |  `maxAngleVariation`   |     double     |         -         |    Yes    |           The maximum unicycle rotation            |
     * | `switchOverSwingRatio` |     double     |         -         |    Yes    | The ratio between single and double support phases |
     * |      `swingLeft`       |      bool      |       false       |    No     |     Perform the first step with the left foot      |
     * |     `terminalStep`     |      bool      |       true        |    No     |   Add a terminal step at the end of the horizon    |
     * | `startAlwaysSameFoot`  |      bool      |       false       |    No     |       Restart with the default foot if still       |
     * |    `left_foot_name`    |     string     |       left        |    No     |               Name of the left foot                |
     * |   `right_foot_name`    |     string     |       right       |    No     |               Name of the right foot               |
     *
     * @param handler Pointer to the parameter handler.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    const UnicyclePlannerOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool setInput(const UnicyclePlannerInput& input) override;

    bool advance() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;

    UnicycleController
    getUnicycleControllerFromString(const std::string& unicycleControllerAsString);

    bool generateFirstTrajectory(const Eigen::Ref<Eigen::Vector3d>& initialBasePosition);

    double m_dt;
    double m_plannerHorizon;
    double m_leftYawDeltaInRad;
    double m_rightYawDeltaInRad;
    double m_nominalWidth;
    int m_leftContactFrameIndex;
    int m_rightContactFrameIndex;
    Eigen::Vector2d m_referencePointDistance;
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
