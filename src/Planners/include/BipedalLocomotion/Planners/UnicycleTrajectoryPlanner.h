/**
 * @file UnicycleTrajectoryPlanner.h
 * @authors Lorenzo Moretti, Diego Ferigo, Giulio Romualdi, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <deque>
#include <iDynTree/Model.h>
#include <iDynTree/VectorDynSize.h>

#include <memory>

#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>

#include <CoMHeightTrajectoryGenerator.h>
#include <vector>

namespace BipedalLocomotion::Planners
{
class UnicycleTrajectoryPlanner;
struct UnicycleTrajectoryPlannerInput;
struct UnicycleTrajectoryPlannerOutput;
struct UnicycleTrajectoryPlannerParameters;
} // namespace BipedalLocomotion::Planners

struct BipedalLocomotion::Planners::UnicycleTrajectoryPlannerInput
{
    /*
    if UnicycleController::PERSON_FOLLOWING, the plannerInput is a vector of size 2 (i.e., [x, y])
    if UnicycleController::DIRECT, the plannerInput is a vector of size 3 (i.e., [xdot, ydot, wz])
    */
    Eigen::VectorXd plannerInput; // The input to the unicycle planner.

    DCMInitialState dcmInitialState; // The initial state of the DCM trajectory generator.

    struct COMInitialState
    {
        Eigen::Vector2d initialPlanarPosition; // The initial planar position of the CoM.
        Eigen::Vector2d initialPlanarVelocity; // The initial planar velocity of the CoM.
    };

    COMInitialState comInitialState; // The initial state of the CoM.

    bool isLeftLastSwinging; // True if the last foot that was swinging is the left one. False
                             // otherwise.

    double initTime; // The initial time of the trajectory.

    manif::SE3d measuredTransform; // The measured transform of the last foot that touched
                                   // the floor.

    static UnicycleTrajectoryPlannerInput generateDummyUnicycleTrajectoryPlannerInput();
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryPlannerOutput
{
    struct COMTrajectory
    {
        std::vector<Eigen::Vector3d> position;
        std::vector<Eigen::Vector3d> velocity;
        std::vector<Eigen::Vector3d> acceleration;
    };

    struct DCMTrajectory
    {
        std::vector<Eigen::Vector2d> dcmPosition;
        std::vector<Eigen::Vector2d> dcmVelocity;
    };

    struct ContactStatus
    {
        std::vector<bool> leftFootInContact; // True if the left foot is in contact. False
                                             // otherwise.
        std::vector<bool> rightFootInContact; // True if the right foot is in contact. False
                                              // otherwise.
        std::vector<bool> UsedLeftAsFixed; // True if the left foot is the last that got in contact.
    };

    struct Steps
    {
        std::deque<Step> leftSteps, rightSteps;
    };

    COMTrajectory comTrajectory; // The CoM trajectory;

    DCMTrajectory dcmTrajectory; // The DCM trajectory;

    ContactStatus contactStatus; // The contact status of the feet;

    Contacts::ContactPhaseList ContactPhaseList; // The list of foot contact phases;

    Steps steps; // The list of steps and their phases;

    std::vector<size_t> mergePoints; // Indexes of the merge points of the trajectory;
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryPlannerParameters
{
    double dt; // The sampling time of the planner.

    double plannerHorizon; // The time horizon of the planner.

    double leftYawDeltaInRad; // Left foot cartesian offset in the yaw.

    double rightYawDeltaInRad; // Right foot cartesian offset in the yaw.

    Eigen::Vector2d referencePointDistance; // The reference position of the unicycle controller

    double nominalWidth; // The nominal feet distance.

    int leftContactFrameIndex; // The index of the left foot contact frame.

    int rightContactFrameIndex; // The index of the right foot contact frame.
};

class BipedalLocomotion::Planners::UnicycleTrajectoryPlanner final
    : public System::Advanceable<UnicycleTrajectoryPlannerInput, UnicycleTrajectoryPlannerOutput>
{
public:
    UnicycleTrajectoryPlanner();

    virtual ~UnicycleTrajectoryPlanner();

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
     *
     * |           Name            |      Type         |      Default      |     Example     |                       Description                               |
     * | :-----------------------: | :---------------: | :---------------: | :-------------: | :-------------------------------------------------------------: |
     * |  `referencePosition`      | list of 2 doubles |         -         |   (0.1 0.0)     | The reference position of the unicycle controller               |
     * |     `controlType`         |     string        |     "direct"      |       -         | The control mode used by the unicycle controller                |
     * |     `unicycleGain`        |     double        |       10.0        |       -         |      The main gain of the unicycle controller                   |
     * | `slowWhenTurningGain`     |     double        |        2.0        |       -         |     The turning gain of the unicycle controller                 |
     * | `slowWhenBackwardFactor`  |     double        |        0.4        |       -         |     The backward gain of the unicycle controller                |
     * | `slowWhenSidewaysFactor`  |     double        |        0.2        |       -         |     The sideways gain of the unicycle controller                |
     * |           `dt`            |     double        |      0.002        |       -         |          The sampling time of the planner                       |
     * |   `plannerHorizon`        |     double        |       20.0        |       -         |          The planner time horizon                               |
     * |    `positionWeight`       |     double        |        1.0        |       -         |       The position weight of the OC problem                     |
     * |      `timeWeight`         |     double        |        2.5        |       -         |         The time weight of the OC problem                       |
     * |    `maxStepLength`        |     double        |       0.32        |       -         |            The maximum length of a step                         |
     * |    `minStepLength`        |     double        |       0.01        |       -         |            The minimum length of a step                         |
     * |`maxLengthBackwardFactor`  |     double        |        0.8        |       -         |   The factor of maximum backward walk                           |
     * |     `nominalWidth`        |     double        |       0.20        |       -         |             The nominal feet distance                           |
     * |       `minWidth`          |     double        |       0.14        |       -         |             The minimum feet distance                           |
     * |   `minStepDuration`       |     double        |       0.65        |       -         |           The minimum duration of a step                        |
     * |   `maxStepDuration`       |     double        |        1.5        |       -         |           The maximum duration of a step                        |
     * |   `nominalDuration`       |     double        |        0.8        |       -         |           The nominal duration of a step                        |
     * |  `maxAngleVariation`      |     double        |       18.0        |       -         |           The maximum unicycle rotation                         |
     * |  `minAngleVariation`      |     double        |        5.0        |       -         |           The minimum unicycle rotation                         |
     * | `saturationFactors`       | list of 2 doubles |        -          |   (0.7 0.7)     |  Linear and Angular velocity conservative factors               |
     * | `leftYawDeltaInDeg`       |     double        |        0.0        |       -         | Offset for the left foot rotation around the z axis             |
     * | `rightYawDeltaInDeg`      |     double        |        0.0        |       -         | Offset for the right foot rotation around the z axis            |
     * |      `swingLeft`          |      bool         |       false       |       -         |     Perform the first step with the left foot                   |
     * | `startAlwaysSameFoot`     |      bool         |       false       |       -         |       Restart with the default foot if still                    |
     * |     `terminalStep`        |      bool         |       true        |       -         |   Add a terminal step at the end of the horizon                 |
     * |     `mergePointRatios`    | list of 2 doubles |         -         |   (0.4 0.4)     | The ratios of the DS phase in which it is present a merge point |
     * | `switchOverSwingRatio`    |     double        |        0.2        |       -         | The ratio between single and double support phases              |
     * | `lastStepSwitchTime`      |     double        |        0.3        |       -         |       Time duration of double support phase in final step       |
     * | `isPauseActive`           |      bool         |       true        |       -         |    If true, the planner can pause, instead of make tiny steps.  |
     * | `comHeight`               |     double        |        0.70       |       -         |    CoM height in double support phase                           |
     * | `comHeightDelta`          |     double        |        0.01       |       -         |    Delta to add to CoM heinght in Single support phases         |
     * | `leftZMPDelta`            | list of 2 doubles |         -         |   (0.0  0.0)    | Local ZMP reference: delta wrt center frame of the foot         |
     * | `rightZMPDelta`           | list of 2 doubles |         -         |   (0.0  0.0)    | Local ZMP reference: delta wrt center frame of the foot         |
     * | `lastStepDCMOffset`       |     double        |        0.5        |       -         | Last Step DCM Offset. If 0, DCM coincides with stance foot ZMP  |
     * | `leftContactFrameName`    |     string        |         -         |    "l_sole"     |       Name of the left foot contact frame                       |
     * | `rightContactFrameName`   |     string        |         -         |    "r_sole"     |       Name of the right foot contact frame                      |
     *
    // clang-format on

     * @param handler Pointer to the parameter handler.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    const UnicycleTrajectoryPlannerOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool setInput(const UnicycleTrajectoryPlannerInput& input) override;

    bool advance() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;

    bool setUnicycleControllerFromString(
    const std::string& unicycleControllerAsString, UnicycleController& unicycleController);

    bool generateFirstTrajectory();
};

namespace BipedalLocomotion::Planners::Utilities
{
bool getContactList(
    const double initTime,
    const double dt,
    const std::vector<bool>& inContact,
    const std::deque<Step>& steps,
    const int contactFrameIndex,
    const std::string& contactName,
    BipedalLocomotion::Contacts::ContactList& contactList);
}; // namespace BipedalLocomotion::Planners::Utilities

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
