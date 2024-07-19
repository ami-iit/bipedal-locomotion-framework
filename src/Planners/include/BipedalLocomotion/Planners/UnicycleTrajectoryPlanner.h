/**
 * @file UnicycleTrajectoryPlanner.h
 * @authors Lorenzo Moretti, Diego Ferigo, Giulio Romualdi, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_TRAJECTORY_PLANNER_H

#include <BipedalLocomotion/Contacts/ContactPhaseList.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/Advanceable.h>

#include <iDynTree/Model.h>
#include <iDynTree/VectorDynSize.h>

#include <CoMHeightTrajectoryGenerator.h>
#include <UnicycleGenerator.h>
#include <UnicyclePlanner.h>

#include <chrono>
#include <deque>
#include <memory>
#include <string>
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
    /**
     * UnicycleTrajectoryPlannerInput implements the input to the planner. Depending on the type of
     * unicycle controller used the plannerInput is a 2d-vector or a 3d-vector. For instance, if
     * unicycle controller is set to UnicycleController::PERSON_FOLLOWING, the plannerInput is a
     * vector of size 2 that represents a reference position (i.e., [x, y]). Instead, if it is set
     * to UnicycleController::DIRECT, the plannerInput is a vector of size 3 that representes a
     * velocity command (i.e., [xdot, ydot, wz]).
     */
    Eigen::VectorXd plannerInput; /**< Input to the unicycle planner */

    DCMInitialState dcmInitialState; /**< Initial state of the DCM trajectory generator */

    struct COMInitialState
    {
        Eigen::Vector2d initialPlanarPosition; /**< Initial planar position of the CoM */
        Eigen::Vector2d initialPlanarVelocity; /**< Initial planar velocity of the CoM */
    };

    COMInitialState comInitialState; /**< Initial state of the CoM */

    bool isLeftLastSwinging; /**< True if the last foot that was swinging is the left one. False
                                  otherwise. */

    std::chrono::nanoseconds initTime; /**< Initial time of the trajectory */

    manif::SE3d measuredTransform; /**< Measured transform of the last foot that touched
                                        the floor */

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
        std::vector<Eigen::Vector2d> position;
        std::vector<Eigen::Vector2d> velocity;
    };

    struct ContactStatus
    {
        std::vector<bool> leftFootInContact; /**< True if the left foot is in contact. False
                                                  otherwise. */
        std::vector<bool> rightFootInContact; /**< True if the right foot is in contact. False
                                                   otherwise. */
        std::vector<bool> UsedLeftAsFixed; /**< True if the left foot is the last that got in
                                                contact. */
    };

    struct Steps
    {
        std::deque<Step> leftSteps, rightSteps;
    };

    struct FootTrajectory
    {
        std::vector<manif::SE3d> transform; /**< Foot transform */
        std::vector<manif::SE3d::Tangent> mixedVelocity; /**< Spatial velocity in mixed
                                                           representation */
        std::vector<manif::SE3d::Tangent> mixedAcceleration; /**< Spatial acceleration in mixed
                                                       representation */
    };

    COMTrajectory comTrajectory; /**< CoM trajectory */

    DCMTrajectory dcmTrajectory; /**< DCM trajectory */

    ContactStatus contactStatus; /**< Contact status of the feet */

    Steps steps; /**< List of steps and their phases */

    FootTrajectory leftFootTrajectory; /**< Left foot trajectory */

    FootTrajectory rightFootTrajectory; /**< Right foot trajectory */

    std::vector<size_t> mergePoints; /**< Indexes of the merge points of the trajectory */
};

struct BipedalLocomotion::Planners::UnicycleTrajectoryPlannerParameters
{
    std::chrono::nanoseconds dt; /**< Sampling time of the planner */

    std::chrono::nanoseconds plannerHorizon; /**< Time horizon of the planner */

    std::chrono::nanoseconds minStepDuration; /**< Time minimum time duration of a step */

    double leftYawDeltaInRad; /**< Left foot cartesian offset in the yaw */

    double rightYawDeltaInRad; /**< Right foot cartesian offset in the yaw */

    Eigen::Vector2d referencePointDistance; /**< Reference position of the unicycle
                                                 controller */

    double nominalWidth; /**< Nominal feet distance */

    int leftContactFrameIndex; /**< Index of the left foot contact frame */

    std::string leftContactFrameName; /**< Name of the left foot contact frame */

    int rightContactFrameIndex; /**< Index of the right foot contact frame */

    std::string rightContactFrameName; /**< Name of the right foot contact frame */
};

/**
 * UnicycleTrajectoryPlanner is a class that uses UnicycleGenerator of
 * https://github.com/robotology/unicycle-footstep-planner) to generate reference trajectories for
 * humanoid robots. Every time the advance() function is called, the planner generates a new
 * trajectory, which spans the time horizon configured by the user.
 * The getOutput() member function returns the generated trajectory, which includes the CoM, DCM,
 * and footstep ones. The planner requires the user to set the robot model using the setRobotModel()
 * member function, before invoking the initialize() member function, which configures the planner.
 * As input, which is set using the setInput() member function, the planner requires an instance of
 * the UnicycleTrajectoryPlannerInput struct.
 */
class BipedalLocomotion::Planners::UnicycleTrajectoryPlanner final
    : public System::Advanceable<UnicycleTrajectoryPlannerInput, UnicycleTrajectoryPlannerOutput>
{
public:
    UnicycleTrajectoryPlanner();

    virtual ~UnicycleTrajectoryPlanner();

    /**
     * Set the robot contact frames.
     * It should be called after the initialize() function.
     * It checks if the contact frames names parsed by the initialize() function exist.
     * If yes, it sets the related contact frames indexes and returns true.
     * Otherwise, it sets the Impl::FSM::State back to NotInitialized and returns false.
     * @param model iDynTree::Model of the robot considered.
     */
    bool setRobotContactFrames(const iDynTree::Model& model);

    /**
     * Get the index of the right foot contact frame.
     * @return The index of the right foot contact frame.
     */
    int getRightContactFrameIndex() const;

    /**
     * Get the index of the left foot contact frame.
     * @return The index of the left foot contact frame.
     */
    int getLeftContactFrameIndex() const;

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
     * @param handler Pointer to the parameter handler.
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;
    // clang-format on

    /**
     * Get the output of the planner.
     * @return The output of the planner.
     */
    const UnicycleTrajectoryPlannerOutput& getOutput() const override;

    /**
     * Check if the output is valid.
     * @return True if the output is valid, false otherwise.
     */
    bool isOutputValid() const override;

    /**
     * Set the input of the planner.
     * @param input Input of the planner.
     * @return True in case of success, false otherwise.
     */
    bool setInput(const UnicycleTrajectoryPlannerInput& input) override;

    /**
     * Advance the planner.
     * @return True in case of success, false otherwise.
     */
    bool advance() override;

    /**
     * Get the contact phase list.
     * It returns the contact phase list built with the footsteps generated by the planner.
     * @return The contact phase list.
     */
    Contacts::ContactPhaseList getContactPhaseList();

    /**
     * Get the minimum time duration of a step.
     * @return minimum time duration of a step.
     */
    std::chrono::nanoseconds getMinStepDuration() const;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;

    /**
     * Set the unicycle controller mode based on the given string.
     * @param unicycleControllerAsString unicycle controller mode as string.
     * @param unicycleController unicycle controller mode.
     * @return True in case of success, false otherwise.
     */
    bool setUnicycleControllerFromString(const std::string& unicycleControllerAsString,
                                         UnicycleController& unicycleController);

    /**
     * Generate the first trajectory.
     */
    bool generateFirstTrajectory();
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
