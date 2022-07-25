/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H

#include "BipedalLocomotion/Contacts/ContactList.h"
#include "BipedalLocomotion/ParametersHandler/IParametersHandler.h"
#include "BipedalLocomotion/System/Advanceable.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <memory>
#include <optional>

namespace BipedalLocomotion::Planners
{
struct UnicycleKnot;
class UnicyclePlanner;
struct UnicyclePlannerInput;
struct UnicyclePlannerOutput;
} // namespace BipedalLocomotion::Planners

struct BipedalLocomotion::Planners::UnicycleKnot
{
    UnicycleKnot(const double _x = 0.0,
                 const double _y = 0.0,
                 const double _dx = 0.0,
                 const double _dy = 0.0,
                 const double _t = 0.0)
        : x(_x)
        , y(_y)
        , dx(_dx)
        , dy(_dy)
        , time(_t)
    {
    }

    UnicycleKnot(const Eigen::Vector2d& _position = {0, 0},
                 const Eigen::Vector2d& _velocity = {0, 0},
                 const double _time = 0.0)
        : x(_position[0])
        , y(_position[1])
        , dx(_velocity[0])
        , dy(_velocity[1])
        , time(_time)
    {
    }

    bool operator==(const UnicycleKnot& rhs)
    {
        return this->x == rhs.x && this->y == rhs.y && this->dx == rhs.dx && this->dy == rhs.dy
               && this->time == rhs.time;
    }

    double x; ///< The knot x coordinates.
    double y; ///< The knot y coordinates.

    double dx = 0.0; ///< The knot x velocity.
    double dy = 0.0; ///< The knot y velocity.

    double time = 0.0; ///< The knot activation time.
};

struct BipedalLocomotion::Planners::UnicyclePlannerInput
{
    UnicyclePlannerInput(const std::vector<UnicycleKnot>& _knots,
                         const double _tf = 0.0,
                         const std::optional<Contacts::PlannedContact>& _initialLeftContact = {},
                         const std::optional<Contacts::PlannedContact>& _initialRightContact = {},
                         const double _t0 = 0.0)
        : t0(_t0)
        , tf(_tf)
        , initialLeftContact(_initialLeftContact)
        , initialRightContact(_initialRightContact)
        , knots(_knots)
    {
    }

    double t0; ///< The beginning of the planner horizon.
    double tf; ///< The end of the planner horizon.

    std::optional<Contacts::PlannedContact> initialLeftContact; ///< Left contact initialization
    std::optional<Contacts::PlannedContact> initialRightContact; ///< Right contact initialization

    std::vector<UnicycleKnot> knots; ///< A list of knots.
};

struct BipedalLocomotion::Planners::UnicyclePlannerOutput
{
    UnicyclePlannerOutput(const Contacts::ContactList& _left = {},
                          const Contacts::ContactList& _right = {})
        : left(_left)
        , right(_right)
    {
    }

    Contacts::ContactList left; ///< The list of left foot contacts;
    Contacts::ContactList right; ///< The list of right foot contacts;
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
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
