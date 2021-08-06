/**
 * @file UnicyclePlanner.h
 * @authors Diego Ferigo, Stefano Dafarra
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H

#include "BipedalLocomotion/Contacts/ContactList.h"
#include "BipedalLocomotion/ParametersHandler/IParametersHandler.h"
#include "BipedalLocomotion/System/Advanceable.h"

#include <Eigen/Core>
#include <Eigen/Dense>

#include <memory>

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

    double x;
    double y;

    double dx = 0.0;
    double dy = 0.0;

    double time = 0.0;
};

struct BipedalLocomotion::Planners::UnicyclePlannerInput
{
    UnicyclePlannerInput(const std::vector<UnicycleKnot>& _knots,
                         const double _tf = 0.0,
                         const double _t0 = 0.0)
        : t0(_t0)
        , tf(_tf)
        , knots(_knots)
    {
    }

    double t0;
    double tf;

    std::vector<UnicycleKnot> knots;
};

struct BipedalLocomotion::Planners::UnicyclePlannerOutput
{
    UnicyclePlannerOutput(const Contacts::ContactList& _left = {},
                          const Contacts::ContactList& _right = {})
        : left(_left)
        , right(_right)
    {
    }

    Contacts::ContactList left;
    Contacts::ContactList right;
};

class BipedalLocomotion::Planners::UnicyclePlanner final
    : public System::Advanceable<UnicyclePlannerInput, UnicyclePlannerOutput>
{
public:
    UnicyclePlanner();

    virtual ~UnicyclePlanner();

    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    const UnicyclePlannerOutput& getOutput() const override;

    bool isOutputValid() const override;

    bool setInput(const UnicyclePlannerInput& input) override;

    bool advance() override;

private:
    class Impl;
    std::unique_ptr<Impl> m_pImpl;
};

#endif // BIPEDAL_LOCOMOTION_PLANNERS_UNICYCLE_PLANNER_H
