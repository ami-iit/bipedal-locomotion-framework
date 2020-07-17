/**
 * @file TimeVaryingDCMPlanner.h
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_PLANNERS_TIME_VARYING_DCM_PLANNER_H
#define BIPEDAL_LOCOMOTION_PLANNERS_TIME_VARYING_DCM_PLANNER_H

#include <memory>

#include <BipedalLocomotion/Planners/DCMPlanner.h>

namespace BipedalLocomotion
{
namespace Planners
{

/**
 * DCMPlanner defines a trajectory generator for the variable height Divergent component of motion
 * (DCM).
 */
class TimeVaryingDCMPlanner : public DCMPlanner
{
    /**
     * Private implementation
     */
    struct Impl;

    std::unique_ptr<Impl> m_pimpl; /**< Pointer to private implementation */

public:
    /**
     * Constructor.
     */
    TimeVaryingDCMPlanner();

    /**
     * Destructor.
     */
    ~TimeVaryingDCMPlanner();

    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @return true in case of success/false otherwise.
     */
     bool initialize(std::shared_ptr<ParametersHandler::IParametersHandler> handler) override;

    /**
     * Compute the DCM trajectory.
     * @return true in case of success, false otherwise.
     */
     bool computeTrajectory() final;

     /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
     const DCMPlannerState& get() const final;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
     bool isValid() const final;

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successfull.
     */
     bool advance() final;

};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_TIME_VARYING_DCM_PLANNER_H
