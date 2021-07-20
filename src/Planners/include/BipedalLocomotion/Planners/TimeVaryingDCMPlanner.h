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
    ~TimeVaryingDCMPlanner() override;

    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |           Parameter Name          |    Type    |                                                                   Description                                                                  | Mandatory |
     * |:---------------------------------:|:----------:|:----------------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |          `linear_solver`          |  `string`  | Linear solver used by ipopt, the default value is mumps. Please check [here](https://coin-or.github.io/Ipopt/#PREREQUISITES) for the available solvers |     No    |
     * |      `planner_sampling_time`      |  `double`  |                                                          Sampling time of the planner                                                          |    Yes    |
     * |      `number_of_foot_corners`     |    `int`   |                                      Number of the corner of the polygon used to describe the foot. E.g. 4                                     |    Yes    |
     * |         `foot_corner_<i>`         | `Vector3d` |             A 3d vector describing the position of the corner w.r.t. frame associated to the foot. `i = 0:number_of_foot_corners`.             |    Yes    |
     * |         `omega_dot_weight`        |  `double`  |                                                   Weight associated to the \f$\dot{\omega}\f$                                                   |    Yes    |
     * |       `dcm_tracking_weight`       |  `double`  |                                                      Weight associated to the DCM tracking                                                     |    Yes    |
     * | `omega_dot_rate_of_change_weight` |  `double`  |                                          Weight associated to the rate of change of \f$\dot{\omega}\f$                                          |    Yes    |
     * |    `vrp_rate_of_change_weight`    |  `double`  |                                               Weight associated to the rate of change of the VRP                                               |    Yes    |
     * |    `dcm_rate_of_change_weight`    |  `double`  |                                               Weight associated to the rate of change of the DCM                                               |    Yes    |
     * |    `use_external_dcm_reference`   |   `bool`   |  Set this option to true if you want provide an external DCM reference with TimeVaryingDCMPlanner::setDCMReference(). (Default value False)    |    No     |
     * |            `gravity`              |  `double`  |  Value of the gravity acceleration. It should be a positive number (Default Math::StandardAccelerationOfGravitation )  |    No     |
     * @return true in case of success/false otherwise.
     */
     bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Compute the DCM trajectory.
     * @return true in case of success, false otherwise.
     */
     bool computeTrajectory() final;

    /**
     * Set the contact phase list
     * @note the contactPhaseList pointer should point to an already initialized ContactPhaseList.
     * Please be sure that the memory pointed is reachable for the entire life time of the
     * DCMPlanner class.
     * @param contactPhaseList pointer containing the list of the contact phases
     * @return true in case of success, false otherwise.
     */
    bool setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList) final;

    /**
     * Set the DCM reference. This value will be used as regularization term from the planner.
     * @param dcmReference matrix containing the DCM trajectory. \a dcmReference must be a 3 x n
     * matrix where the rows are the x y and z coordinates of the DCM and the columns contain the
     * trajectory of the DCM.
     * @note The number of columns should be coherent with the contactPhaseList. In details the
     * number of samples \f$s\f$ is equal to
     * \f[
     *  s = \left\lceil {\frac{t_f - t_i}{dT}} \right\rceil
     * \f]
     * where \f$dT\f$ is the sampling time of the planner, \f$t_f\f$ and \f$t_i\f$ are the end time
     * of the last contact and the initial time of the first contact.
     * @return true in case of success, false otherwise.
     */
    bool setDCMReference(Eigen::Ref<const Eigen::MatrixXd> dcmReference) final;

    /**
     * Set the initial state of the planner
     * @param initialState the initial state of the planner
     */
    void setInitialState(const DCMPlannerState& initialState) final;

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    const DCMPlannerState& getOutput() const final;

    /**
     * @brief Determines the validity of the object retrieved with get()
     * @return True if the object is valid, false otherwise.
     */
     bool isOutputValid() const final;

    /**
     * @brief Advance the internal state. This may change the value retrievable from get().
     * @return True if the advance is successfull.
     */
     bool advance() final;

};
} // namespace Planners
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_PLANNERS_TIME_VARYING_DCM_PLANNER_H
