/**
 * @file CentroidalMPC.h
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_CENTROIDAL_MPC_H
#define BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_CENTROIDAL_MPC_H

#include <memory>
#include <vector>
#include <map>

#include <BipedalLocomotion/System/Source.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Contacts/ContactPhaseList.h>

namespace BipedalLocomotion
{
namespace ReducedModelControllers
{

struct CentroidalMPCState
{
    std::map<std::string, Contacts::ContactWithCorners> contacts;
};

/**
 * DCMPlanner defines a trajectory generator for the variable height Divergent component of motion
 * (DCM).
 */
class CentroidalMPC : public System::Source<CentroidalMPCState>
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
    CentroidalMPC();

    /**
     * Destructor.
     */
    ~CentroidalMPC();

    /**
     * Initialize the planner.
     * @param handler pointer to the parameter handler.
     * @note the following parameters are required by the class
     * |           Parameter Name          |    Type    |                                                                   Description                                                                  | Mandatory |
     * |:---------------------------------:|:----------:|:----------------------------------------------------------------------------------------------------------------------------------------------:|:---------:|
     * |          `linear_solver`          |  `string`  | Linear solver used by ipopt, the default value is mumps. Please check https://coin-or.github.io/Ipopt/#PREREQUISITES for the available solvers |     No    |
     * |      `planner_sampling_time`      |  `double`  |                                                          Sampling time of the planner                                                          |    Yes    |
     * |      `number_of_foot_corners`     |    `int`   |                                      Number of the corner of the polygon used to describe the foot. E.g. 4                                     |    Yes    |
     * |         `foot_corner_<i>`         | `Vector3d` |             A 3d vector describing the position of the corner w.r.t. frame associated to the foot. `i = 0:number_of_foot_corners`.             |    Yes    |
     * |         `omega_dot_weight`        |  `double`  |                                                   Weight associated to the \f$\dot{omega}\f$                                                   |    Yes    |
     * |       `dcm_tracking_weight`       |  `double`  |                                                      Weight associated to the DCM tracking                                                     |    Yes    |
     * | `omega_dot_rate_of_change_weight` |  `double`  |                                          Weight associated to the rate of change of \f$\dot{omega}\f$                                          |    Yes    |
     * |    `vrp_rate_of_change_weight`    |  `double`  |                                               Weight associated to the rate of change of the VRP                                               |    Yes    |
     * |    `dcm_rate_of_change_weight`    |  `double`  |                                               Weight associated to the rate of change of the DCM                                               |    Yes    |
     * @return true in case of success/false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) final;

    bool setContactPhaseList(const Contacts::ContactPhaseList &contactPhaseList);

    bool setState(Eigen::Ref<const Eigen::Vector3d> com,
                  Eigen::Ref<const Eigen::Vector3d> dcom,
                  Eigen::Ref<const Eigen::Vector3d> angularMomentum);

    bool setReferenceTrajectory(Eigen::Ref<const Eigen::MatrixXd> com);

    /**
     * @brief Get the object.
     * @return a const reference of the requested object.
     */
    const CentroidalMPCState& getOutput() const final;

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
} // namespace ReducedModelControllers
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_REDUCE_MODEL_CONTROLLERS_CENTROIDAL_MPC_H
