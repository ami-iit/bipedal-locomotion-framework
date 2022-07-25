/**
 * @file QPFixedBaseTSID.h
 * @authors Ines Sorrentino, Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef BIPEDAL_LOCOMOTION_QP_FIXED_BASE_TSID_H
#define BIPEDAL_LOCOMOTION_QP_FIXED_BASE_TSID_H

#include <memory>
#include <optional>
#include <functional>

#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotion/System/VariablesHandler.h>
#include <BipedalLocomotion/TSID/QPTSID.h>

#include <iDynTree/KinDynComputations.h>

namespace BipedalLocomotion
{

namespace TSID
{

/**
 * QPFixedBaseTSID is specialization of QPTSID class in the case of fixed base system.
 * The TSID is here implemented as Quadratic Programming (QP) problem. The user should
 * set the desired task with the method QPFixedBaseTSID::addTask. Each task has a given
 * priority. Currently we support only priority equal to 0 or 1. If the task priority is set to 0
 * the task will be considered as a hard task, thus treated as a constraint. If the priority
 * is equal to 1 the task will be embedded in the cost function. The class is also able to treat
 * inequality constraints. Note that this class considers just one contact wrench as we assume the
 * external wrench acting on only the base link.
 * Here you can find an example of the QPFixedBaseTSID class.
 * <br/> <img
 * src="https://user-images.githubusercontent.com/43743081/112606007-308f7780-8e18-11eb-875f-d8a7c4b960eb.png"
 * width="1500">
 * @note If you want to solve the Inverse Dynamics for a floating base system please use QPTSID.
 */
class QPFixedBaseTSID : public QPTSID
{
    /**
     * Private implementation
     */
    struct Impl;
    std::unique_ptr<Impl> m_pimpl;

public:

    /**
     * Constructor.
     */
    QPFixedBaseTSID();

    /**
     * Destructor.
     */
    ~QPFixedBaseTSID();

    /**
     * Initialize the TSID algorithm.
     * @param handler pointer to the IParametersHandler interface.h
     * @note the following parameters are required by the class
     * |            Parameter Name            |   Type   |                                             Description                                            | Mandatory |
     * |:------------------------------------:|:--------:|:--------------------------------------------------------------------------------------------------:|:---------:|
     * |  `robot_acceleration_variable_name`  | `string` | Name of the variable contained in `VariablesHandler` describing the generalized robot acceleration |    Yes    |
     * |    `robot_torque_variable_name`      | `string` |         Name of the variable contained in `VariablesHandler` describing the robot torque           |    Yes    |
     * |             `verbosity`              |  `bool`  |                        Verbosity of the solver. Default value `false`                              |     No    |
     * Where the generalized robot acceleration is a vector containing the base acceleration
     * (expressed in mixed representation) and the joint accelerations,
     * @return True in case of success, false otherwise.
     */
    bool initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler) override;

    /**
     * Finalize the TSID.
     * @param handler parameter handler.
     * @note You should call this method after you add ALL the tasks.
     * @return true in case of success, false otherwise.
     */
    bool finalize(const System::VariablesHandler& handler) override;

    /**
     * Set the kinDynComputations object.
     * @param kinDyn pointer to a kinDynComputations object.
     * @return True in case of success, false otherwise.
     */
    bool setKinDyn(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);
};

} // namespace TSID
} // namespace BipedalLocomotion

#endif // BIPEDAL_LOCOMOTION_FIXED_BASE_TSID_H
