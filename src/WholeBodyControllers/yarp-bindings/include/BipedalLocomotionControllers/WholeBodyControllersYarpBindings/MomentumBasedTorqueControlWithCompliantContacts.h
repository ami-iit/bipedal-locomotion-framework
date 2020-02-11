/**
 * @file MomentumBasedTorqueControlWithCompliantContacts.h
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#ifndef BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_YARP_BINDINGS_MOMENTUM_BASED_TORQUE_CONTROL_WITH_COMPLIANT_CONTACTS_H
#define BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_YARP_BINDINGS_MOMENTUM_BASED_TORQUE_CONTROL_WITH_COMPLIANT_CONTACTS_H

#include <memory>
#include <string>

#include <yarp/os/Searchable.h>

#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Core/VectorDynSize.h>

#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedTorqueControlWithCompliantContacts.h>

namespace BipedalLocomotionControllers
{
namespace WholeBodyControllersYarpBindings
{

class MomentumBasedTorqueControl
    : public BipedalLocomotionControllers::WholeBodyControllers::MomentumBasedTorqueControl
{
    bool addCentroidalLinearMomentumElement(const yarp::os::Searchable& config);

    bool addOrientationElement(const yarp::os::Searchable& config, const std::string& label);

    void addFloatingBaseDynamicsElement(const yarp::os::Searchable& config);

    void addJointDynamicsElement(const yarp::os::Searchable& config);

    void addRegularizationWithControlElement(const yarp::os::Searchable& config,
                                             const std::string& label);

    void addJointValuesFeasibilityElement(const yarp::os::Searchable& config,
                                          const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                          const iDynTree::VectorDynSize& minJointPositionsLimit);

public:
    MomentumBasedTorqueControl(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    ~MomentumBasedTorqueControl() = default;

    /**
     * Initialize the Task-based torque control problem.
     * @param config config of the control problem solver
     * @param minJointsPosition is a vector containing the min joints position limit
     * @param minJointsPosition is a vector containing the max joints position limit
     */
    bool initialize(const yarp::os::Searchable& config,
                    const iDynTree::VectorDynSize& maxJointsPosition,
                    const iDynTree::VectorDynSize& minJointsPosition);
};
} // namespace WholeBodyControllersYarpBindings
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_YARP_BINDINGS_MOMENTUM_BASED_TORQUE_CONTROL_WITH_COMPLIANT_CONTACTS_H
