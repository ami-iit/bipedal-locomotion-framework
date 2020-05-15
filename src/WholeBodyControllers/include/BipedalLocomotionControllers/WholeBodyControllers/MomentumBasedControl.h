/**
 * @file MomentumBasedControl.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#ifndef BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_CONTROL_H
#define BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_CONTROL_H

#include <string>
#include <unordered_map>

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.h>

namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{
class MomentumBasedControl
{
    enum class WalkingState
    {
        DoubleSupport,
        SingleSupportLeft,
        SingleSupportRight,
    };
    std::unordered_map<WalkingState, std::unique_ptr<MomentumBasedControlHelper>> m_controllers;

    WalkingState m_currentWalkingState;

public:
    MomentumBasedControl(std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler,
                    const iDynTree::VectorDynSize& maxJointsPosition,
                    const iDynTree::VectorDynSize& minJointsPosition);

    void setFeetState(bool isLeftInContact, bool isRightInContact);

    void
    setCentroidalMomentumReference(const iDynTree::SpatialForceVector& momentumSecondDerivative,
                                   const iDynTree::SpatialForceVector& momentumDerivative,
                                   const iDynTree::SpatialForceVector& momentum,
                                   const iDynTree::Vector3& centerOfMass);

    bool setMeasuredContactWrench(
        const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

    void setRotationReference(const iDynTree::Vector3& acceleration,
                              const iDynTree::Vector3& velocity,
                              const iDynTree::Rotation& rotation,
                              const std::string& name);

    void setContactState(const iDynTree::Transform& transform, const std::string& name);

    void setTransformationReference(const iDynTree::SpatialAcc& acceleration,
                                    const iDynTree::Twist& twist,
                                    const iDynTree::Transform& transform,
                                    const std::string& name);

    void setFootUpperBoundNormalForce(const std::string& name, const double& force);

    void setRegularizationReference(const iDynTree::VectorDynSize& acceleration,
                                    const iDynTree::VectorDynSize& velocity,
                                    const iDynTree::VectorDynSize& position,
                                    const std::string& name);

    void
    setJointState(const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position);

    bool solve();

    iDynTree::VectorDynSize getDesiredAcceleration();

    void setContactParameters(const std::string& name, const double& k, const double& b);
};
} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_CONTROL_H
