
/**
 * @file MomentumBasedTorqueControlWithCompliantContacts.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#ifndef BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_TORQUE_CONTROL_WITH_COMPLIANT_CONTACTS_H
#define BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_TORQUE_CONTROL_WITH_COMPLIANT_CONTACTS_H

#include <memory>
#include <string>
#include <vector>

#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/KinDynComputations.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Weight.h>

namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{
class MomentumBasedTorqueControl
{
private:
    class Impl;
    std::unique_ptr<Impl> m_pimpl;

public:
    MomentumBasedTorqueControl(const std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    virtual ~MomentumBasedTorqueControl();

    /**
     * Set the verbosity of the controller
     * @param isVerbose if true the controller will prints all the warning
     */
    void setVerbosity(bool isVerbose) noexcept;

    void addCentroidalLinearMomentumElement(const std::vector<OptimalControlUtilities::FrameInContactWithContactModel<std::string, std::string>>& framesInContact,
                                            std::unique_ptr<OptimalControlUtilities::LinearPD<iDynTree::Vector3>> pdController,
                                            bool isConstraint,
                                            const OptimalControlUtilities::Weight<iDynTree::VectorDynSize>& weight = OptimalControlUtilities::Weight<iDynTree::VectorDynSize>::Zero(3));


    void addOrientationElement(const OptimalControlUtilities::Frame<std::string, std::string>& frameName,
                               std::unique_ptr<OptimalControlUtilities::OrientationPD> pdController,
                               bool isConstraint,
                               const OptimalControlUtilities::Weight<iDynTree::VectorDynSize>& weight = OptimalControlUtilities::Weight<iDynTree::VectorDynSize>::Zero(6));

    void addFloatingBaseDynamicsElement(const std::vector<OptimalControlUtilities::FrameInContact<std::string, std::string>>& framesInContact,
                                        bool isConstraint,
                                        const OptimalControlUtilities::Weight<iDynTree::VectorDynSize>& weight = OptimalControlUtilities::Weight<iDynTree::VectorDynSize>::Zero(1));

    void addRegularizationWithControlElement(const std::string& label,
                                             std::unique_ptr<OptimalControlUtilities::LinearPD<iDynTree::VectorDynSize>> pdController,
                                             bool isConstraint,
                                             const OptimalControlUtilities::Weight<iDynTree::VectorDynSize>& weight = OptimalControlUtilities::Weight<iDynTree::VectorDynSize>::Zero(1));

    void addJointValuesFeasibilityElement(const std::string& variableName,
                                          const iDynTree::VectorDynSize& maxJointPositionsLimit,
                                          const iDynTree::VectorDynSize& minJointPositionsLimit,
                                          const double& samplingTime);



    void initialize();

    void solve();

    size_t variableSize(const std::string& name) const;

    void
    setDesiredLinearMomentumValue(const iDynTree::Vector3& centroidalLinearMomentumSecondDerivative,
                                  const iDynTree::Vector3& centroidalLinearMomentumDerivative,
                                  const iDynTree::Vector3& centroidalLinearMomentum);


    void addJointDynamicsElement(const std::vector<OptimalControlUtilities::FrameInContact<std::string, std::string>>& framesInContact);
    /* bool getVerbosity() const; */

    void setMeasuredContactWrench(
        const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

    iDynTree::VectorDynSize getDesiredTorques();

    void setContactState(const std::string& name,
                         bool isInContact,
                         const iDynTree::Transform& desiredFootPose);

    void setDesiredRotationReference(const iDynTree::Vector3& acceleration,
                                     const iDynTree::Vector3& velocity,
                                     const iDynTree::Rotation& rotation,
                                     const std::string& name);

    void setDesiredRegularizationTrajectory(const iDynTree::VectorDynSize& acceleration,
                                            const iDynTree::VectorDynSize& velocity,
                                            const iDynTree::VectorDynSize& position,
                                            const std::string& name);

    void
    setJointState(const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position);

    /* void setDesiredVRP(const iDynTree::Vector3& VRP); */

    /* void setDesiredAngularMomentum(const iDynTree::Vector3& centroidalAngularMomentumVelocity, */
    /*                                const iDynTree::Vector3& centroidalAngularMomentum); */

};
} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers

#endif // BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_TORQUE_CONTROL_WITH_COMPLIANT_CONTACTS_H
