/**
 * @file MomentumBasedControlHelper.h
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#ifndef BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_CONTROL_HELPER_H
#define BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_CONTROL_HELPER_H

#include <memory>
#include <string>
#include <unordered_map>
#include <variant>
#include <vector>

#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/VectorDynSize.h>
#include <iDynTree/Core/SpatialVector.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumRateOfChangeElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumRateOfChangeBounds.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ContactModelElement.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/ContactWrenchFeasibilityElement.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FloatingBaseMultiBodyDynamicsElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Weight.h>
#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>

// osqp-eigen solver
#include <OsqpEigen/OsqpEigen.h>

namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{
class MomentumBasedControlHelper
{
    template <typename T> using dictionary = std::map<std::string, T>;
    template <typename T> using unique_ptr = std::unique_ptr<T>;

    enum class FootType
    {
        Swing,
        Stance
    };

    std::vector<OptimalControlUtilities::Frame<std::string, std::string>> m_swingFeetIdetrifiers;
    std::vector<OptimalControlUtilities::Frame<std::string, std::string>> m_stanceFeetIdetrifiers;

    const std::shared_ptr<iDynTree::KinDynComputations> m_kinDyn; /**< KinDyn pointer object */
    unique_ptr<OsqpEigen::Solver> m_solver; /**< Quadratic programming solver */
    OptimalControlUtilities::VariableHandler m_variableHandler; /**< Variable handler */
    unique_ptr<OptimalControlUtilities::Constraints> m_constraints; /**< Collection of all the constraints */
    unique_ptr<OptimalControlUtilities::CostFunction> m_costFunction; /**< The cost function */

    bool m_isVerbose{true}; /**< If true the controller will be verbose */

    std::string m_description;

    /** Dynamics of the floating base */
    unique_ptr<OptimalControlUtilities::FloatingBaseDynamicsElement> m_floatingBaseDynamics;

    /** Dynamics of the joint base */
    unique_ptr<OptimalControlUtilities::JointSpaceDynamicsElement> m_jointDynamics;

    /** Centroidal linear momentum in case of elastic contacts */
    unique_ptr<OptimalControlUtilities::CentroidalLinearMomentumRateOfChangeElement> m_centroidalLinearMomentumElement;

    /** Centroidal angular momentum in case of elastic contacts */
    unique_ptr<OptimalControlUtilities::CentroidalAngularMomentumRateOfChangeElement> m_centroidalAngularMomentumElement;

    /** Centroidal angular momentum in case of elastic contacts */
    unique_ptr<OptimalControlUtilities::CentroidalAngularMomentumRateOfChangeBounds> m_centroidalAngularMomentumBound;

    /** Dictionary containing regularization elements */
    dictionary<unique_ptr<OptimalControlUtilities::RegularizationElement>> m_regularizationElements;

    /** Dictionary containing regularization with control elements */
    dictionary<unique_ptr<OptimalControlUtilities::RegularizationWithControlElement>>
        m_regularizationWithControlElements;

    /** Dictionary containing Orientation elements */
    dictionary<unique_ptr<OptimalControlUtilities::CartesianElement<
        OptimalControlUtilities::CartesianElementType::POSE>>>
        m_cartesianElements;

    dictionary<unique_ptr<OptimalControlUtilities::CartesianElement<
        OptimalControlUtilities::CartesianElementType::ORIENTATION>>>
        m_orientationElements;

    // Joint values element
    unique_ptr<OptimalControlUtilities::JointValuesFeasibilityElement> m_jointValuesFeasibilityElement;

    dictionary<unique_ptr<OptimalControlUtilities::ContactWrenchRateOfChangeFeasibilityElement>> m_contactWrenchFeasibilityElements;

    dictionary<unique_ptr<OptimalControlUtilities::ContactModelElement>> m_contactModelElements;

    template <class T>
    bool addFeetTypeIdentifiers(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                const FootType& type);

    template <class T>
    bool addFeetIdentifiers(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler);

    template <class T>
    bool addLinearMomentumElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler);

    template <class T>
    bool addAngularMomentumElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler);

    template <class T>
    bool addAngularMomentumBounds(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler);

    template <OptimalControlUtilities::CartesianElementType type, class T>
    bool addCartesianElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                             const OptimalControlUtilities::Frame<std::string, std::string>& frame);

    template <class T>
    bool addSystemDynamicsElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler);

    template <class T>
    bool addRegularizationWithControlElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                             const std::string& label);

    template <class T>
    bool addRegularizationElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                  const std::string& label);


    template <class T>
    bool addJointValuesFeasibilityElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                          const iDynTree::VectorDynSize& maxJointsPosition,
                                          const iDynTree::VectorDynSize& minJointsPosition);

    template <class T>
    bool addContactWrenchFeasibilityElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                            const OptimalControlUtilities::Frame<std::string, std::string>& frame);

    template <class T>
    bool addContactModelElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                const OptimalControlUtilities::Frame<std::string, std::string>& frame);

    void printElements() const;

    void initialzeSolver();

    void initializeVariableHandler();

public:
    MomentumBasedControlHelper(const std::shared_ptr<iDynTree::KinDynComputations> kinDyn);

    template <class T>
    bool initialize(std::weak_ptr<ParametersHandler::IParametersHandler<T>> handlerWeak,
                    const std::string& controllerType,
                    const iDynTree::VectorDynSize& maxJointsPosition,
                    const iDynTree::VectorDynSize& minJointsPosition);

    bool solve();

    void setVerbosity(bool isVerbose) noexcept;

    void setCentroidalMomentumReference(const iDynTree::SpatialForceVector& momentumSecondDerivative,
                                        const iDynTree::SpatialForceVector& momentumDerivative,
                                        const iDynTree::SpatialForceVector& momentum,
                                        const iDynTree::Vector3& centerOfMass);

    bool setMeasuredContactWrench(const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches);

    void setContactState(const std::string& name,
                         bool isInContact,
                         const iDynTree::Transform& desiredFootPose);

    void setRotationReference(const iDynTree::Vector3& acceleration,
                              const iDynTree::Vector3& velocity,
                              const iDynTree::Rotation& rotation,
                              const std::string& name);

    void setTransformationReference(const iDynTree::SpatialAcc& acceleration,
                                    const iDynTree::Twist& twist,
                                    const iDynTree::Transform& transform,
                                    const std::string& name);

    void setRegularizationReference(const iDynTree::VectorDynSize& acceleration,
                                    const iDynTree::VectorDynSize& velocity,
                                    const iDynTree::VectorDynSize& position,
                                    const std::string& name);

    void setJointState(const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position);

    void setFootUpperBoundNormalForce(const std::string& name, const double& force);

    iDynTree::VectorDynSize getDesiredTorques();
    iDynTree::VectorDynSize getDesiredAcceleration();

    iDynTree::Vector6 getLeftFootWrenchRateOfChange() const;

    iDynTree::Vector6 getRightFootWrenchRateOfChange() const;
};
} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers

#include "MomentumBasedControlHelper.tpp"

#endif // BIPEDAL_LCOMOTION_CONTROLLERS_WHOLE_BODY_CONTROLLERS_MOMENTUM_BASED_CONTROL_HELPER_H
