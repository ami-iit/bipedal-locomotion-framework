/**
 * @file MomentumBasedControlHelper.cpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

// std library
#include <unordered_map>
#include <vector>

// osqp-eigen solver
#include <OsqpEigen/OsqpEigen.h>

// iDynTree library
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Core/Wrench.h>
#include <iDynTree/KinDynComputations.h>
#include <iDynTree/Model/Model.h>

// BipedalLocomotionControllers library
#include <BipedalLocomotionControllers/OptimalControlUtilities/CartesianElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/CentroidalMomentumElementsWithCompliantContacts.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FeasibilityElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/FloatingBaseMultiBodyDynamicsElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/RegularizationElements.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/OptimizationProblemElements.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/VariableHandler.h>

#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.h>

using namespace BipedalLocomotionControllers::WholeBodyControllers;
using namespace BipedalLocomotionControllers::OptimalControlUtilities;
using namespace BipedalLocomotionControllers::ParametersHandler;

bool MomentumBasedControlHelper::addFeetTypeIdentifiers(IParametersHandler::shared_ptr handler,
                                                        const FootType& type)
{
    bool isSwing = type == FootType::Swing;
    const std::string feetType = isSwing ? "swing" : "stance";
    auto& feetIdentifiers = isSwing ? m_swingFeetIdetrifiers : m_stanceFeetIdetrifiers;

    std::vector<std::string> feetVariablesName;
    if (!handler->getParameter(feetType + "_feet_name", feetVariablesName, GenericContainer::VectorResizeMode::Resizable))
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetTypeIdentifiers] Unable to find the "
                  << feetType << "feet names" << std::endl;
        return false;
    }

    std::vector<std::string> feetFramesName;
    if (!handler->getParameter(feetType + "_feet_frame", feetFramesName, GenericContainer::VectorResizeMode::Resizable))
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetTypeIdentifiers] Unable to find the "
                  << feetType << "feet names" << std::endl;
        return false;
    }

    if (feetFramesName.size() != feetVariablesName.size())
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetTypeIdentifiers] The number of identifiers "
                     "is different. For the "
                  << feetType << " feet" << std::endl;
        return false;
    }

    // add the identifiers
    for (std::size_t i = 0; i < feetFramesName.size(); i++)
        feetIdentifiers.emplace_back(feetVariablesName[i], feetFramesName[i]);

    return true;
}

bool MomentumBasedControlHelper::addFeetIdentifiers(IParametersHandler::shared_ptr handler)
{
    if (handler->isEmpty())
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetIdentifiers] The handler is empty. "
                     "Unable to retrieve the parameters related to stance and swing feet"
                  << std::endl;
        return false;
    }

    if (!addFeetTypeIdentifiers(handler, FootType::Swing))
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetIdentifiers] Unable to add the Swing feet "
                     "identifiers"
                  << std::endl;
        return false;
    }

    if (!addFeetTypeIdentifiers(handler, FootType::Stance))
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetIdentifiers] Unable to add the Stance "
                     "feet identifiers"
                  << std::endl;
        return false;
    }

    return true;
}

bool MomentumBasedControlHelper::addLinearMomentumElement(IParametersHandler::shared_ptr handler)
{


    // the frame in contact are two (left and right foot)
    std::vector<FrameInContact<std::string, std::string>> framesInContact;

    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    // gains and weights
    // Gain
    iDynTree::Vector3 kp, kd, ki;
    if (!handler->getParameter("kp", kp) || !handler->getParameter("kd", kd) || !handler->getParameter("ki", ki))
    {
        std::cerr << "[MomentumBasedControlHelper::addCentroidalLinearMomentumElement] Unable to "
                     "get the gains.";
        return false;
    }

    // create the pd controller
    PIDController<iDynTree::Vector3> pidController(kd, kp, ki);

    m_centroidalLinearMomentumElement
        = std::make_unique<CentroidalLinearMomentumRateOfChangeElement>(m_kinDyn,
                                                                        pidController,
                                                                        m_variableHandler,
                                                                        framesInContact);

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight(3);
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedControlHelper::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }

        m_costFunction->addCostFunction(m_centroidalLinearMomentumElement.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        "centroidal_linear_momentum");

    } else
        m_constraints->addConstraint(m_centroidalLinearMomentumElement.get());

    return true;
}

bool MomentumBasedControlHelper::addAngularMomentumElement(IParametersHandler::shared_ptr handler)
{
    // get all the required parameters


    // the frame in contact are two (left and right foot)
    std::vector<FrameInContact<std::string, std::string>> framesInContact;

    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    // gains and weights
    // Gain
    iDynTree::Vector3 kp, kd, ki;
    if (!handler->getParameter("kp", kp) || !handler->getParameter("kd", kd)
        || !handler->getParameter("ki", ki))
    {
        std::cerr << "[MomentumBasedControlHelper::addCentroidalAngulatMomentumElement] Unable to "
                     "get the gains.";
        return false;
    }

    // create the pid controller
    PIDController<iDynTree::Vector3> pidController(kd, kp, ki);

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedControlHelper::addCentroidalAngulatMomentumElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }


    m_centroidalAngularMomentumElement
        = std::make_unique<CentroidalAngularMomentumRateOfChangeElement>(m_kinDyn,
                                                                         pidController,
                                                                         m_variableHandler,
                                                                         framesInContact,
                                                                         samplingTime);

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight(3);
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedControlHelper::addCentroidalAngularMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }

        m_costFunction->addCostFunction(m_centroidalAngularMomentumElement.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        "centroidal_angular_momentum");

    } else
        m_constraints->addConstraint(m_centroidalAngularMomentumElement.get());

    return true;
}

bool MomentumBasedControlHelper::addAngularMomentumBounds(IParametersHandler::shared_ptr handler)
{
    // the frame in contact are two (left and right foot)
    std::vector<FrameInContact<std::string, std::string>> framesInContact;

    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedControlHelper::addCentroidalAngulatMomentumBounds] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }

    iDynTree::Vector3 upperBound, lowerBound;
    if (!handler->getParameter("upper_bound", upperBound))
    {
        std::cerr << "[MomentumBasedControlHelper::addCentroidalAngulatMomentumBounds] Unable to "
                     "find the angular momentum upperbound"
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("lower_bound", lowerBound))
    {
        std::cerr << "[MomentumBasedControlHelper::addCentroidalAngulatMomentumBounds] Unable to "
                     "find the angular momentum lowerbound"
                  << std::endl;
        return false;
    }

    m_centroidalAngularMomentumBound
        = std::make_unique<CentroidalAngularMomentumRateOfChangeBounds>(m_kinDyn,
                                                                        m_variableHandler,
                                                                        framesInContact,
                                                                        upperBound,
                                                                        lowerBound,
                                                                        samplingTime);

    m_constraints->addConstraint(m_centroidalAngularMomentumBound.get());
    return true;
}

bool MomentumBasedControlHelper::addSystemDynamicsElement(IParametersHandler::shared_ptr handler)
{
    // get the frames in contact name
    std::vector<FrameInContact<std::string, std::string>> framesInContact;
    bool isCompliantContact = true;
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        framesInContact.emplace_back(stanceFoot.identifierInVariableHandler(),
                                     stanceFoot.identifierInModel(),
                                     isCompliantContact);

    m_floatingBaseDynamics = std::make_unique<FloatingBaseDynamicsElement>(m_kinDyn,
                                                                           m_variableHandler,
                                                                           framesInContact);

    // initialize joint dynamics element
    VariableHandler tempVariableHandler(m_variableHandler);
    size_t jointAccelerationSize = m_variableHandler.getVariable("joint_accelerations").size;
    tempVariableHandler.addVariable("joint_torques", jointAccelerationSize);

    m_jointDynamics = std::make_unique<JointSpaceDynamicsElement>(m_kinDyn,
                                                                  tempVariableHandler,
                                                                  framesInContact);

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight",
                                   rawWeight,
                                   GenericContainer::VectorResizeMode::Resizable))
        {
            std::cerr << "[MomentumBasedControlHelper::addSystemDynamicsElement] Unable to get the "
                         "Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_floatingBaseDynamics.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        "floating_base_dynamics");
    } else
        m_constraints->addConstraint(m_floatingBaseDynamics.get());

    return true;
}
bool MomentumBasedControlHelper::addRegularizationElement(IParametersHandler::shared_ptr handler,
                                                          const std::string& label)
{
    if (m_regularizationElements.find(label) != m_regularizationElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addRegularizationElement] The element named "
                  << label << " has been already added" << std::endl;
        return false;
    }


    m_regularizationElements.emplace(label,
                                     std::make_unique<RegularizationElement>(m_kinDyn,
                                                                             m_variableHandler,
                                                                             label));
    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight, GenericContainer::VectorResizeMode::Resizable))
        {
            std::cerr << "[MomentumBasedControlHelper::addRegularizationElement] Unable to get the "
                         "Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_regularizationElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_regularization");
    } else
        m_constraints->addConstraint(m_regularizationElements.find(label)->second.get());

    return true;
}

bool MomentumBasedControlHelper::addRegularizationWithControlElement(IParametersHandler::shared_ptr handler,
                                                                     const std::string& label)
{
    if (m_regularizationWithControlElements.find(label) != m_regularizationWithControlElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addRegularizationWithControlElement] The "
                     "element named "
                  << label << " has been already added" << std::endl;
        return false;
    }

    // instantiate gains
    iDynTree::VectorDynSize kp, kd;
    if (!handler->getParameter("kp", kp, GenericContainer::VectorResizeMode::Resizable))
    {
        std::cerr << "[MomentumBasedControlHelper::addRegularizationWithControlElement] The Kp of "
                     "the "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    bool useDefaultKp = false;
    handler->getParameter("use_default_kd", useDefaultKp);
    if (useDefaultKp)
    {
        double scaling = 1.0;
        handler->getParameter("scaling", scaling);

        kd.resize(kp.size());
        iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
    } else
    {
        if (!handler->getParameter("kd", kd, GenericContainer::VectorResizeMode::Resizable))
        {
            std::cerr << "[MomentumBasedControlHelper::addRegularizationWithControlElement] The kd "
                         "cannot be found"
                      << std::endl;
            return false;
        }
    }
    LinearPD<iDynTree::VectorDynSize> pdController(kp, kd);

    m_regularizationWithControlElements.emplace(label,
                                                std::make_unique<RegularizationWithControlElement>(m_kinDyn,
                                                                                                   pdController,
                                                                                                   m_variableHandler,
                                                                                                   label));
    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight",
                                   rawWeight,
                                   GenericContainer::VectorResizeMode::Resizable))
        {
            std::cerr << "[MomentumBasedControlHelper::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_regularizationWithControlElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_regularization_with_constraints");
    } else
        m_constraints->addConstraint(m_regularizationWithControlElements.find(label)->second.get());

    return true;
}

bool MomentumBasedControlHelper::addJointValuesFeasibilityElement(IParametersHandler::shared_ptr handler,
                                                                  const iDynTree::VectorDynSize& maxJointsPosition,
                                                                  const iDynTree::VectorDynSize& minJointsPosition)
{
    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedControlHelper::addJointValuesFeasibilityElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }

    m_jointValuesFeasibilityElement = std::make_unique<JointValuesFeasibilityElement>(m_kinDyn,
                                                                                      m_variableHandler,
                                                                                      "joint_accelerations",
                                                                                      maxJointsPosition,
                                                                                      minJointsPosition,
                                                                                      samplingTime);

    m_constraints->addConstraint(m_jointValuesFeasibilityElement.get());

    return true;
}
bool MomentumBasedControlHelper::addContactWrenchFeasibilityElement(IParametersHandler::shared_ptr handler,
                                                                    const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    const auto& label = frame.identifierInVariableHandler();

    if (m_contactWrenchFeasibilityElements.find(label) != m_contactWrenchFeasibilityElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] This "
                     "element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedControlHelper::addJointValuesFeasibilityElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }

    double staticFrictionCoefficient;
    if (!handler->getParameter("static_friction_coefficient", staticFrictionCoefficient))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] "
                     "static_friction_coefficient of "
                         + label + " cannot be found"
                  << std::endl;
        return false;
    }

    int numberOfPoints;
    if (!handler->getParameter("number_of_points", numberOfPoints))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] "
                     "static_friction_coefficient of "
                         + label + " cannot be found"
                  << std::endl;
        return false;
    }

    double torsionalFrictionCoefficient;
    if (!handler->getParameter("torsional_friction_coefficient", torsionalFrictionCoefficient))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] "
                     "torsional_friction_coefficient of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    iDynTree::Vector2 footLimitsX;
    if (!handler->getParameter("foot_limits_x", footLimitsX))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] "
                     "foot_limits_x of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    iDynTree::Vector2 footLimitsY;
    if (!handler->getParameter("foot_limits_y", footLimitsY))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] "
                     "foot_limits_y of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    double minimalNormalForce;
    if (!handler->getParameter("minimal_normal_force", minimalNormalForce))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactWrenchFeasibilityElement] "
                     "foot_limits_y of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    m_contactWrenchFeasibilityElements.insert(
        {label,
         std::make_unique<ContactWrenchRateOfChangeFeasibilityElement>(m_kinDyn,
                                                                       m_variableHandler,
                                                                       frame,
                                                                       numberOfPoints,
                                                                       staticFrictionCoefficient,
                                                                       torsionalFrictionCoefficient,
                                                                       minimalNormalForce,
                                                                       footLimitsX,
                                                                       footLimitsY,
                                                                       OsqpEigen::INFTY,
                                                                       samplingTime)});

    m_constraints->addConstraint(m_contactWrenchFeasibilityElements.find(label)->second.get());

    return true;
}

bool MomentumBasedControlHelper::addContactModelElement(IParametersHandler::shared_ptr handler,
                                                        const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    using namespace BipedalLocomotionControllers::ContactModels;

    const auto& label = frame.identifierInVariableHandler();

    if (m_contactModelElements.find(label) != m_contactModelElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addContactModelElement] This element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    FrameInContactWithContactModel<std::string, std::string> frameInContact;
    frameInContact.identifierInVariableHandler() = label;
    frameInContact.identifierInModel() = frame.identifierInModel();
    frameInContact.isInCompliantContact() = true;
    frameInContact.contactModel() = std::make_shared<ContinuousContactModel>();
    if(!frameInContact.contactModel()->initialize(handler))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactModelElement] Unable to initialize "
                     "the contact model."
                  << std::endl;
        return false;
    }

    m_contactModelElements.insert(
        {label,
         std::make_unique<ContactModelElement>(m_kinDyn, m_variableHandler, frameInContact)});

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedControlHelper::addContactModelElement] Unable to get the "
                         "Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_contactModelElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label);
    } else
        m_constraints->addConstraint(m_contactModelElements.find(label)->second.get());


    return true;
}

bool MomentumBasedControlHelper::initialize(IParametersHandler::weak_ptr handlerWeak,
                                            const std::string& controllerType,
                                            const iDynTree::VectorDynSize& maxJointsPosition,
                                            const iDynTree::VectorDynSize& minJointsPosition)
{

    auto handler = handlerWeak.lock();
    if (handler == nullptr)
    {
        std::cerr << "[MomentumBasedControlHelper::initialize] The handler has expired"
                  << std::endl;
        return false;
    }

    m_description = controllerType;

    auto feetIdentifiersOptions = handler->getGroup(controllerType).lock();
    if (feetIdentifiersOptions == nullptr)
    {
        std::cerr << "[MomentumBasedControlHelper::initialize] The feet identifiers options named "
                  << controllerType << " has been expired" << std::endl;
        return false;
    }

    if (!addFeetIdentifiers(feetIdentifiersOptions))
    {
        std::cerr << "[MomentumBasedControlHelper::initialize] Unable to load the feet identifiers"
                  << std::endl;
        return false;
    }

    initializeVariableHandler();

    double samplingTime;
    if(!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedControlHelper::initialize] Unable to find the sampling time"
                  << std::endl;
        return false;
    }

    auto linearMomentumOptions = handler->getGroup("CENTROIDAL_LINEAR_MOMENTUM");
    if (auto ptr = linearMomentumOptions.lock())
        if (!addLinearMomentumElement(ptr))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the linear "
                         "momentum element"
                      << std::endl;
            return false;
        }

    auto angularMomentumOptions = handler->getGroup("CENTROIDAL_ANGULAR_MOMENTUM");
    if(auto ptr = angularMomentumOptions.lock())
    {
        ptr->setParameter("sampling_time", samplingTime);
        if(!addAngularMomentumElement(ptr))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the angular "
                         "momentum element"
                      << std::endl;
            return false;
        }
    }

    auto angularMomentumBoundsOptions = handler->getGroup("CENTROIDAL_ANGULAR_MOMENTUM_BOUNDS");
    if(auto ptr = angularMomentumBoundsOptions.lock())
    {
        ptr->setParameter("sampling_time", samplingTime);
        if(!addAngularMomentumBounds(ptr))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the angular "
                         "momentum bounds"
                      << std::endl;
            return false;
        }
    }

    auto torsoOptions = handler->getGroup("TORSO");
    if (auto ptr = torsoOptions.lock())
    {
        OptimalControlUtilities::Frame<std::string, std::string> torsoIdentifiers;
        torsoIdentifiers.identifierInVariableHandler() = "torso";
        if(!ptr->getParameter("frame_name", torsoIdentifiers.identifierInModel()))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the find the torso frame name"
                      << std::endl;
            return false;
        }

        if (!addCartesianElement<OptimalControlUtilities::CartesianElementType::ORIENTATION>(ptr, torsoIdentifiers))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the torso element"
                      << std::endl;
            return false;
        }
    }
    auto systemDynamicsOptions = handler->getGroup("SYSTEM_DYNAMICS");
    if (auto ptr = systemDynamicsOptions.lock())
        if (!addSystemDynamicsElement(ptr))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the system dynamics"
                      << std::endl;
            return false;
        }

    auto jointRegularizationOptions = handler->getGroup("JOINT_REGULARIZATION");
    if (auto ptr = jointRegularizationOptions.lock())
        if (!addRegularizationWithControlElement(ptr, "joint_accelerations"))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the joint "
                         "regularization element"
                      << std::endl;
            return false;
        }

    for (const auto& identifier : m_stanceFeetIdetrifiers)
    {
        // We assume that the name of a group is composed by capital characters only
        std::string upperIdentifier = identifier.identifierInVariableHandler();
        std::transform(upperIdentifier.begin(),
                       upperIdentifier.end(),
                       upperIdentifier.begin(),
                       [](unsigned char c) { return std::toupper(c); });

        // add the regularization element
        auto stanceFootRegularizationRateOptions = handler->getGroup(upperIdentifier + "_WRENCH_RATE_REGULARIZATION");
        if (auto ptr = stanceFootRegularizationRateOptions.lock())
            if (!addRegularizationElement(ptr, identifier.identifierInVariableHandler()))
            {
                std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the wrench "
                             "regularization rate of change element for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }

        // add the regularization element
        auto stanceFootRegularizationOptions = handler->getGroup(upperIdentifier + "_WRENCH_REGULARIZATION");
        if (auto ptr = stanceFootRegularizationOptions.lock())
        {
            if (!addRegularizationWithControlElement(ptr, identifier.identifierInVariableHandler()))
            {
                std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the wrench "
                             "regularization element for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }

            // set the setpoint
            iDynTree::VectorDynSize dummyZero(6);
            iDynTree::VectorDynSize weight(6);
            dummyZero.zero();
            weight.zero();
            weight(2) = 9.81 * m_kinDyn->model().getTotalMass() / 2;
            m_regularizationWithControlElements[identifier.identifierInVariableHandler()]
                ->setReference(dummyZero, dummyZero, weight);
        }

        // add the wrench feasibility element
        auto stanceFootWrenchOptions = handler->getGroup(upperIdentifier + "_WRENCH_FEASIBILITY");
        if (auto ptr = stanceFootWrenchOptions.lock())
        {
            ptr->setParameter("sampling_time", samplingTime);

            if (!addContactWrenchFeasibilityElement(ptr, identifier))
            {
                std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the wrench "
                             "feasibility element for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }
        }

        // add the contact model
        auto stanceFootContactModelOptions = handler->getGroup(upperIdentifier + "_CONTACT_MODEL");
        if (auto ptr = stanceFootContactModelOptions.lock())
            if (!addContactModelElement(ptr, identifier))
            {
                std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the contact "
                             "model for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }
    }

    for (const auto& identifier : m_swingFeetIdetrifiers)
    {
        std::string upperIdentifier = identifier.identifierInVariableHandler();
        std::transform(upperIdentifier.begin(),
                       upperIdentifier.end(),
                       upperIdentifier.begin(),
                       [](unsigned char c) { return std::toupper(c); });

        // add the wrench feasibility element
        auto footControlTask = handler->getGroup(upperIdentifier + "_CONTROL_TASK");
        if (auto ptr = footControlTask.lock())
            if (!addCartesianElement<OptimalControlUtilities::CartesianElementType::POSE>(ptr,
                                                                                          identifier))
            {
                std::cerr << "[MomentumBasedControlHelper::initialize] Unable to add the control "
                             "task for the "
                          << identifier.identifierInVariableHandler() << std::endl;
                return false;
            }
    }

    auto jointAccelerationRegularization = handler->getGroup("JOINT_ACCELERATION_REGULARIZATION");
    if (auto ptr = jointAccelerationRegularization.lock())
        if (!addRegularizationElement(ptr, "joint_accelerations"))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to load the joint "
                         "regularization element"
                      << std::endl;
            return false;
        }

    auto baseAccelerationRegularization = handler->getGroup("BASE_ACCELERATION_REGULARIZATION");
    if (auto ptr = baseAccelerationRegularization.lock())
        if (!addRegularizationElement(ptr, "base_acceleration"))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to load the base "
                         "regularization element"
                      << std::endl;
            return false;
        }

    auto jointFeasibilityOptions = handler->getGroup("JOINT_VALUES_FEASIBILITY");
    if (auto ptr = jointFeasibilityOptions.lock())
    {
        ptr->setParameter("sampling_time", samplingTime);
        if (!addJointValuesFeasibilityElement(ptr, maxJointsPosition, minJointsPosition))
        {
            std::cerr << "[MomentumBasedControlHelper::initialize] Unable to load the joint values "
                         "feasibility element"
                      << std::endl;
            return false;
        }
    }

    initialzeSolver();
    printElements();

    return true;
}

void MomentumBasedControlHelper::initializeVariableHandler()
{
    // instantiate variable handler and initialize the variables
    m_variableHandler.addVariable("base_acceleration", 6);
    m_variableHandler.addVariable("joint_accelerations", m_kinDyn->model().getNrOfDOFs());

    // add the stance feet in the optimization problem
    for (const auto& stanceFoot : m_stanceFeetIdetrifiers)
        m_variableHandler.addVariable(stanceFoot.identifierInVariableHandler(), 6);


    // initialize the constraints
    m_constraints = std::make_unique<Constraints>(m_variableHandler);

    // initialize the cost function
    m_costFunction = std::make_unique<CostFunction>(m_variableHandler);
}

void MomentumBasedControlHelper::setVerbosity(bool isVerbose) noexcept
{
    m_isVerbose = isVerbose;
}

MomentumBasedControlHelper::MomentumBasedControlHelper(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : m_kinDyn(kinDyn)
{
}

void MomentumBasedControlHelper::printElements() const
{
    // print useful information
    std::cout << m_description << std::endl;
    std::cout << "Cost Functions" << std::endl;
    std::cout << "---------------------" << std::endl;
    for (const auto& cost : m_costFunction->getCostFunctions())
        std::cout << cost.second.element->getName() << std::endl;
    std::cout << "---------------------" << std::endl;

    std::cout << "Equality Constraints" << std::endl;
    std::cout << "---------------------" << std::endl;
    for (const auto& constraint : m_constraints->getEqualityConstraints())
        std::cout << constraint.element->getName() << std::endl;
    std::cout << "---------------------" << std::endl;

    std::cout << "Inequality Constraints" << std::endl;
    std::cout << "----------------------" << std::endl;
    for (const auto& constraint : m_constraints->getInequalityConstraints())
        std::cout << constraint.element->getName() << std::endl;
    std::cout << "---------------------" << std::endl;
}

void MomentumBasedControlHelper::initialzeSolver()
{
    // initialize the optimization problem
    m_solver = std::make_unique<OsqpEigen::Solver>();
    m_solver->data()->setNumberOfVariables(m_variableHandler.getNumberOfVariables());
    m_solver->data()->setNumberOfConstraints(m_constraints->getNumberOfConstraints());

    m_solver->settings()->setVerbosity(false);
    m_solver->settings()->setLinearSystemSolver(0);
    m_solver->settings()->setMaxIteraction(100000);
    m_solver->settings()->setPolish(false);
};

bool MomentumBasedControlHelper::solve()
{
    auto costElements = m_costFunction->getElements();
    auto bounds = m_constraints->getBounds();

    Eigen::SparseMatrix<double> hessianSparse
        = iDynTree::toEigen(costElements.hessian()).sparseView();

    Eigen::SparseMatrix<double> constraintSparse
        = iDynTree::toEigen(m_constraints->getConstraintMatrix()).sparseView();

    iDynTree::VectorDynSize& gradient = costElements.gradient();
    if (m_solver->isInitialized())
    {
        // update matrices hessian matrix
        // TODO do it in a smart way

        if (!m_solver->updateHessianMatrix(hessianSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the hessian"
                      << std::endl;
            return false;
        }

        if (!m_solver->updateGradient(iDynTree::toEigen(gradient)))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the gradient"
                      << std::endl;
            return false;
        }

        if (!m_solver->updateLinearConstraintsMatrix(constraintSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the linear "
                         "constraint matrix"
                      << std::endl;
            return false;
        }

        if (!m_solver->updateBounds(iDynTree::toEigen(bounds.lowerBound()),
                                    iDynTree::toEigen(bounds.upperBound())))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to update the linear "
                         "constraint matrix"
                      << std::endl;
            return false;
        }
    } else
    {
        if (!m_solver->data()->setHessianMatrix(hessianSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the hessian the first "
                         "time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setGradient(iDynTree::toEigen(gradient)))
        {

            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the gradient the first "
                         "time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setLinearConstraintsMatrix(constraintSparse))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the linear constraint "
                         "matrix the first time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setLowerBound(iDynTree::toEigen(bounds.lowerBound())))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the lower bound "
                         "vector the first time"
                      << std::endl;
            return false;
        }

        if (!m_solver->data()->setUpperBound(iDynTree::toEigen(bounds.upperBound())))
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to set the upper bound "
                         "vector the first time"
                      << std::endl;
            return false;
        }

        if (!m_solver->initSolver())
        {
            std::cerr << "[MomentumBasedControlHelper::solve] Unable to initialize the software"
                      << std::endl;
            return false;
        }
    }

    if (!m_solver->solve())
    {
        std::cerr << "[MomentumBasedControlHelper::solve] Unable to solve the problem" << std::endl;
        return false;
    }

    return true;
}

void MomentumBasedControlHelper::setCentroidalMomentumReference(
    const iDynTree::SpatialForceVector& momentumSecondDerivative,
    const iDynTree::SpatialForceVector& momentumDerivative,
    const iDynTree::SpatialForceVector& momentum,
    const iDynTree::Vector3& centerOfMass)
{
    if (m_centroidalLinearMomentumElement != nullptr)
        m_centroidalLinearMomentumElement->setReference(momentumSecondDerivative.getLinearVec3(),
                                                        momentumDerivative.getLinearVec3(),
                                                        momentum.getLinearVec3(),
                                                        centerOfMass);


    if (m_centroidalAngularMomentumElement != nullptr)
        m_centroidalAngularMomentumElement->setReference(momentumSecondDerivative.getAngularVec3(),
                                                         momentumDerivative.getAngularVec3(),
                                                         momentum.getAngularVec3());
}

bool MomentumBasedControlHelper::setMeasuredContactWrench(const std::unordered_map<std::string, iDynTree::Wrench>& contactWrenches)
{
    std::vector<iDynTree::LinearForceVector3> contactForces;

    iDynTree::VectorDynSize dummy(6);
    dummy.zero();

    for (const auto& contactWrench : contactWrenches)
    {
        auto contactWrenchElement = m_contactWrenchFeasibilityElements.find(contactWrench.first);
        if(contactWrenchElement != m_contactWrenchFeasibilityElements.end())
            contactWrenchElement->second->setContactWrench(contactWrench.second);

        iDynTree::VectorDynSize wrench(6);
        iDynTree::toEigen(wrench) = iDynTree::toEigen(contactWrench.second.asVector());
        auto contactWrenchRegularizationElement = m_regularizationWithControlElements.find(contactWrench.first);
        if (contactWrenchRegularizationElement != m_regularizationWithControlElements.end())
            contactWrenchRegularizationElement->second->setState(dummy, wrench);
    }

    if (m_jointDynamics != nullptr)
        if (!m_jointDynamics->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the joint dynamics"
                      << std::endl;
            return false;
        }

    if (m_floatingBaseDynamics != nullptr)
        if (!m_floatingBaseDynamics->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the floating base dynamics"
                      << std::endl;
            return false;
        }

    if (m_centroidalLinearMomentumElement != nullptr)
        if (!m_centroidalLinearMomentumElement->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the linear momentum element"
                      << std::endl;
            return false;
        }

    if (m_centroidalAngularMomentumElement != nullptr)
        if (!m_centroidalAngularMomentumElement->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the angular momentum element"
                      << std::endl;
            return false;
        }

    if (m_centroidalAngularMomentumBound != nullptr)
        if (!m_centroidalAngularMomentumBound->setMeasuredContactWrenches(contactWrenches))
        {
            std::cerr << "[MomentumBasedControlHelper::setMeasuredContactWrench] Unable to set the "
                         "measured contact wrench in the angular momentum bounds"
                      << std::endl;
            return false;
        }

    return true;
}

void MomentumBasedControlHelper::setContactState(const std::string& name,
                                                 bool isInContact,
                                                 const iDynTree::Transform& desiredFootPose)
{
    auto contactModel = m_contactModelElements.find(name);
    if (contactModel != m_contactModelElements.end())
        contactModel->second->setContactState(isInContact, desiredFootPose);
}

void MomentumBasedControlHelper::setFootUpperBoundNormalForce(const std::string& name, const double& force)
{
    auto element = m_contactWrenchFeasibilityElements.find(name);
    if (element != m_contactWrenchFeasibilityElements.end())
        element->second->setUpperBoundNormalForce(force);
}

void MomentumBasedControlHelper::setRotationReference(const iDynTree::Vector3& acceleration,
                                                      const iDynTree::Vector3& velocity,
                                                      const iDynTree::Rotation& rotation,
                                                      const std::string& name)
{
    auto element = m_orientationElements.find(name);
    if (element != m_orientationElements.end())
        element->second->setReference(acceleration, velocity, rotation);
}

void MomentumBasedControlHelper::setTransformationReference(const iDynTree::SpatialAcc& acceleration,
                                                            const iDynTree::Twist& twist,
                                                            const iDynTree::Transform& transform,
                                                            const std::string& name)
{
    auto element = m_cartesianElements.find(name);
    if (element != m_cartesianElements.end())
        element->second->setReference(acceleration, twist, transform);
}

void MomentumBasedControlHelper::setRegularizationReference(const iDynTree::VectorDynSize& acceleration,
                                                            const iDynTree::VectorDynSize& velocity,
                                                            const iDynTree::VectorDynSize& position,
                                                            const std::string& name)
{
    const auto& element = m_regularizationWithControlElements.find(name);
    if (element != m_regularizationWithControlElements.end())
        element->second->setReference(acceleration, velocity, position);
}

void MomentumBasedControlHelper::setJointState(
    const iDynTree::VectorDynSize& velocity, const iDynTree::VectorDynSize& position)
{
    const auto& element = m_regularizationWithControlElements.find("joint_accelerations");

    if (element != m_regularizationWithControlElements.end())
        element->second->setState(velocity, position);
}

iDynTree::VectorDynSize MomentumBasedControlHelper::getDesiredAcceleration()
{
    const size_t numberOfJoints = m_variableHandler.getVariable("joint_accelerations").size;
    iDynTree::VectorDynSize robotAcceleration(numberOfJoints + 6);

    iDynTree::toEigen(robotAcceleration) = m_solver->getSolution().head(numberOfJoints + 6);

    return robotAcceleration;
}

iDynTree::VectorDynSize MomentumBasedControlHelper::getDesiredTorques()
{
    const size_t numberOfJoints = m_variableHandler.getVariable("joint_accelerations").size;
    iDynTree::VectorDynSize jointTorques(numberOfJoints);

    iDynTree::toEigen(jointTorques) = iDynTree::toEigen(m_jointDynamics->getB())
          - iDynTree::toEigen(m_jointDynamics->getA()).leftCols(numberOfJoints + 6)
                * m_solver->getSolution().head(numberOfJoints + 6);

    return jointTorques;
}

iDynTree::Vector6 MomentumBasedControlHelper::getLeftFootWrenchRateOfChange() const
{
    auto index = m_variableHandler.getVariable("left_foot");
    iDynTree::Vector6 forceRateOfChange;
    iDynTree::toEigen(forceRateOfChange) = m_solver->getSolution().segment(index.offset, index.size);

    return forceRateOfChange;
}

iDynTree::Vector6 MomentumBasedControlHelper::getRightFootWrenchRateOfChange() const
{
    auto index = m_variableHandler.getVariable("right_foot");
    iDynTree::Vector6 forceRateOfChange;
    iDynTree::toEigen(forceRateOfChange) = m_solver->getSolution().segment(index.offset, index.size);

    return forceRateOfChange;
}
