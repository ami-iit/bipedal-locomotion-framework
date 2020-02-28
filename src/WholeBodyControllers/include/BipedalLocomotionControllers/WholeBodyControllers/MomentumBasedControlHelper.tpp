/**
 * @file MomentumBasedTorqueControl.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedControlHelper.h>
#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>


namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{

template <class T>
bool MomentumBasedControlHelper::addFeetTypeIdentifiers(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                        const FootType& type)
{
    bool isSwing = type == FootType::Swing;
    const std::string feetType = isSwing ? "swing" : "stance";
    auto& feetIdentifiers = isSwing ? m_swingFeetIdetrifiers : m_stanceFeetIdetrifiers;

    std::vector<std::string> feetVariablesName;
    if (!handler->getParameter(feetType + "_feet_name", feetVariablesName))
    {
        std::cerr << "[MomentumBasedControlHelper::addFeetTypeIdentifiers] Unable to find the "
                  << feetType << "feet names" << std::endl;
        return false;
    }

    std::vector<std::string> feetFramesName;
    if (!handler->getParameter(feetType + "_feet_frame", feetFramesName))
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

template <class T>
bool MomentumBasedControlHelper::addFeetIdentifiers(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    if(handler->isEmpty())
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

template <class T>
bool MomentumBasedControlHelper::addLinearMomentumElement(
    std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    using namespace OptimalControlUtilities;

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

template <typename T>
bool MomentumBasedControlHelper::addAngularMomentumElement(
    std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    // get all the required parameters
    using namespace OptimalControlUtilities;

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

template <OptimalControlUtilities::CartesianElementType type,
          class T>
bool MomentumBasedControlHelper::addCartesianElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler, const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    // get all the required parameters
    using namespace OptimalControlUtilities;

    const auto& label = frame.identifierInVariableHandler();

    if (m_cartesianElements.find(label) != m_cartesianElements.end()
        || m_orientationElements.find(label) != m_orientationElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The element named " << label
                  << " has been already added." << std::endl;
        return false;
    }

    // initialize the PD controller
    typename CartesianElement<type>::ControllerType pdController;

    // if the type is Pose or orientation
    double kpRotational, kdRotational, c0;
    if constexpr (type == CartesianElementType::ORIENTATION || type == CartesianElementType::POSE)
    {
        if (!handler->getParameter("kp_rotational", kpRotational))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kp cannot be "
                         "found"
                      << std::endl;
            return false;
        }
        bool useDefaultKp = false;
        handler->getParameter("use_default_kd_rotational", useDefaultKp);
        if (useDefaultKp)
        {
            double scaling = 1.0;
            handler->getParameter("scaling_rotational", scaling);
            kdRotational = 2 / scaling * std::sqrt(kpRotational);
        } else
        {
            if (!handler->getParameter("kd_rotational", kdRotational))
            {
                std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kd cannot be "
                             "found"
                          << std::endl;
                return false;
            }
        }

        if (!handler->getParameter("c0", c0))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The c0 cannot be "
                         "found"
                      << std::endl;
            return false;
        }
    }

    // if the type is position or pose
    iDynTree::Vector3 kpPosition, kdPosition;
    if constexpr (type == CartesianElementType::POSITION || type == CartesianElementType::POSE)
    {
        if (!handler->getParameter("kp_position", kpPosition))
        {
            std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kp cannot be "
                         "found"
                      << std::endl;
            return false;
        }
        bool useDefaultKp = false;
        handler->getParameter("use_default_kd_position", useDefaultKp);
        if (useDefaultKp)
        {
            double scaling = 1.0;
            handler->getParameter("scaling_linear", scaling);
            iDynTree::toEigen(kdPosition) = 2 / scaling * iDynTree::toEigen(kpPosition).array().sqrt();
        } else
        {
            if (!handler->getParameter("kd_position", kdPosition))
            {
                std::cerr << "[MomentumBasedControlHelper::addCartesianElement] The kd cannot be "
                             "found"
                          << std::endl;
                return false;
            }
        }
    }

    // set the gains for each type of PD
    if constexpr (type == CartesianElementType::POSITION)
        pdController.setGains(kpPosition, kdPosition);

    if constexpr (type == CartesianElementType::ORIENTATION)
        pdController.setGains(c0, kdRotational, kpRotational);

    if constexpr (type == CartesianElementType::POSE)
        pdController.setGains(kpPosition, kdPosition, c0, kdRotational, kpRotational);


    typename dictionary<unique_ptr<CartesianElement<type>>>::iterator element;

    // initialize the element
    if constexpr (type == CartesianElementType::ORIENTATION)
    {
        std::cerr << "labellllllllll orientation" << label << std::endl;
        m_orientationElements
            .emplace(label,
                     std::make_unique<CartesianElement<type>>(m_kinDyn,
                                                              pdController,
                                                              m_variableHandler,
                                                              frame.identifierInModel()));
        element = m_orientationElements.find(label);
    }
    else if constexpr (type == CartesianElementType::POSE)
    {

        std::cerr << "labellllllllll pose" << label << std::endl;

        m_cartesianElements
            .emplace(label,
                     std::make_unique<CartesianElement<type>>(m_kinDyn,
                                                              pdController,
                                                              m_variableHandler,
                                                              frame.identifierInModel()));

        element = m_cartesianElements.find(label);
    }

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedControlHelper::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(element->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_cartesian_element");
    } else
        m_constraints->addConstraint(element->second.get());

    return true;
}

template <class T>
bool MomentumBasedControlHelper::addSystemDynamicsElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    using namespace OptimalControlUtilities;

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
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedControlHelper::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
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


template <class T>
bool MomentumBasedControlHelper::addRegularizationElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                          const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

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
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedControlHelper::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
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

template <class T>
bool MomentumBasedControlHelper::addRegularizationWithControlElement( std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                      const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    if (m_regularizationWithControlElements.find(label) != m_regularizationWithControlElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addRegularizationWithControlElement] The "
                     "element named "
                  << label << " has been already added" << std::endl;
        return false;
    }

    // instantiate gains
    iDynTree::VectorDynSize kp, kd;
    bool outcome = true;
    outcome = handler->getParameter("kp", kp);

    if (!outcome)
    {
        std::cerr << "[MomentumBasedControlHelper::addRegularizationWithControlElement] The Kp of "
                     "the "
                  << label << " cannot be found" << std::endl;
        return outcome;
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
        outcome = handler->getParameter("kd", kd);
        if (!outcome)
        {
            std::cerr << "[MomentumBasedControlHelper::addRegularizationWithControlElement] The kd "
                         "cannot be found"
                      << std::endl;
            return outcome;
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
        if (!handler->getParameter("weight", rawWeight))
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

template <class T>
bool MomentumBasedControlHelper::addJointValuesFeasibilityElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                  const iDynTree::VectorDynSize& maxJointsPosition,
                                                                  const iDynTree::VectorDynSize& minJointsPosition)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

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

template <class T>
bool MomentumBasedControlHelper::addContactWrenchFeasibilityElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                    const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

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

template <class T>
bool MomentumBasedControlHelper::addContactModelElement(std::shared_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                        const OptimalControlUtilities::Frame<std::string, std::string>& frame)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;
    using namespace BipedalLocomotionControllers::ContactModels;

    const auto& label = frame.identifierInVariableHandler();

    if (m_contactModelElements.find(label) != m_contactModelElements.end())
    {
        std::cerr << "[MomentumBasedControlHelper::addContactModelElement] This element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    // get contact models parameters
    double length, width, springCoeff, damperCoeff;
    if (!handler->getParameter("length", length) || !handler->getParameter("width", width)
        || !handler->getParameter("spring_coeff", springCoeff)
        || !handler->getParameter("damper_coeff", damperCoeff))
    {
        std::cerr << "[MomentumBasedControlHelper::addContactModelElement] Unable to get the "
            "contact parameters." << std::endl;
        return false;
    }
    const std::unordered_map<std::string, std::any> parameters({{"length", length},
                                                                {"width", width},
                                                                {"spring_coeff", springCoeff},
                                                                {"damper_coeff", damperCoeff}});

    FrameInContactWithContactModel<std::string, std::string> frameInContact;
    frameInContact.identifierInVariableHandler() = label;
    frameInContact.identifierInModel() = frame.identifierInModel();
    frameInContact.isInCompliantContact() = true;
    frameInContact.contactModel() = std::make_shared<ContinuousContactModel>(parameters);

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

template<class T>
bool MomentumBasedControlHelper::initialize(std::weak_ptr<ParametersHandler::IParametersHandler<T>> handlerWeak,
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

        if (!addCartesianElement<OptimalControlUtilities::CartesianElementType::ORIENTATION,T>(ptr, torsoIdentifiers))
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

} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers
