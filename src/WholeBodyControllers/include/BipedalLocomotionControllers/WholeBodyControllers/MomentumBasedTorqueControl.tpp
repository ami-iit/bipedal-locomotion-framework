/**
 * @file MomentumBasedTorqueControl.tpp
 * @authors Giulio Romualdi <giulio.romualdi@iit.it>
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 * @date 2020
 */

#include <BipedalLocomotionControllers/ParametersHandler/IParametersHandler.h>
#include <BipedalLocomotionControllers/WholeBodyControllers/MomentumBasedTorqueControlWithCompliantContacts.h>
#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>


namespace BipedalLocomotionControllers
{
namespace WholeBodyControllers
{
template <class T>
bool MomentumBasedTorqueControl::addLinearMomentumElement(std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    // get all the required parameters
    using namespace OptimalControlUtilities;
    // the frame in contact are two (left and right foot)
    std::vector<FrameInContact<std::string, std::string>> framesInContact(2);

    bool outcome = true;

    // left foot
    framesInContact[0].identifierInVariableHandler() = "left_foot";

    outcome = handler->getParameter("left_foot_frame", framesInContact[0].identifierInModel());
    if(!outcome)
    {
        std::cerr << "[MomentumBasedTorqueControl::addLinearMomentumElement] Unable to find the "
                     "parameter left_foot_frame"
                  << std::endl;
        return outcome;
    }
    framesInContact[0].isInCompliantContact() = true;

    framesInContact[1].identifierInVariableHandler() = "right_foot";
    outcome = handler->getParameter("right_foot_frame", framesInContact[0].identifierInModel());
    if(!outcome)
    {
        std::cerr << "[MomentumBasedTorqueControl::addLinearMomentumElement] Unable to find the "
                     "parameter right_foot_frame"
                  << std::endl;
        return outcome;
    }
    framesInContact[1].isInCompliantContact() = true;

    // gains and weights
    // Gain
    iDynTree::Vector3 kp, kd, ki;
    if (!handler->getParameter("kp", kp) || !handler->getParameter("kd", kd) || !handler->getParameter("ki", ki))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable to "
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
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
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

template <class T>
bool MomentumBasedTorqueControl::addOrientationElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                       const std::string& label)
{

    // get all the required parameters
    using namespace OptimalControlUtilities;


    auto type = CartesianElementType::ORIENTATION;
    auto axis = CartesianElementAxisName::ALL;

    std::string frameInModel;
    if (!handler->getParameter("frame_name", frameInModel))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The frame_name cannot be "
                     "found"
                  << std::endl;
        return false;
    }

    double kp, kd, c0;
    if (!handler->getParameter("kp", kp))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The kp cannot be "
                     "found"
                  << std::endl;
        return false;
    }
    bool useDefaultKp = false;
    handler->getParameter("use_default_kd", useDefaultKp);
    if (useDefaultKp)
    {
        double scaling = 1.0;
        handler->getParameter("scaling", scaling);
        kd = 2 / scaling * std::sqrt(kp);
    } else
    {
        if (!handler->getParameter("kd", kd))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The kd cannot be "
                     "found"
                  << std::endl;
            return false;
        }
    }

    if (!handler->getParameter("c0", c0))
    {
        std::cerr << "[MomentumBasedTorqueControl::addCartesianElement] The c0 cannot be "
                     "found"
                  << std::endl;
        return false;
    }

    auto pdController = std::make_unique<OrientationPD>();
    pdController->setGains(c0, kd, kp);

    if (m_cartesianElements.find(label) != m_cartesianElements.end())
        throw std::runtime_error("[TaskBasedTorqueControl::addCartesianElement] The element named "
                                 + label + " has been already added.");

    m_cartesianElements.emplace(label,
                                std::make_unique<CartesianElement<CartesianElementType::ORIENTATION>>(m_kinDyn,
                                                                                                      std::move(pdController),
                                                                                                      m_variableHandler,
                                                                                                      frameInModel));

    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_cartesianElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label + "_cartesian_element");
    } else
        m_constraints->addConstraint(m_cartesianElements.find(label)->second.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addSystemDynamicsElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler)
{
    using namespace OptimalControlUtilities;

    // get the frames in contact name
    std::vector<FrameInContact<std::string, std::string>> framesInContact;
    std::string frameName;
    bool isCompliantContact = true;

    if (!handler->getParameter("left_foot_frame", frameName))
    {
        std::cerr << "[MomentumBasedTorqueControl::addFloatingBaseDynamicsElement] Unable to find "
                     "left_foot_frame"
                  << std::endl;
        return false;
    }
    framesInContact.emplace_back("left_foot", frameName, isCompliantContact);

    if (!handler->getParameter("right_foot_frame", frameName))
    {
        std::cerr << "[MomentumBasedTorqueControl::addFloatingBaseDynamicsElement] Unable to find "
                     "right_foot_frame"
                  << std::endl;
        return false;
    }
    framesInContact.emplace_back("right_foot", frameName, isCompliantContact);

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
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
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
bool MomentumBasedTorqueControl::addRegularizationWithControlElement( std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                      const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    if (m_regularizationWithControlElements.find(label) != m_regularizationWithControlElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addRegularizationWithControlElement] The "
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
        std::cerr << "[MomentumBasedTorqueControl::addRegularizationWithControlElement] The Kp of "
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
            std::cerr << "[MomentumBasedTorqueControl::addRegularizationWithControlElement] The kd "
                         "cannot be found"
                      << std::endl;
            return outcome;
        }
    }
    auto pdController = std::make_unique<LinearPD<iDynTree::VectorDynSize>>(kp, kd);

    m_regularizationWithControlElements.emplace(label,
                                                std::make_unique<RegularizationWithControlElement>(m_kinDyn,
                                                                                                   std::move(pdController),
                                                                                                   m_variableHandler,
                                                                                                   label));
    bool asConstraint = false;
    handler->getParameter("as_constraint", asConstraint);
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight;
        if (!handler->getParameter("weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::addCentroidalLinearMomentumElement] Unable "
                         "to get the Weight."
                      << std::endl;
            return false;
        }
        m_costFunction->addCostFunction(m_regularizationWithControlElements.find(label)->second.get(),
                                        Weight<iDynTree::VectorDynSize>(rawWeight),
                                        label);
    } else
        m_constraints->addConstraint(m_regularizationWithControlElements.find(label)->second.get());

    return true;
}

template <class T>
bool MomentumBasedTorqueControl::addJointValuesFeasibilityElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                  const iDynTree::VectorDynSize& maxJointsPosition,
                                                                  const iDynTree::VectorDynSize& minJointsPosition)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::addJointValuesFeasibilityElement] Unable to "
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
bool MomentumBasedTorqueControl::addContactWrenchFeasibilityElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                                    const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;

    if (m_contactWrenchFeasibilityElements.find(label) != m_contactWrenchFeasibilityElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] This "
                     "element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    Frame<std::string, std::string> frameInContact;
    frameInContact.identifierInVariableHandler() = label;

    double samplingTime;
    if (!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::addJointValuesFeasibilityElement] Unable to "
                     "find the sampling time"
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("frame_name", frameInContact.identifierInModel()))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] The  cannot "
                     "be found"
                  << std::endl;
        return false;
    }

    double staticFrictionCoefficient;
    if (!handler->getParameter("static_friction_coefficient", staticFrictionCoefficient))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "static_friction_coefficient of "
                         + label + " cannot be found"
                  << std::endl;
        return false;
    }

    int numberOfPoints;
    if (!handler->getParameter("number_of_points", numberOfPoints))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "static_friction_coefficient of "
                         + label + " cannot be found"
                  << std::endl;
        return false;
    }

    double torsionalFrictionCoefficient;
    if (!handler->getParameter("torsional_friction_coefficient", torsionalFrictionCoefficient))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "torsional_friction_coefficient of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    iDynTree::Vector2 footLimitsX;
    if (!handler->getParameter("foot_limits_x", footLimitsX))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "foot_limits_x of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    iDynTree::Vector2 footLimitsY;
    if (!handler->getParameter("foot_limits_y", footLimitsY))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "foot_limits_y of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    double minimalNormalForce;
    if (!handler->getParameter("minimal_normal_force", minimalNormalForce))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactWrenchFeasibilityElement] "
                     "foot_limits_y of "
                  << label << " cannot be found" << std::endl;
        return false;
    }

    m_contactWrenchFeasibilityElements.insert(
        {label,
         std::make_unique<ContactWrenchRateOfChangeFeasibilityElement>(m_kinDyn,
                                                                       m_variableHandler,
                                                                       frameInContact,
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
bool MomentumBasedTorqueControl::addContactModelElement(unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                                        const std::string& label)
{
    using namespace BipedalLocomotionControllers::OptimalControlUtilities;
    using namespace BipedalLocomotionControllers::ContactModels;

    if (m_contactModelElements.find(label) != m_contactModelElements.end())
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] This element named "
                  << label << "has been already added" << std::endl;
        return false;
    }

    // get contact models parameters
    double length, width, springCoeff, damperCoeff;
    if (!handler->getParameter("length", length) || !handler->getParameter("width", width)
        || !handler->getParameter("spring_coeff", springCoeff)
        || !handler->getParameter("damper_coeff", damperCoeff))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the "
            "contact parameters." << std::endl;
        return false;
    }
    const std::unordered_map<std::string, std::any> parameters({{"length", length},
                                                                {"width", width},
                                                                {"spring_coeff", springCoeff},
                                                                {"damper_coeff", damperCoeff}});

    FrameInContactWithContactModel<std::string, std::string> frameInContact;
    frameInContact.identifierInVariableHandler() = label;

    if (!handler->getParameter("frame_name", frameInContact.identifierInModel()))
    {
        std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the "
                     "frame name."
                  << std::endl;
        return false;
    }
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
            std::cerr << "[MomentumBasedTorqueControl::addContactModelElement] Unable to get the "
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
bool MomentumBasedTorqueControl::initialize(std::unique_ptr<ParametersHandler::IParametersHandler<T>> handler,
                                            const iDynTree::VectorDynSize& maxJointsPosition,
                                            const iDynTree::VectorDynSize& minJointsPosition)
{
    double samplingTime;
    if(!handler->getParameter("sampling_time", samplingTime))
    {
        std::cerr << "[MomentumBasedTorqueControl::initialize] Unable to find the sampling time"
                  << std::endl;
        return false;
    }
    addLinearMomentumElement(handler->getGroup("CENTROIDAL_LINEAR_MOMENTUM"));
    addOrientationElement(handler->getGroup("TORSO"), "torso");
    addSystemDynamicsElement(handler->getGroup("SYSTEM_DYNAMICS"));
    addRegularizationWithControlElement(handler->getGroup("JOINT_REGULARIZATION"),
                                        "joint_accelerations");

    auto jointFeasibilityOptions = handler->getGroup("JOINT_FEASIBILITY");
    jointFeasibilityOptions->setParameter("sampling_time", samplingTime);
    addJointValuesFeasibilityElement(std::move(jointFeasibilityOptions),
                                     maxJointsPosition,
                                     minJointsPosition);

    auto leftWrenchOptions = handler->getGroup("LEFT_WRENCH_FEASIBILITY");
    leftWrenchOptions->setParameter("sampling_time", samplingTime);
    addContactWrenchFeasibilityElement(std::move(leftWrenchOptions), "left_foot");

    auto rightWrenchOptions = handler->getGroup("RIGHT_WRENCH_FEASIBILITY");
    rightWrenchOptions->setParameter("sampling_time", samplingTime);
    addContactWrenchFeasibilityElement(std::move(rightWrenchOptions), "right_foot");

    addContactModelElement(handler->getGroup("LEFT_CONTACT_MODEL"), "left_foot");
    addContactModelElement(handler->getGroup("RIGHT_CONTACT_MODEL"), "right_foot");

    this->initialzeSolver();
    this->printElements();

    return true;
}

} // namespace WholeBodyControllers
} // namespace BipedalLocomotionControllers
