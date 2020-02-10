/**
 * @file MomentumBasedTorqueControl.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Frame.h>
#include <BipedalLocomotionControllers/OptimalControlUtilities/Weight.h>
#include <BipedalLocomotionControllers/WholeBodyControllersYarpBindings/MomentumBasedTorqueControlWithCompliantContacts.h>
#include <BipedalLocomotionControllers/YarpUtilities/Helper.h>

using namespace BipedalLocomotionControllers::WholeBodyControllersYarpBindings;
using namespace BipedalLocomotionControllers::OptimalControlUtilities;
using namespace BipedalLocomotionControllers::YarpUtilities;
using namespace BipedalLocomotionControllers;

bool MomentumBasedTorqueControl::addCentroidalLinearMomentumElement(
    const yarp::os::Searchable& config)
{
    // get contact models parameters
    double length, width, springCoeff, damperCoeff;
    if (!getElementFromSearchable(config, "length", length)
        || !getElementFromSearchable(config, "width", width)
        || !getElementFromSearchable(config, "spring_coeff", springCoeff)
        || !getElementFromSearchable(config, "damper_coeff", damperCoeff))
    {
        std::cerr << "[MomentumBasedTorqueControl::"
                     "addCentroidalLinearMomentumElement] Unable to get the contact parameters.";
        return false;
    }
    const std::unordered_map<std::string, std::any> parameters({{"length", length},
                                                                {"width", width},
                                                                {"spring_coeff", springCoeff},
                                                                {"damper_coeff", damperCoeff}});

    // the frame in contact are two (left and right foot)
    std::vector<FrameInContactWithContactModel<std::string, std::string>> framesInContact(2);

    // left foot
    framesInContact[0].identifierInVariableHandler() = "left_foot";
    getElementFromSearchable(config, "left_foot_frame", framesInContact[0].identifierInModel());
    framesInContact[0].isInCompliantContact() = true;
    framesInContact[0].contactModel()
        = std::make_shared<ContactModels::ContinuousContactModel>(parameters);

    framesInContact[1].identifierInVariableHandler() = "right_foot";
    getElementFromSearchable(config, "right_foot_frame", framesInContact[1].identifierInModel());
    framesInContact[1].isInCompliantContact() = true;
    framesInContact[1].contactModel()
        = std::make_shared<ContactModels::ContinuousContactModel>(parameters);

    // gains and weights
    // Gain
    iDynTree::Vector3 kp, kd;
    if (!getVectorFromSearchable(config, "kp", kp) || !getVectorFromSearchable(config, "kd", kd))
    {
        std::cerr << "[MomentumBasedTorqueControl::"
                     "addCentroidalLinearMomentumElement] Unable to get the gains.";
        return false;
    }

    // create the pd controller
    auto pdController = std::make_unique<LinearPD<iDynTree::Vector3>>(kp, kd);

    // Weight
    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    if (!asConstraint)
    {
        iDynTree::VectorDynSize rawWeight(3);
        if (!getVectorFromSearchable(config, "weight", rawWeight))
        {
            std::cerr << "[MomentumBasedTorqueControl::"
                         "addCentroidalLinearMomentumElement] Unable to get the Weight.";
            return false;
        }

        Weight<iDynTree::VectorDynSize> weight(rawWeight);
        WholeBodyControllers::MomentumBasedTorqueControl::
            addCentroidalLinearMomentumElement(framesInContact,
                                               std::move(pdController),
                                               asConstraint,
                                               weight);
    } else
    {
        WholeBodyControllers::MomentumBasedTorqueControl::
            addCentroidalLinearMomentumElement(framesInContact,
                                               std::move(pdController),
                                               asConstraint);
    }

    return true;
}

bool MomentumBasedTorqueControl::addOrientationElement(const yarp::os::Searchable& config,
                                                       const std::string& label)
{
    Frame<std::string, std::string> frameName;
    frameName.identifierInVariableHandler() = label;

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    iDynTree::VectorDynSize rawWeight;
    if (!asConstraint)
    {

        if (!YarpUtilities::getVectorFromSearchable(config, "weight", rawWeight))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] "
                                     "The weight of "
                                     + label + " cannot be found");
    }
    Weight<iDynTree::VectorDynSize> weight(rawWeight);

    auto type = CartesianElementType::POSE;
    auto axis = CartesianElementAxisName::X;
    if (!YarpUtilities::getElementFromSearchable(config,
                                                 "frame_name",
                                                 frameName.identifierInModel()))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The "
                                 "frame_name cannot be found");

    double kp, kd, c0;
    if (!YarpUtilities::getElementFromSearchable(config, "kp", kp))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The "
                                 "kp of "
                                 + label + " cannot be found");

    if (config.check("use_default_kd", yarp::os::Value("False")).asBool())
    {
        double scaling = config.check("scaling", yarp::os::Value(1.0)).asDouble();
        kd = 2 / scaling * std::sqrt(kp);
    } else if (!YarpUtilities::getElementFromSearchable(config, "kp", kp))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The "
                                 "kp of "
                                 + label + " cannot be found");

    if (!YarpUtilities::getElementFromSearchable(config, "c0", c0))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addCartesianElement] The "
                                 "c0 of the "
                                 + label + " cannot be found");
    auto pdController = std::make_unique<OrientationPD>();
    pdController->setGains(c0, kd, kp);

    WholeBodyControllers::MomentumBasedTorqueControl::addOrientationElement(frameName,
                                                                            std::move(pdController),
                                                                            asConstraint,
                                                                            weight);

    return true;
}

void MomentumBasedTorqueControl::addFloatingBaseDynamicsElement(const yarp::os::Searchable& config)
{
    // get the frames in contact name
    std::vector<FrameInContact<std::string, std::string>> framesInContact;
    std::string frameName;
    if(!YarpUtilities::getElementFromSearchable(config, "left_foot_frame", frameName))
        throw std::runtime_error("[MomentumBasedTorqueControl::addSystemDynamicsElement] Unable to "
                                 "find left_foot_frame");

    // if not find the parameters the contact is considered stiff
    bool isCompliantContact = config.check("is_left_foot_in_compliant_contact", yarp::os::Value(false)).asBool();
    framesInContact.emplace_back("left_foot", frameName, isCompliantContact);

    if(!YarpUtilities::getElementFromSearchable(config, "right_foot_frame", frameName))
        throw std::runtime_error("[MomentumBasedTorqueControl::addSystemDynamicsElement] Unable to "
                                 "find right_foot_frame");

    // if not find the parameters the contact is considered stiff
    isCompliantContact = config.check("is_right_foot_in_compliant_contact", yarp::os::Value(false)).asBool();
    framesInContact.emplace_back("right_foot", frameName, isCompliantContact);

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    iDynTree::VectorDynSize rawWeight;
    if (!asConstraint)
    {

        if (!YarpUtilities::getVectorFromSearchable(config, "weight", rawWeight))
            throw std::runtime_error("[MomentumBasedTorqueControl::addSystemdynamicsElement] "
                                     "The weight of the system dynamics element cannot be found");
    }
    Weight<iDynTree::VectorDynSize> weight(rawWeight);

    WholeBodyControllers::MomentumBasedTorqueControl::addFloatingBaseDynamicsElement(framesInContact,
                                                                                     asConstraint,
                                                                                     weight);
}

void MomentumBasedTorqueControl::addJointDynamicsElement(const yarp::os::Searchable& config)
{
    std::vector<FrameInContact<std::string, std::string>> framesInContact;
    std::string frameName;
    if(!YarpUtilities::getElementFromSearchable(config, "left_foot_frame", frameName))
        throw std::runtime_error("[MomentumBasedTorqueControl::addSystemDynamicsElement] Unable to "
                                 "find left_foot_frame");

    // if not find the parameters the contact is considered stiff
    bool isCompliantContact = config.check("is_left_foot_in_compliant_contact", yarp::os::Value(false)).asBool();
    framesInContact.emplace_back("left_foot", frameName, isCompliantContact);

    if(!YarpUtilities::getElementFromSearchable(config, "right_foot_frame", frameName))
        throw std::runtime_error("[MomentumBasedTorqueControl::addSystemDynamicsElement] Unable to "
                                 "find right_foot_frame");

    // if not find the parameters the contact is considered stiff
    isCompliantContact = config.check("is_right_foot_in_compliant_contact", yarp::os::Value(false)).asBool();
    framesInContact.emplace_back("right_foot", frameName, isCompliantContact);


    WholeBodyControllers::MomentumBasedTorqueControl::addJointDynamicsElement(framesInContact);
}

void MomentumBasedTorqueControl::addRegularizationWithControlElement(const yarp::os::Searchable& config,
                                                                     const std::string& label)
{

    // instantiate gains
    iDynTree::VectorDynSize kp(variableSize(label));
    iDynTree::VectorDynSize kd(variableSize(label));

    if(!YarpUtilities::getVectorFromSearchable(config, "kp", kp))
        throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement] The Kp of the "
                                 + label
                                 + " cannot be found");

    if(config.check("use_default_kd", yarp::os::Value("False")).asBool())
    {
        double scaling = config.check("scaling", yarp::os::Value(1.0)).asDouble();
        iDynTree::toEigen(kd) = 2 / scaling * iDynTree::toEigen(kp).array().sqrt();
    }
    else
        if(!YarpUtilities::getVectorFromSearchable(config, "kd", kd))
            throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addRegularizationWithControlElement] The Kd of the "
                                     + label
                                     + " cannot be found");

    auto controller = std::make_unique<LinearPD<iDynTree::VectorDynSize>>();

    bool asConstraint = config.check("as_constraint", yarp::os::Value(false)).asBool();
    iDynTree::VectorDynSize rawWeight(variableSize(label));
    if (!asConstraint)
    {

        if (!YarpUtilities::getVectorFromSearchable(config, "weight", rawWeight))
            throw std::runtime_error("[MomentumBasedTorqueControl::addSystemdynamicsElement] "
                                     "The weight of the system dynamics element cannot be found");
    }
    Weight<iDynTree::VectorDynSize> weight(rawWeight);

    WholeBodyControllers::MomentumBasedTorqueControl::addRegularizationWithControlElement(label,
                                                                                          std::move(controller),
                                                                                          asConstraint,
                                                                                          weight);
}

void MomentumBasedTorqueControl::addJointValuesFeasibilityElement(const yarp::os::Searchable &config,
                                                                  const iDynTree::VectorDynSize& maxJointsPosition,
                                                                  const iDynTree::VectorDynSize& minJointsPosition)
{
     double samplingTime;
     if(!YarpUtilities::getElementFromSearchable(config, "sampling_time", samplingTime))
         throw std::runtime_error("[TaskBasedTorqueControlYARPInterface::addJointValuesFeasibilityElement] Unable to find the sampling_time");

     WholeBodyControllers::MomentumBasedTorqueControl::addJointValuesFeasibilityElement("joint_accelerations",
                                                                                        maxJointsPosition,
                                                                                        minJointsPosition,
                                                                                        samplingTime);
 }

MomentumBasedTorqueControl::MomentumBasedTorqueControl(
    std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
    : WholeBodyControllers::MomentumBasedTorqueControl(kinDyn)
{
}
