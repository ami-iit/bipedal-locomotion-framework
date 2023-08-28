/**
 * @file StableCentroidalMPC.cpp
 * @copyright 2023 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <chrono>
#include <string>
#include <unordered_map>

#include <casadi/casadi.hpp>

#include <BipedalLocomotion/Contacts/Contact.h>
#include <BipedalLocomotion/Conversions/CasadiConversions.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/ReducedModelControllers/StableCentroidalMPC.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::Contacts;


struct StableCentroidalMPC::Impl
{
    casadi::Opti opti; /**< CasADi opti stack */
    casadi::Function controller;
    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};

    CentroidalMPCOutput output;
    Contacts::ContactPhaseList contactPhaseList;
    Math::LinearizedFrictionCone frictionCone;

    enum class FSM
    {
        Idle,
        Initialized,
        OutputValid,
        OutputInvalid,
    };

    FSM fsm{FSM::Idle};

    struct StableConstants
    {
        const double alpha = 0.3;
        const double k1 = 10.0;
    };
    StableConstants stableConstants;

    struct CasadiCorner
    {
        casadi::DM position;
        casadi::MX force;

        std::string cornerName;

        CasadiCorner(const std::string& cornerName)
            : cornerName(cornerName)
        {
        }

        CasadiCorner() = default;

        CasadiCorner(const std::string& cornerName, const Corner& other)
            : cornerName(cornerName)
        {
            this->operator=(other);
        }

        CasadiCorner& operator=(const Corner& other)
        {
            this->position.resize(3, 1);
            this->position(0, 0) = other.position(0);
            this->position(1, 0) = other.position(1);
            this->position(2, 0) = other.position(2);

            this->force = casadi::MX::sym(cornerName + "_force", other.force.size(), 1);

            return *this;
        }
    };

    struct CasadiContact
    {
        casadi::MX position;
        casadi::MX linearVelocity;
        casadi::MX orientation;
        casadi::MX isEnabled;
        std::vector<CasadiCorner> corners;

        std::string contactName;

        CasadiContact(const std::string& contactName)
            : contactName(contactName)
        {
        }

        CasadiContact& operator=(const DiscreteGeometryContact& other)
        {
            corners.resize(other.corners.size());

            for (int i = 0; i < other.corners.size(); i++)
            {
                this->corners[i].cornerName = contactName + "_" + std::to_string(i);
                this->corners[i] = other.corners[i];
            }

            this->orientation = casadi::MX::sym(contactName + "_orientation", 3 * 3);
            this->position = casadi::MX::sym(contactName + "_position", 3);
            this->linearVelocity = casadi::MX::sym(contactName + "_linear_velocity", 3);
            this->isEnabled = casadi::MX::sym(contactName + "_is_enable");

            return *this;
        }

        CasadiContact(const std::string& contactName, const DiscreteGeometryContact& other)
            : contactName(contactName)
        {
            this->operator=(other);
        }
    };

    struct CasadiContactWithConstraints : CasadiContact
    {

        CasadiContactWithConstraints(const std::string& contactName)
            : CasadiContact(contactName)
        {
        }

        casadi::MX currentPosition;
        casadi::MX nominalPosition;
        casadi::MX upperLimitPosition;
        casadi::MX lowerLimitPosition;
    };

    struct OptimizationSettings
    {
        int solverVerbosity{0}; /**< Verbosity of ipopt */
        std::string ipoptLinearSolver{"mumps"}; /**< Linear solved used by ipopt */
        double ipoptTolerance{1e-8}; /**< Tolerance of ipopt
                                        (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_tol) */
        int ipoptMaxIteration{3000}; /**< Maximum number of iteration */

        int horizon; /**<Number of samples used in the horizon */
        std::chrono::nanoseconds samplingTime; /**< Sampling time of the planner */
        std::chrono::nanoseconds timeHorizon; /**< Duration of the horizon */
        bool isWarmStartEnabled{false}; /**< True if the user wants to warm start the CoM, angular
                                           momentum and contact. */
        bool isCseEnabled{false}; /**< True if the Common subexpression elimination casadi option is
                                       enabled. */
    };

    OptimizationSettings optiSettings; /**< Settings */

    /**
     * OptimizationVariables contains the optimization variables expressed as CasADi elements.
     */
    struct OptimizationVariables
    {
        casadi::MX com;
        casadi::MX dcom;
        casadi::MX angularMomentum;
        std::map<std::string, CasadiContactWithConstraints> contacts;

        /**< Optimization variables from the adaptive redesign*/
        casadi::MX z1;
        casadi::MX z2;

        casadi::MX comReference;
        casadi::MX angularMomentumReference;
        casadi::MX comCurrent;
        casadi::MX dcomCurrent;
        casadi::MX angularMomentumCurrent;
        casadi::MX externalForce;
        casadi::MX externalTorque;

    };
    OptimizationVariables optiVariables; /**< Optimization variables */

    struct ContactsInputs
    {
        casadi::DM* currentPosition;
        casadi::DM* orientation;
        casadi::DM* nominalPosition;
        casadi::DM* upperLimitPosition;
        casadi::DM* lowerLimitPosition;
        casadi::DM* isEnabled;
    };
    struct ControllerInputs
    {
        std::map<std::string, ContactsInputs> contacts;

        casadi::DM* comReference;
        casadi::DM* angularMomentumReference;
        casadi::DM* comCurrent;
        casadi::DM* dcomCurrent;
        casadi::DM* angularMomentumCurrent;
        casadi::DM* externalForce;
        casadi::DM* externalTorque;
    };
    ControllerInputs controllerInputs; /**< The pointers will point to the vectorized input */

    struct InitialGuess
    {
        std::map<std::string, casadi::DM*> contactsLocation;

        casadi::DM* com;
        casadi::DM* angularMomentum;
    };
    InitialGuess initialGuess;

    std::vector<casadi::DM> vectorizedOptiInputs;

    struct Weights
    {
        Eigen::Vector3d com;
        double contactPosition;
        Eigen::Vector3d forceRateOfChange;
        double angularMomentum;
        double contactForceSymmetry;
    };
    Weights weights;

    struct ContactBoundingBox
    {
        Eigen::Vector3d upperLimit;
        Eigen::Vector3d lowerLimit;
    };

    std::unordered_map<std::string, ContactBoundingBox> contactBoundingBoxes;

    bool loadContactCorners(std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                            DiscreteGeometryContact& contact)
    {
        constexpr auto errorPrefix = "[CentroidalMPC::Impl::loadContactCorners]";

        int numberOfCorners;
        if (!ptr->getParameter("number_of_corners", numberOfCorners))
        {
            log()->error("{} Unable to get the number of corners.");
            return false;
        }
        contact.corners.resize(numberOfCorners);

        for (std::size_t j = 0; j < numberOfCorners; j++)
        {
            if (!ptr->getParameter("corner_" + std::to_string(j), contact.corners[j].position))
            {
                // prepare the error
                std::string cornesNames;
                for (std::size_t k = 0; k < numberOfCorners; k++)
                {
                    cornesNames += " corner_" + std::to_string(k);
                }

                log()->error("{} Unable to load the corner number {}. Please provide the corners "
                             "having the following names:{}.",
                             errorPrefix,
                             j,
                             cornesNames);

                return false;
            }
        }

        return true;
    }

    bool loadParameters(std::shared_ptr<const ParametersHandler::IParametersHandler> ptr)
    {
        constexpr auto logPrefix = "[CentroidalMPC::Impl::loadParameters]";

        auto getParameter
            = [logPrefix](std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                          const std::string& paramName,
                          auto& param) -> bool {
            if (!ptr->getParameter(paramName, param))
            {
                log()->error("{} Unable to load the parameter named '{}'.", logPrefix, paramName);
                return false;
            }
            return true;
        };

        auto getOptionalParameter
            = [logPrefix](std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                          const std::string& paramName,
                          auto& param) -> void {
            if (!ptr->getParameter(paramName, param))
            {
                log()->info("{} Unable to load the parameter named '{}'. The default one will be "
                            "used '{}'.",
                            logPrefix,
                            paramName,
                            param);
            }
        };

        bool ok = getParameter(ptr, "sampling_time", this->optiSettings.samplingTime);
        ok = ok && getParameter(ptr, "time_horizon", this->optiSettings.timeHorizon);

        if (!ok)
        {
            return false;
        }
        this->optiSettings.horizon
            = this->optiSettings.timeHorizon / this->optiSettings.samplingTime;

        int numberOfMaximumContacts = 0;
        ok = ok && getParameter(ptr, "number_of_maximum_contacts", numberOfMaximumContacts);

        for (std::size_t i = 0; i < numberOfMaximumContacts; i++)
        {
            auto contactHandler = ptr->getGroup("CONTACT_" + std::to_string(i)).lock();

            if (contactHandler == nullptr)
            {
                log()->error("{} Unable to load the contact {}. Please be sure that CONTACT_{} "
                             "group exists.",
                             logPrefix,
                             i,
                             i);
                return false;
            }

            std::string contactName;
            ok = ok && getParameter(contactHandler, "contact_name", contactName);
            if (!ok)
            {
                return false;
            }

            // set the contact name
            this->output.contacts[contactName].name = contactName;
            ok = ok
                 && getParameter(contactHandler,
                                 "bounding_box_upper_limit",
                                 this->contactBoundingBoxes[contactName].upperLimit);
            ok = ok
                 && getParameter(contactHandler,
                                 "bounding_box_lower_limit",
                                 this->contactBoundingBoxes[contactName].lowerLimit);

            if (!this->loadContactCorners(contactHandler, this->output.contacts[contactName]))
            {
                log()->error("{} Unable to load the contact corners for the contact {}.",
                             logPrefix,
                             i);
                return false;
            }
        }

        ok = ok && getParameter(ptr, "com_weight", this->weights.com);
        ok = ok && getParameter(ptr, "contact_position_weight", this->weights.contactPosition);
        ok = ok
             && getParameter(ptr, "force_rate_of_change_weight", this->weights.forceRateOfChange);
        ok = ok && getParameter(ptr, "angular_momentum_weight", this->weights.angularMomentum);
        ok = ok
             && getParameter(ptr,
                             "contact_force_symmetry_weight",
                             this->weights.contactForceSymmetry);

        // initialize the friction cone
        ok = ok && frictionCone.initialize(ptr);

        getOptionalParameter(ptr, "linear_solver", this->optiSettings.ipoptLinearSolver);
        getOptionalParameter(ptr, "ipopt_tolerance", this->optiSettings.ipoptTolerance);
        getOptionalParameter(ptr, "ipopt_max_iteration", this->optiSettings.ipoptMaxIteration);
        getOptionalParameter(ptr, "solver_verbosity", this->optiSettings.solverVerbosity);
        getOptionalParameter(ptr, "is_warm_start_enabled", this->optiSettings.isWarmStartEnabled);
        getOptionalParameter(ptr, "is_cse_enabled", this->optiSettings.isCseEnabled);

        return ok;
    }

    casadi::Function ode()
    {
        // Convert DiscreteGeometryContact into a casadiContact object
        std::map<std::string, CasadiContact> casadiContacts;

        for (const auto& [key, contact] : this->output.contacts)
        {
            CasadiContact temp(key);
            temp = contact;
            auto [contactIt, outcome] = casadiContacts.emplace(key, temp);
        }

        // we assume mass equal to 1
        constexpr double mass = 1;

        casadi::MX com = casadi::MX::sym("com_in", 3);
        casadi::MX dcom = casadi::MX::sym("dcom_in", 3);
        casadi::MX angularMomentum = casadi::MX::sym("angular_momentum_in", 3);

        casadi::MX externalForce = casadi::MX::sym("external_force", 3);
        casadi::MX externalTorque = casadi::MX::sym("external_torque", 3);

        casadi::MX ddcom = casadi::MX::sym("ddcom", 3);
        casadi::MX angularMomentumDerivative = casadi::MX::sym("angular_momentum_derivative", 3);

        casadi::DM gravity = casadi::DM::zeros(3);
        gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

        ddcom = gravity + externalForce / mass;
        angularMomentumDerivative = externalTorque;

        std::vector<casadi::MX> input;
        input.push_back(externalForce);
        input.push_back(externalTorque);
        input.push_back(com);
        input.push_back(dcom);
        input.push_back(angularMomentum);

        for (const auto& [key, contact] : casadiContacts)
        {
            input.push_back(contact.position);
            input.push_back(contact.orientation);
            input.push_back(contact.isEnabled);
            input.push_back(contact.linearVelocity);

            for (const auto& corner : contact.corners)
            {
                using namespace casadi;
                ddcom += contact.isEnabled / mass * corner.force;
                angularMomentumDerivative
                    += contact.isEnabled
                       * MX::cross(MX::mtimes(MX::reshape(contact.orientation, 3, 3),
                                              corner.position)
                                       + contact.position - com,
                                   corner.force);

                input.push_back(corner.force);
            }
        }

        const double dT = chronoToSeconds(this->optiSettings.samplingTime);

        std::vector<std::string> outputName{"com", "dcom", "angular_momentum"};
        std::vector<casadi::MX> rhs{com + dcom * dT,
                                    dcom + ddcom * dT,
                                    angularMomentum + angularMomentumDerivative * dT};

        for (const auto& [key, contact] : casadiContacts)
        {
            rhs.push_back(contact.position + (1 - contact.isEnabled) * contact.linearVelocity * dT);
            outputName.push_back(key);
        }

        return casadi::Function("centroidal_dynamics",
                                std::move(input),
                                std::move(rhs),
                                extractVariablesName(input),
                                std::move(outputName));
    }

    casadi::Function contactPositionError()
    {
        casadi::MX contactPosition = casadi::MX::sym("contact_position", 3);
        casadi::MX nominalContactPosition = casadi::MX::sym("nominal_contact_position", 3);
        casadi::MX contactOrientation = casadi::MX::sym("contact_orientation", 3 * 3);

        // the orientation is stored as a vectorized version of the matrix. We need to reshape it
        casadi::MX rhs = casadi::MX::mtimes(casadi::MX::reshape(contactOrientation, 3, 3).T(),
                                            contactPosition - nominalContactPosition);

        return casadi::Function("contact_position_error",
                                {contactPosition, nominalContactPosition, contactOrientation},
                                {rhs},
                                extractVariablesName({contactPosition, //
                                                      nominalContactPosition,
                                                      contactOrientation}),
                                {"error"});
    }

    void resizeControllerInputs()
    {
        constexpr int vector3Size = 3;
        const int stateHorizon = this->optiSettings.horizon + 1;

        // In case of no warmstart the variables are:
        // - centroidalVariables = 7: external force + external torque + com current + dcom current
        //                            + current angular momentum + com reference
        //                            + angular momentum reference
        // - contactVariables = 6: for each contact we have current position + nominal position +
        //                         orientation + is enabled + upper limit in position
        //                         + lower limit in position
        constexpr std::size_t centroidalVariables = 7;
        constexpr std::size_t contactVariables = 6;

        std::size_t vectorizedOptiInputsSize = centroidalVariables + //
                                               (this->output.contacts.size() * contactVariables);

        if (this->optiSettings.isWarmStartEnabled)
        {
            // in this case we need to add the com, the angular momentum and the contact location
            constexpr std::size_t centroidalVariablesWarmStart = 2;
            constexpr std::size_t contactVariablesWarmStart = 1;
            vectorizedOptiInputsSize += centroidalVariablesWarmStart + //
                                        (this->output.contacts.size() * contactVariablesWarmStart);
        }

        // we reserve in advance so the push_back will not invalidate the pointers
        // Indeed the standard guarantees that if the new size() is greater than capacity() then all
        // iterators and references (including the end() iterator) are invalidated. Otherwise only
        // the end() iterator is invalidated.
        // https://en.cppreference.com/w/cpp/container/vector/push_back
        this->vectorizedOptiInputs.reserve(vectorizedOptiInputsSize);

        // prepare the controller inputs struct
        // The order matches the one required by createController
        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, //
                                                               this->optiSettings.horizon));
        this->controllerInputs.externalForce = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, //
                                                               this->optiSettings.horizon));
        this->controllerInputs.externalTorque = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.comCurrent = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.dcomCurrent = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
        this->controllerInputs.angularMomentumCurrent = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
        this->controllerInputs.comReference = &this->vectorizedOptiInputs.back();

        this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
        this->controllerInputs.angularMomentumReference = &this->vectorizedOptiInputs.back();

        if (this->optiSettings.isWarmStartEnabled)
        {
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
            this->initialGuess.com = &this->vectorizedOptiInputs.back();

            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
            this->initialGuess.angularMomentum = &this->vectorizedOptiInputs.back();
        }

        for (const auto& [key, contact] : this->output.contacts)
        {
            // The current position of the contact
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size));
            this->controllerInputs.contacts[key].currentPosition
                = &this->vectorizedOptiInputs.back();

            // The nominal contact position is a parameter that regularize the solution
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
            this->controllerInputs.contacts[key].nominalPosition
                = &this->vectorizedOptiInputs.back();

            // The orientation is stored as a vectorized version of the rotation matrix
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(9, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].orientation = &this->vectorizedOptiInputs.back();

            // Maximum admissible contact force. It is expressed in the contact body frame
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(1, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].isEnabled = &this->vectorizedOptiInputs.back();

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            this->vectorizedOptiInputs.push_back(
                casadi::DM::zeros(vector3Size, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].upperLimitPosition
                = &this->vectorizedOptiInputs.back();

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            this->vectorizedOptiInputs.push_back(
                casadi::DM::zeros(vector3Size, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].lowerLimitPosition
                = &this->vectorizedOptiInputs.back();

            if (this->optiSettings.isWarmStartEnabled)
            {
                this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
                this->initialGuess.contactsLocation[key] = &this->vectorizedOptiInputs.back();
            }
        }

        assert(vectorizedOptiInputsSize == this->vectorizedOptiInputs.size());
    }

    void populateOptiVariables()
    {
        constexpr int vector3Size = 3;
        const int stateHorizon = this->optiSettings.horizon + 1;

        // create the variables for the state
        this->optiVariables.com = this->opti.variable(vector3Size, stateHorizon);
        this->optiVariables.dcom = this->opti.variable(vector3Size, stateHorizon);
        this->optiVariables.angularMomentum = this->opti.variable(vector3Size, stateHorizon);

        // create the variables for the stability
        this->optiVariables.z1 = this->opti.variable(vector3Size);
        this->optiVariables.z2 = this->opti.variable(vector3Size);


        // the casadi contacts depends on the maximum number of contacts
        for (const auto& [key, contact] : this->output.contacts)
        {
            auto [contactIt, outcome]
                = this->optiVariables.contacts.insert_or_assign(key,
                                                                CasadiContactWithConstraints(key));

            auto& c = contactIt->second;

            // each contact has a different number of corners
            c.corners.resize(contact.corners.size());

            // the position of the contact is an optimization variable
            c.position = this->opti.variable(vector3Size, stateHorizon);

            // the linear velocity of the contact is an optimization variable
            c.linearVelocity = this->opti.variable(vector3Size, this->optiSettings.horizon);

            // the orientation is a parameter. The orientation is stored as a vectorized version of
            // the rotation matrix
            c.orientation = this->opti.parameter(9, this->optiSettings.horizon);

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            c.upperLimitPosition = this->opti.parameter(vector3Size, this->optiSettings.horizon);

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            c.lowerLimitPosition = this->opti.parameter(vector3Size, this->optiSettings.horizon);

            // Maximum admissible contact force. It is expressed in the contact body frame
            c.isEnabled = this->opti.parameter(1, this->optiSettings.horizon);

            // The nominal contact position is a parameter that regularize the solution
            c.nominalPosition = this->opti.parameter(vector3Size, stateHorizon);

            c.currentPosition = this->opti.parameter(vector3Size);

            for (int j = 0; j < contact.corners.size(); j++)
            {
                c.corners[j].force = this->opti.variable(vector3Size, this->optiSettings.horizon);

                c.corners[j].position
                    = casadi::DM(std::vector<double>(contact.corners[j].position.data(),
                                                     contact.corners[j].position.data()
                                                         + contact.corners[j].position.size()));
            }
        }

        this->optiVariables.comCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.dcomCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.angularMomentumCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.comReference = this->opti.parameter(vector3Size, stateHorizon);
        this->optiVariables.angularMomentumReference
            = this->opti.parameter(vector3Size, stateHorizon);
        this->optiVariables.externalForce = this->opti.parameter(vector3Size, //
                                                                 this->optiSettings.horizon);
        this->optiVariables.externalTorque = this->opti.parameter(vector3Size, //
                                                                  this->optiSettings.horizon);
    }

    /**
     * Setup the optimization problem options
     */
    void setupOptiOptions()
    {
        casadi::Dict casadiOptions;
        casadi::Dict ipoptOptions;

        if (this->optiSettings.solverVerbosity != 0)
        {
            casadi_int ipoptVerbosity = static_cast<long long>(optiSettings.solverVerbosity - 1);
            ipoptOptions["print_level"] = ipoptVerbosity;
            casadiOptions["print_time"] = true;
        } else
        {
            ipoptOptions["print_level"] = 0;
            casadiOptions["print_time"] = false;
        }

        ipoptOptions["max_iter"] = this->optiSettings.ipoptMaxIteration;
        ipoptOptions["tol"] = this->optiSettings.ipoptTolerance;
        ipoptOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;
        casadiOptions["expand"] = true;
        casadiOptions["error_on_fail"] = true;

        this->opti.solver("ipopt", casadiOptions, ipoptOptions);
    }

    casadi::Function createController()
    {
        using Sl = casadi::Slice;

        this->populateOptiVariables();

        // get the variables to simplify the readability
        auto& com = this->optiVariables.com;
        auto& dcom = this->optiVariables.dcom;
        auto& angularMomentum = this->optiVariables.angularMomentum;
        auto& externalForce = this->optiVariables.externalForce;
        auto& externalTorque = this->optiVariables.externalTorque;

        auto& z1 = this->optiVariables.z1;
        auto& z2 = this->optiVariables.z2;

        // prepare the input of the ode
        std::vector<casadi::MX> odeInput;
        odeInput.push_back(externalForce);
        odeInput.push_back(externalTorque);
        odeInput.push_back(com(Sl(), Sl(0, -1)));
        odeInput.push_back(dcom(Sl(), Sl(0, -1)));
        odeInput.push_back(angularMomentum(Sl(), Sl(0, -1)));
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            odeInput.push_back(contact.position(Sl(), Sl(0, -1)));
            odeInput.push_back(contact.orientation);
            odeInput.push_back(contact.isEnabled);
            odeInput.push_back(contact.linearVelocity);

            for (const auto& corner : contact.corners)
            {
                odeInput.push_back(corner.force);
            }
        }

        // set the feedback
        this->opti.subject_to(this->optiVariables.comCurrent == com(Sl(), 0));
        this->opti.subject_to(this->optiVariables.dcomCurrent == dcom(Sl(), 0));
        this->opti.subject_to(this->optiVariables.angularMomentumCurrent
                              == angularMomentum(Sl(), 0));

        // stability constraint

        this->opti.subject_to(z1 == com(Sl(),1) - this->optiVariables.comReference(Sl(), 1));
        this->opti.subject_to(z2 == dcom(Sl(),1) + this->stableConstants.k1 * (com(Sl(),1) - this->optiVariables.comReference(Sl(), 1)));


        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            for (const auto& corner : contact.corners)
            {
                this->opti.subject_to(
                    -casadi::MX::mtimes(z1(Sl(), 0).T(), z1(Sl(), 0))
                        - casadi::MX::mtimes(z2(Sl(), 0).T(), z2(Sl(), 0))
                        + casadi::MX::mtimes(z1(Sl(), 0).T(), z2(Sl(), 0))
                        + casadi::MX::mtimes(z2(Sl(), 0).T(), corner.force(Sl(), 0))
                    <= 0.0);
            }
        }


        this->opti.subject_to(angularMomentum(Sl(),1)  <= this->stableConstants.alpha * casadi::MX::ones(3));

        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            this->opti.subject_to(this->optiVariables.contacts.at(key).currentPosition
                                  == contact.position(Sl(), 0));
        }

        // set the dynamics
        // map computes the multiple shooting method
        auto dynamics = this->ode().map(this->optiSettings.horizon);
        auto fullTrajectory = dynamics(odeInput);
        this->opti.subject_to(extractFutureValuesFromState(com) == fullTrajectory[0]);
        this->opti.subject_to(extractFutureValuesFromState(dcom) == fullTrajectory[1]);
        this->opti.subject_to(extractFutureValuesFromState(angularMomentum) == fullTrajectory[2]);

        // footstep dynamics
        std::size_t contactIndex = 0;
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            this->opti.subject_to(extractFutureValuesFromState(contact.position)
                                  == fullTrajectory[3 + contactIndex]);
            contactIndex++;
        }

        // add constraints for the contacts
        auto contactPositionErrorMap = this->contactPositionError().map(this->optiSettings.horizon);

        // convert the eigen matrix into casadi
        // please check https://github.com/casadi/casadi/issues/2563 and
        // https://groups.google.com/forum/#!topic/casadi-users/npPcKItdLN8
        // Assumption: the matrices are stored as column-major
        casadi::DM frictionConeMatrix = casadi::DM::zeros(frictionCone.getA().rows(), //
                                                          frictionCone.getA().cols());

        std::memcpy(frictionConeMatrix.ptr(),
                    frictionCone.getA().data(),
                    sizeof(double) * frictionCone.getA().rows() * frictionCone.getA().cols());

        const casadi::DM zero = casadi::DM::zeros(frictionCone.getA().rows(), 1);
        casadi::MX rotatedFrictionCone;

        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            auto error
                = contactPositionErrorMap({extractFutureValuesFromState(contact.position),
                                           extractFutureValuesFromState(contact.nominalPosition),
                                           contact.orientation});

            this->opti.subject_to(contact.lowerLimitPosition <= error[0]
                                  <= contact.upperLimitPosition);

            for (int i = 0; i < this->optiSettings.horizon; i++)
            {
                rotatedFrictionCone
                    = casadi::MX::mtimes(frictionConeMatrix, //
                                         casadi::MX::reshape(contact.orientation(Sl(), i), 3, 3)
                                             .T());

                // TODO please if you want to add heel to toe motion you should define a
                // contact.maximumNormalForce for each corner. At this stage is too premature.
                for (const auto& corner : contact.corners)
                {
                    this->opti.subject_to(casadi::MX::mtimes(rotatedFrictionCone, //
                                                             corner.force(Sl(), i))
                                          <= zero);

                    // limit on the normal force
                    this->opti.subject_to(
                        0 <= casadi::MX::mtimes(casadi::MX::reshape(contact.orientation(Sl(), i),
                                                                    3,
                                                                    3),
                                                corner.force(Sl(), i)(2)));
                }
            }
        }

        // create the cost function
        auto& comReference = this->optiVariables.comReference;
        auto& angularMomentumReference = this->optiVariables.angularMomentumReference;

        // (max - mix) * exp(-i) + min
        casadi::DM weightCoMZ = casadi::DM::zeros(1, com.columns());
        const double min = this->weights.com(2) / 2;
        for (int i = 0; i < com.columns(); i++)
        {
            weightCoMZ(Sl(), i) = (this->weights.com(2) - min) * std::exp(-i) + min;
        }

        casadi::MX cost
            = this->weights.angularMomentum
                  * casadi::MX::sumsqr(angularMomentum - angularMomentumReference)
              + this->weights.com(0) * casadi::MX::sumsqr(com(0, Sl()) - comReference(0, Sl()))
              + this->weights.com(1) * casadi::MX::sumsqr(com(1, Sl()) - comReference(1, Sl()))
              + casadi::MX::sumsqr(weightCoMZ * (com(2, Sl()) - comReference(2, Sl())));

        casadi::MX averageForce;
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            cost += this->weights.contactPosition
                    * casadi::MX::sumsqr(contact.nominalPosition - contact.position);

            averageForce = casadi::MX::vertcat(
                {contact.isEnabled * contact.corners[0].force(0, Sl()) / contact.corners.size(),
                 contact.isEnabled * contact.corners[0].force(1, Sl()) / contact.corners.size(),
                 contact.isEnabled * contact.corners[0].force(2, Sl()) / contact.corners.size()});
            for (int i = 1; i < contact.corners.size(); i++)
            {
                averageForce += casadi::MX::vertcat(
                    {contact.isEnabled * contact.corners[i].force(0, Sl()) / contact.corners.size(),
                     contact.isEnabled * contact.corners[i].force(1, Sl()) / contact.corners.size(),
                     contact.isEnabled * contact.corners[i].force(2, Sl())
                         / contact.corners.size()});
            }

            for (const auto& corner : contact.corners)
            {
                auto forceRateOfChange = casadi::MX::diff(corner.force.T()).T();

                cost += this->weights.contactForceSymmetry
                        * casadi::MX::sumsqr(corner.force - averageForce);

                cost += this->weights.forceRateOfChange(0)
                        * casadi::MX::sumsqr(forceRateOfChange(0, Sl()));
                cost += this->weights.forceRateOfChange(1)
                        * casadi::MX::sumsqr(forceRateOfChange(1, Sl()));
                cost += this->weights.forceRateOfChange(2)
                        * casadi::MX::sumsqr(forceRateOfChange(2, Sl()));
            }
        }

        this->opti.minimize(cost);

        this->setupOptiOptions();

        // prepare the casadi function
        std::vector<casadi::MX> input;
        std::vector<casadi::MX> output;
        std::vector<std::string> inputName;
        std::vector<std::string> outputName;

        auto concatenateInput
            = [&input, &inputName](const casadi::MX& inputVariable, std::string inputVariableName) {
                  input.push_back(inputVariable);
                  inputName.push_back(std::move(inputVariableName));
              };

        auto concatenateOutput = [&output, &outputName](const casadi::MX& outputVariable,
                                                        std::string outputVariableName) {
            output.push_back(outputVariable);
            outputName.push_back(std::move(outputVariableName));
        };

        concatenateInput(this->optiVariables.externalForce, "external_force");
        concatenateInput(this->optiVariables.externalTorque, "external_torque");
        concatenateInput(this->optiVariables.comCurrent, "com_current");
        concatenateInput(this->optiVariables.dcomCurrent, "dcom_current");
        concatenateInput(this->optiVariables.angularMomentumCurrent, "angular_momentum_current");

        concatenateInput(this->optiVariables.comReference, "com_reference");
        concatenateInput(this->optiVariables.angularMomentumReference,
                         "angular_momentum_reference");

        // this only if warm-start is enabled
        if (this->optiSettings.isWarmStartEnabled)
        {
            concatenateInput(this->optiVariables.com, "com_warmstart");
            concatenateInput(this->optiVariables.angularMomentum, "angular_momentum_warmstart");
        }

        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            concatenateInput(contact.currentPosition, "contact_" + key + "_current_position");
            concatenateInput(contact.nominalPosition, "contact_" + key + "_nominal_position");
            concatenateInput(contact.orientation, "contact_" + key + "_orientation_input");
            concatenateInput(contact.isEnabled, "contact_" + key + "is_enable_in");
            concatenateInput(contact.upperLimitPosition,
                             "contact_" + key + "_upper_limit_position");
            concatenateInput(contact.lowerLimitPosition,
                             "contact_" + key + "_lower_limit_position");

            // this only if warm-start is enabled
            if (this->optiSettings.isWarmStartEnabled)
            {
                concatenateInput(contact.position, "contact_" + key + "_position_warmstart");
            }

            concatenateOutput(contact.isEnabled, "contact_" + key + "_is_enable");
            concatenateOutput(contact.position, "contact_" + key + "_position");
            concatenateOutput(contact.orientation, "contact_" + key + "_orientation");

            std::size_t cornerIndex = 0;
            for (const auto& corner : contact.corners)
            {
                concatenateOutput(corner.force,
                                  "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                      + "_force");
                cornerIndex++;
            }
        }

        casadi::Dict toFunctionOptions;
        if (casadiVersionIsAtLeast360())
        {
            toFunctionOptions["cse"] = this->optiSettings.isCseEnabled;
        }

        return this->opti
            .to_function("controller", input, output, inputName, outputName, toFunctionOptions);
    }
};

bool StableCentroidalMPC::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[CentroidalMPC::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.", errorPrefix);
        return false;
    }

    if (!m_pimpl->loadParameters(ptr))
    {
        log()->error("{} Unable to load the parameters.", errorPrefix);
        return false;
    }

    m_pimpl->resizeControllerInputs();
    m_pimpl->controller = m_pimpl->createController();
    m_pimpl->fsm = Impl::FSM::Initialized;

    return true;
}

StableCentroidalMPC::~StableCentroidalMPC() = default;

StableCentroidalMPC::StableCentroidalMPC()
{
    m_pimpl = std::make_unique<Impl>();
}

const CentroidalMPCOutput& StableCentroidalMPC::getOutput() const
{
    return m_pimpl->output;
}



bool StableCentroidalMPC::isOutputValid() const
{
    return m_pimpl->fsm == Impl::FSM::OutputValid;
}

bool StableCentroidalMPC::advance()
{
    constexpr auto errorPrefix = "[CentroidalMPC::advance]";
    assert(m_pimpl);

    using Sl = casadi::Slice;

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    // invalidate the output
    m_pimpl->fsm = Impl::FSM::OutputInvalid;

    // compute the output
    std::vector<casadi::DM> controllerOutput;
    try
    {
        controllerOutput = m_pimpl->controller(m_pimpl->vectorizedOptiInputs);
    } catch (const std::exception& e)
    {
        log()->error("{} Unable to solve the problem. The following exception has been thrown {}.",
                     errorPrefix,
                     e.what());
        return false;
    }

    // get the solution
    auto it = controllerOutput.begin();
    m_pimpl->output.nextPlannedContact.clear();
    for (auto& [key, contact] : m_pimpl->output.contacts)
    {
        // this is required for toEigen
        using namespace BipedalLocomotion::Conversions;

        int index = toEigen(*it).size();
        const int size = toEigen(*it).size();
        for (int i = 0; i < size; i++)
        {
            if (toEigen(*it)(i) > 0.5)
            {
                if (i == 0)
                {
                    break;
                } else if (toEigen(*it)(i - 1) < 0.5)
                {
                    index = i;
                    break;
                }
            }
        }

        double isEnabled = toEigen(*it)(0);
        std::advance(it, 1);
        contact.pose.translation(toEigen(*it).leftCols<1>());

        if (index < size)
        {
            m_pimpl->output.nextPlannedContact[key].name = key;
            m_pimpl->output.nextPlannedContact[key].pose.translation(toEigen(*it).col(index));
        }

        std::advance(it, 1);
        contact.pose.quat(Eigen::Quaterniond(
            Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

        if (index < size)
        {
            m_pimpl->output.nextPlannedContact[key].pose.quat(Eigen::Quaterniond(
                Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

            const std::chrono::nanoseconds nextPlannedContactTime
                = m_pimpl->currentTime + m_pimpl->optiSettings.samplingTime * index;

            auto nextPlannedContact = m_pimpl->contactPhaseList.lists().at(key).getPresentContact(
                nextPlannedContactTime);
            if (nextPlannedContact == m_pimpl->contactPhaseList.lists().at(key).end())
            {
                log()->error("[CentroidalMPC::advance] Unable to get the next planned contact");
                return false;
            }

            auto& contact = m_pimpl->output.nextPlannedContact[key];
            contact.activationTime = nextPlannedContact->activationTime;
            contact.deactivationTime = nextPlannedContact->deactivationTime;
            contact.index = nextPlannedContact->index;
            contact.type = nextPlannedContact->type;
        }

        std::advance(it, 1);

        for (auto& corner : contact.corners)
        {
            // isEnabled == 1 means that the contact is active
            if (isEnabled > 0.5)
            {
                corner.force = toEigen(*it).leftCols<1>();
            } else
            {
                corner.force.setZero();
            }
            std::advance(it, 1);
        }
    }

    // advance the time
    m_pimpl->currentTime += m_pimpl->optiSettings.samplingTime;

    // Make the output valid
    m_pimpl->fsm = Impl::FSM::OutputValid;

    return true;
}

bool StableCentroidalMPC::setReferenceTrajectory(const std::vector<Eigen::Vector3d>& com,
                                           const std::vector<Eigen::Vector3d>& angularMomentum)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setReferenceTrajectory]";

    const int stateHorizon = m_pimpl->optiSettings.horizon + 1;

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    if (com.size() < stateHorizon)
    {
        log()->error("{} The CoM trajectory vector should have at least {} elements. Provided "
                     "size: {}.",
                     errorPrefix,
                     stateHorizon,
                     com.size());
        return false;
    }

    if (angularMomentum.size() < stateHorizon)
    {
        log()->error("{} The angular momentum trajectory vector should have at least {} elements. "
                     "Provided size: {}.",
                     errorPrefix,
                     stateHorizon,
                     angularMomentum.size());
        return false;
    }

    // Since Eigen vector is a contiguous we can copy the CoM and the angular momentum references by
    // columns.
    for (int i = 0; i < stateHorizon; i++)
    {
        using namespace BipedalLocomotion::Conversions;
        toEigen(*(m_pimpl->controllerInputs.comReference)).col(i)
            = Eigen::Map<const Eigen::Vector3d>(com[i].data());
        toEigen(*(m_pimpl->controllerInputs.angularMomentumReference)).col(i)
            = Eigen::Map<const Eigen::Vector3d>(angularMomentum[i].data());
    }

    // if the warmstart is enabled then the reference is used also as warmstart
    if (m_pimpl->optiSettings.isWarmStartEnabled)
    {
        using namespace BipedalLocomotion::Conversions;
        toEigen(*(m_pimpl->initialGuess.com)) = toEigen(*(m_pimpl->controllerInputs.comReference));
        toEigen(*(m_pimpl->initialGuess.angularMomentum))
            = toEigen(*(m_pimpl->controllerInputs.angularMomentumReference));
    }

    return true;
}

bool StableCentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum)
{
    const Math::Wrenchd dummy = Math::Wrenchd::Zero();
    return this->setState(com, dcom, angularMomentum, dummy);
}

bool StableCentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                             const Math::Wrenchd& externalWrench)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setState]";
    assert(m_pimpl);

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    auto& inputs = m_pimpl->controllerInputs;

    using namespace BipedalLocomotion::Conversions;
    toEigen(*inputs.comCurrent) = com;
    toEigen(*inputs.dcomCurrent) = dcom;
    toEigen(*inputs.angularMomentumCurrent) = angularMomentum;

    toEigen(*inputs.externalForce).setZero();
    toEigen(*inputs.externalTorque).setZero();

    toEigen(*inputs.externalForce).leftCols<1>() = externalWrench.force();
    toEigen(*inputs.externalTorque).leftCols<1>() = externalWrench.torque();

    return true;
}

bool StableCentroidalMPC::setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setContactPhaseList]";
    assert(m_pimpl);

    if (m_pimpl->fsm == Impl::FSM::Idle)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    if (contactPhaseList.size() == 0)
    {
        log()->error("{} The contactPhaseList is empty.", errorPrefix);
        return false;
    }

    m_pimpl->contactPhaseList = contactPhaseList;

    // The orientation is stored as a vectorized version of the rotation matrix
    const Eigen::Matrix3d identity = Eigen::Matrix3d::Identity();

    auto& inputs = m_pimpl->controllerInputs;

    // clear previous data
    for (const auto& [key, contact] : m_pimpl->output.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        // initialize the current contact pose to zero. If the contact is active the current
        // position will be set later on
        toEigen(*inputs.contacts[key].currentPosition).setZero();

        // initialize all the orientation to the identity
        toEigen(*inputs.contacts[key].orientation).colwise()
            = Eigen::Map<const Eigen::VectorXd>(identity.data(), identity.cols() * identity.rows());

        // Upper limit of the position of the contact. It is expressed in the contact body frame
        toEigen(*inputs.contacts[key].upperLimitPosition).setZero();

        // Lower limit of the position of the contact. It is expressed in the contact body frame
        toEigen(*inputs.contacts[key].lowerLimitPosition).setZero();

        // Maximum admissible contact force. It is expressed in the contact body frame
        toEigen(*inputs.contacts[key].isEnabled).setZero();

        // The nominal contact position is a parameter that regularize the solution
        toEigen(*inputs.contacts[key].nominalPosition).setZero();
    }

    const std::chrono::nanoseconds absoluteTimeHorizon
        = m_pimpl->currentTime + m_pimpl->optiSettings.timeHorizon;

    // find the contactPhase associated to the current time
    auto initialPhase = contactPhaseList.getPresentPhase(m_pimpl->currentTime);
    if (initialPhase == contactPhaseList.end())
    {
        log()->error("{} Unable to find the contact phase related to the current time.",
                     errorPrefix);
        return false;
    }

    // find the contactPhase associated to the end time
    auto finalPhase = contactPhaseList.getPresentPhase(absoluteTimeHorizon);
    // if the list is not found the latest contact phase is considered
    if (finalPhase == contactPhaseList.end())
    {
        finalPhase = std::prev(contactPhaseList.end());
        return false;
    }

    int index = 0;
    for (auto it = initialPhase; it != std::next(finalPhase); std::advance(it, 1))
    {
        const std::chrono::nanoseconds tInitial = std::max(m_pimpl->currentTime, it->beginTime);
        const std::chrono::nanoseconds tFinal = std::min(absoluteTimeHorizon, it->endTime);

        const int numberOfSamples = (tFinal - tInitial) / m_pimpl->optiSettings.samplingTime;

        for (const auto& [key, contact] : it->activeContacts)
        {
            using namespace BipedalLocomotion::Conversions;

            auto inputContact = inputs.contacts.find(key);
            if (inputContact == inputs.contacts.end())
            {
                log()->error("{} Unable to find the input contact named {}.", errorPrefix, key);
                return false;
            }

            toEigen(*(inputContact->second.nominalPosition))
                .middleCols(index, numberOfSamples + 1)
                .colwise()
                = contact->pose.translation();

            // this is required to reshape the matrix into a vector
            const Eigen::Matrix3d orientation = contact->pose.quat().toRotationMatrix();
            toEigen(*(inputContact->second.orientation)).middleCols(index, numberOfSamples).colwise()
                = Eigen::Map<const Eigen::VectorXd>(orientation.data(), orientation.size());

            constexpr double isEnabled = 1;
            toEigen(*(inputContact->second.isEnabled))
                .middleCols(index, numberOfSamples)
                .setConstant(isEnabled);
        }

        index += numberOfSamples;
    }

    assert(index == m_pimpl->optiSettings.horizon);

    // set the current contact position to for the active contact only
    for (auto& [key, contact] : inputs.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        toEigen(*contact.currentPosition) = toEigen(*contact.nominalPosition).leftCols<1>();

        // if warmstart is enabled the contact location is used as warmstart to initialize the
        // problem
        if (m_pimpl->optiSettings.isWarmStartEnabled)
        {
            toEigen(*(m_pimpl->initialGuess.contactsLocation[key]))
                = toEigen(*contact.nominalPosition);
        }
    }

    // TODO this part can be improved. For instance you do not need to fill the vectors every time.
    for (auto& [key, contact] : inputs.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        const auto& boundingBox = m_pimpl->contactBoundingBoxes.at(key);
        toEigen(*contact.upperLimitPosition).colwise() = boundingBox.upperLimit;
        toEigen(*contact.lowerLimitPosition).colwise() = boundingBox.lowerLimit;
    }

    return true;
}

