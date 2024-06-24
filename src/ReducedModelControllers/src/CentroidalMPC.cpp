/**
 * @file CentroidalMPC.cpp
 * @authors Giulio Romualdi
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
#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::Contacts;

#define STR_(x) #x
#define STR(x) STR_(x)

bool casadiVersionIsAtLeast360()
{
    std::string str;
    std::stringstream ss(STR(casadi_VERSION));

    // Use while loop to check the getline() function condition.
    int index = 0;
    while (getline(ss, str, '.'))
    {
        if (index == 0 && stoi(str) < 3)
        {
            return false;
        }
        if (index == 1 && stoi(str) < 6)
        {
            return false;
        }
        index++;
    }

    return true;
}

inline double chronoToSeconds(const std::chrono::nanoseconds& d)
{
    return std::chrono::duration<double>(d).count();
}

std::vector<std::string> extractVariablesName(const std::vector<casadi::MX>& variables)
{
    std::vector<std::string> variablesName;
    variablesName.reserve(variables.size());
    for (const auto& variable : variables)
    {
        variablesName.push_back(variable.name());
    }

    return variablesName;
}

template <class T> inline auto extractFutureValuesFromState(T& variable)
{
    using Sl = casadi::Slice;
    return variable(Sl(), Sl(1, variable.columns()));
}

template <class T> inline auto extractFutureValuesFromState(const T& variable)
{
    using Sl = casadi::Slice;
    return variable(Sl(), Sl(1, variable.columns()));
}

struct CentroidalMPC::Impl
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

    struct CasadiCorner
    {
        casadi::DM position;
        casadi::MX force;

        casadi::MX isEnabled;
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
            this->isEnabled = casadi::MX::sym(cornerName + "_is_enabled");

            return *this;
        }
    };

    struct CasadiContact
    {
        casadi::MX position;
        casadi::MX linearVelocity;
        casadi::MX orientation;
        casadi::MX isEnabled;
        casadi::MX isNotMovable;
        casadi::MX amountOfNormalForceRespectToRobotWeight;
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
            this->isNotMovable = casadi::MX::sym(contactName + "_is_not_movable");
            this->amountOfNormalForceRespectToRobotWeight
                = casadi::MX::sym(contactName + "_amount_of_normal_force_respect_to_robot_weight");

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
        bool errorOnFail{true}; /**< True if the user wants to throw an error in case of failure of
                                   the MPC */
        double ipoptTolerance{1e-8}; /**< Tolerance of ipopt
                                        (https://coin-or.github.io/Ipopt/OPTIONS.html#OPT_tol) */
        int ipoptMaxIteration{3000}; /**< Maximum number of iteration */

        int horizon; /**<Number of samples used in the horizon */
        int adjustmentPreventionKnots{0}; /**< Number of knots used to prevent the adjustment of the
                                             contact position before the contact enabling*/
        std::chrono::nanoseconds samplingTime; /**< Sampling time of the planner */
        std::chrono::nanoseconds timeHorizon; /**< Duration of the horizon */
        bool isWarmStartEnabled{false}; /**< True if the user wants to warm start the CoM, angular
                                           momentum and contact. */
        bool isCseEnabled{false}; /**< True if the Common subexpression elimination casadi option is
                                       enabled. */

        std::string solverName{"ipopt"}; /**< Name of the solver used by the MPC. */
        bool isJITEnabled{false}; /**< True if the JIT compilation is enabled. */
        int numberOfQPIterations{10}; /**< Number of QP iteration. */
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
        casadi::DM* isNotMovable;
        casadi::DM* amountOfNormalForceRespectToRobotWeight;
        std::vector<casadi::DM*> isCornerEnabled;
        double previousAmountOfNormalForceRespectToRobotWeight{0.5};
        int lastTrajectoryNumberOfIndexForSmoothing{5};
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

    struct ContactInitialGuess
    {
        casadi::DM* contactLocation;
        std::vector<casadi::DM*> contactForce;
    };

    struct InitialGuess
    {
        std::map<std::string, ContactInitialGuess> contactsInitialGuess;

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
        double force;
    };
    Weights weights;

    struct ContactBoundingBox
    {
        Eigen::Vector3d upperLimit;
        Eigen::Vector3d lowerLimit;
    };

    std::unordered_map<std::string, ContactBoundingBox> contactBoundingBoxes;

    struct CoMLimits
    {
        int cbfHorizon{0};
        double cbfGain{0.0};
        double cbfMultiplier{0.0};
        double zMin{0.0};
        double zMax{0.0};
        bool enableZLimit{false};
    };

    CoMLimits comLimits;

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
        ok = ok && getParameter(ptr, "force_weight", this->weights.force);

        // initialize the friction cone
        ok = ok && frictionCone.initialize(ptr);
        ok = ok && getParameter(ptr, "solver_name", this->optiSettings.solverName);
        ok = ok && getParameter(ptr, "enable_z_limit", this->comLimits.enableZLimit);

        if (!ok)
        {
            return false;
        }

        if (this->comLimits.enableZLimit)
        {
            if (!getParameter(ptr, "com_z_min", this->comLimits.zMin))
            {
                return false;
            }
            if (!getParameter(ptr, "com_z_max", this->comLimits.zMax))
            {
                return false;
            }

            if (this->comLimits.zMin > this->comLimits.zMax)
            {
                log()->error("{} The minimum value of the CoM z limit is greater than the maximum "
                             "value. The minimum value is {} and the maximum value is {}.",
                             logPrefix,
                             this->comLimits.zMin,
                             this->comLimits.zMax);
                return false;
            }

            if (!getParameter(ptr, "cbf_horizon", this->comLimits.cbfHorizon))
            {
                return false;
            }

            if (!getParameter(ptr, "cbf_gain", this->comLimits.cbfGain))
            {
                return false;
            }

            if (!getParameter(ptr, "cbf_multiplier", this->comLimits.cbfMultiplier))
            {
                return false;
            }

            // cbf gain and multiplier must be positive and lower than 1
            if (this->comLimits.cbfGain < 0 || this->comLimits.cbfGain > 1)
            {
                log()->error("{} The gain of the CoM z limit is negative or greater than 1. The "
                             "gain is {}.",
                             logPrefix,
                             this->comLimits.cbfGain);
                return false;
            }

            if (this->comLimits.cbfHorizon < 0)
            {
                log()->error("{} The horizon of the CoM z limit is negative. The horizon is {}.",
                             logPrefix,
                             this->comLimits.cbfHorizon);
                return false;
            }
        }

        if (this->optiSettings.solverName != "ipopt" && this->optiSettings.solverName != "sqp")
        {
            log()->error("{} The solver name '{}' is not supported. The supported solvers are "
                         "'ipopt' and 'sqp'.",
                         logPrefix,
                         this->optiSettings.solverName);
            return false;
        }

        std::chrono::nanoseconds adjustmentPreventionTimeDelta{0};
        getOptionalParameter(ptr, "adjustment_prevention_time", adjustmentPreventionTimeDelta);

        this->optiSettings.adjustmentPreventionKnots
            = adjustmentPreventionTimeDelta / this->optiSettings.samplingTime;

        if (this->optiSettings.solverName == "ipopt")
        {
            getOptionalParameter(ptr, "linear_solver", this->optiSettings.ipoptLinearSolver);
            getOptionalParameter(ptr, "ipopt_tolerance", this->optiSettings.ipoptTolerance);
            getOptionalParameter(ptr, "ipopt_max_iteration", this->optiSettings.ipoptMaxIteration);
        } else
        {
            getOptionalParameter(ptr,
                                 "number_of_qp_iterations",
                                 this->optiSettings.numberOfQPIterations);
        }

        getOptionalParameter(ptr, "error_on_fail", this->optiSettings.errorOnFail);
        getOptionalParameter(ptr, "jit_compilation", this->optiSettings.isJITEnabled);
        getOptionalParameter(ptr, "solver_verbosity", this->optiSettings.solverVerbosity);
        getOptionalParameter(ptr, "is_warm_start_enabled", this->optiSettings.isWarmStartEnabled);
        getOptionalParameter(ptr, "is_cse_enabled", this->optiSettings.isCseEnabled);

        return ok;
    }

    /**
     * Centroidal dynamics and contact position function.
     * @note the function has the following inputs:
     *     1. externalForce
     *     2. externalTorque
     *     3. com
     *     4. dcom
     *     5. angularMomentum
     *     6. for each contact:
     *        1. position
     *        2. orientation
     *        3. isEnabled
     *        4. isNotMovable
     *        5. linearVelocity
     *        6. for each corner:
     *           1. force
     *           2. isEnabled
     * @note the function has the following outputs:
     *     1. com
     *     2. dcom
     *     3. angularMomentum
     *     4. for each contact:
     *        1. position
     * @return the casadi function
     */
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
            input.push_back(contact.isNotMovable);
            input.push_back(contact.linearVelocity);

            for (const auto& corner : contact.corners)
            {
                using namespace casadi;
                ddcom += corner.isEnabled / mass * corner.force;
                angularMomentumDerivative
                    += corner.isEnabled
                       * MX::cross(MX::mtimes(MX::reshape(contact.orientation, 3, 3),
                                              corner.position)
                                       + contact.position - com,
                                   corner.force);

                input.push_back(corner.force);
                input.push_back(corner.isEnabled);
            }
        }

        const double dT = chronoToSeconds(this->optiSettings.samplingTime);

        std::vector<std::string> outputName{"com", "dcom", "angular_momentum"};
        std::vector<casadi::MX> rhs{com + dcom * dT,
                                    dcom + ddcom * dT,
                                    angularMomentum + angularMomentumDerivative * dT};

        for (const auto& [key, contact] : casadiContacts)
        {
            rhs.push_back(contact.position
                          + (1 - contact.isNotMovable) * contact.linearVelocity * dT);
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

    /**
     * Resize the controller inputs.
     * @note this function vectorized the controller input variables. The order of the variables
     * must match the one required by the createController function. There is no check that ensure
     * this so is up to the programmer to ensure that the order is correct.
     */
    bool resizeControllerInputs()
    {
        constexpr int vector3Size = 3;
        const int stateHorizon = this->optiSettings.horizon + 1;

        // resize the CoM Trajectory
        this->output.comTrajectory.resize(stateHorizon);
        this->output.comVelocityTrajectory.resize(stateHorizon);
        this->output.angularMomentumTrajectory.resize(stateHorizon);

        // In case of no warmstart the variables are:
        // - centroidalVariables = 7: external force + external torque + com current + dcom current
        //                            + current angular momentum + com reference
        //                            + angular momentum reference
        // - contactVariables = 8: for each contact we have current position + nominal position
        //                          + orientation + is enabled + is movable
        //                          + amount of normal force respect to robot weight
        //                          + upper limit in position
        //                          + lower limit in position
        // - cornerVariables = 1: is enabled
        constexpr std::size_t centroidalVariables = 7;
        constexpr std::size_t contactVariables = 8;

        // compute the corner variables
        std::size_t cornerVariables = 0;
        for (const auto& [key, contact] : this->output.contacts)
        {
            for (const auto& corner : contact.corners)
            {
                // for each corner we need to propagate if the corner is enabled. An enable corner
                // means that the corner of the contact is in contact with the environment
                cornerVariables += 1;
            }
        }

        std::size_t vectorizedOptiInputsSize = centroidalVariables //
                                               + (this->output.contacts.size() * contactVariables)
                                               + cornerVariables;

        if (this->optiSettings.isWarmStartEnabled)
        {
            // in this case we need to add the com, the angular momentum and the contact location
            constexpr std::size_t centroidalVariablesWarmStart = 2;
            constexpr std::size_t contactVariablesWarmStart = 1;
            vectorizedOptiInputsSize += centroidalVariablesWarmStart + //
                                        (this->output.contacts.size() * contactVariablesWarmStart);

            for (const auto& [key, contact] : this->output.contacts)
            {
                for (const auto& corner : contact.corners)
                {
                    // for each corner we have the force
                    vectorizedOptiInputsSize += 1;
                }
            }
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

            // Possibility to not move a contact (in general this should be equal to isEnabled
            // however we want to avoid to move a contact when the foot is next to the ground)
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(1, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].isNotMovable = &this->vectorizedOptiInputs.back();

            // The amount of normal force respect to the robot weight
            this->vectorizedOptiInputs.push_back(casadi::DM::zeros(1, this->optiSettings.horizon));
            this->controllerInputs.contacts[key].amountOfNormalForceRespectToRobotWeight
                = &this->vectorizedOptiInputs.back();

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

            // variable useful to enable or disable the corner (i.e., the corner of the contact
            // is in contact with the environment)
            for (const auto& corner : contact.corners)
            {
                this->vectorizedOptiInputs.push_back(
                    casadi::DM::zeros(1, this->optiSettings.horizon));
                this->controllerInputs.contacts[key].isCornerEnabled.push_back(
                    &this->vectorizedOptiInputs.back());
            }

            if (this->optiSettings.isWarmStartEnabled)
            {
                this->vectorizedOptiInputs.push_back(casadi::DM::zeros(vector3Size, stateHorizon));
                this->initialGuess.contactsInitialGuess[key].contactLocation
                    = &this->vectorizedOptiInputs.back();

                for (const auto& corner : contact.corners)
                {
                    this->vectorizedOptiInputs.push_back(
                        casadi::DM::zeros(vector3Size, this->optiSettings.horizon));
                    this->initialGuess.contactsInitialGuess[key].contactForce.push_back(
                        &this->vectorizedOptiInputs.back());
                }
            }
        }

        if (vectorizedOptiInputsSize != this->vectorizedOptiInputs.size())
        {
            log()->error("[CentroidalMPC::Impl::resizeControllerInputs] The size of the vectorized "
                         "opti inputs is not correct. The expected size is {} and the actual size "
                         "is {}. This is a bug. Please report it.",
                         vectorizedOptiInputsSize,
                         this->vectorizedOptiInputs.size());
            return false;
        }

        return true;
    }

    void populateOptiVariables()
    {
        constexpr int vector3Size = 3;
        const int stateHorizon = this->optiSettings.horizon + 1;

        // create the variables for the state
        this->optiVariables.com = this->opti.variable(vector3Size, stateHorizon);
        this->optiVariables.dcom = this->opti.variable(vector3Size, stateHorizon);
        this->optiVariables.angularMomentum = this->opti.variable(vector3Size, stateHorizon);

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

            // Possibility to not move a contact (in general this should be equal to isEnabled
            // however we want to avoid to move a contact when the foot is next to the ground)
            c.isNotMovable = this->opti.parameter(1, this->optiSettings.horizon);

            // The amount of normal force respect to the robot weight
            c.amountOfNormalForceRespectToRobotWeight
                = this->opti.parameter(1, this->optiSettings.horizon);

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
                c.corners[j].isEnabled = this->opti.parameter(1, this->optiSettings.horizon);
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
        casadi::Dict solverOptions;
        if (this->optiSettings.solverName == "ipopt")
        {
            if (this->optiSettings.solverVerbosity != 0)
            {
                casadi_int ipoptVerbosity
                    = static_cast<long long>(optiSettings.solverVerbosity - 1);
                solverOptions["print_level"] = ipoptVerbosity;
                casadiOptions["print_time"] = true;
            } else
            {
                solverOptions["print_level"] = 0;
                casadiOptions["print_time"] = false;
            }

            solverOptions["max_iter"] = this->optiSettings.ipoptMaxIteration;
            solverOptions["tol"] = this->optiSettings.ipoptTolerance;
            solverOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;
            casadiOptions["expand"] = true;
            casadiOptions["error_on_fail"] = this->optiSettings.errorOnFail;

            if (this->optiSettings.isJITEnabled)
            {
                casadiOptions["jit"] = true;
                casadiOptions["compiler"] = "shell";

                casadi::Dict jitOptions;
                jitOptions["flags"] = {"-O3"};
                jitOptions["verbose"] = true;
                casadiOptions["jit_options"] = jitOptions;
            }

            this->opti.solver("ipopt", casadiOptions, solverOptions);
            return;
        }

        // if not ipopt it is sqpmethod
        casadi::Dict osqpOptions;
        if (this->optiSettings.solverVerbosity != 0)
        {
            casadiOptions["print_header"] = true;
            casadiOptions["print_iteration"] = true;
            casadiOptions["print_status"] = true;
            casadiOptions["print_time"] = true;
            osqpOptions["verbose"] = true;
        } else
        {
            casadiOptions["print_header"] = false;
            casadiOptions["print_iteration"] = false;
            casadiOptions["print_status"] = false;
            casadiOptions["print_time"] = false;
            osqpOptions["verbose"] = false;
        }
        casadiOptions["error_on_fail"] = this->optiSettings.errorOnFail;
        casadiOptions["expand"] = true;
        casadiOptions["qpsol"] = "osqp";

        solverOptions["error_on_fail"] = this->optiSettings.errorOnFail;

        osqpOptions["verbose"] = false;
        solverOptions["osqp"] = osqpOptions;

        casadiOptions["qpsol_options"] = solverOptions;
        casadiOptions["max_iter"] = this->optiSettings.numberOfQPIterations;

        if (this->optiSettings.isJITEnabled)
        {
            casadiOptions["jit"] = true;
            casadiOptions["compiler"] = "shell";

            casadi::Dict jitOptions;
            jitOptions["flags"] = {"-O3"};
            jitOptions["verbose"] = true;
            casadiOptions["jit_options"] = jitOptions;
        }
        this->opti.solver("sqpmethod", casadiOptions);
    }

    /**
     * Create the controller. The controller is a casadi function that takes several input and solve
     * the optimization problem.
     * @note The input of the controller are:
     * 1. external_force
     * 2. external_torque
     * 3. com_current
     * 4. dcom_current
     * 5. angular_momentum_current
     * 6. com_reference
     * 7. angular_momentum_reference
     * 8. if warm start is enabled:
     *   a. com_warmstart
     *   b. angular_momentum_warmstart
     * 9. for each contact:
     *   a. contact_current_position
     *   b. contact_nominal_position
     *   c. contact_orientation
     *   d. contact_is_enable
     *   e. contact_amount_of_normal_force_respect_to_robot_weight
     *   f. contact_upper_limit_position
     *   g. contact_lower_limit_position
     *   h. for each corner:
     *     - contact_corner_is_enable
     *   i. if warm start is enabled:
     *      -. contact_position_warmstart
     *      -. for each corner:
     *        []. contact_corner_force_warmstart
     * @note The output of the controller are:
     * 1. for each contact:
     *   a. contact_is_enable
     *   b. contact_position
     *   c. contact_orientation
     *   d. for each corner:
     *     -. contact_corner_is_enable
     *     -. contact_corner_force
     * 2. com
     * 3. dcom
     * 4. angular_momentum
     * @return the casadi function
     */
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

        // prepare the input of the ode as vector. The order is the Impl::ode described in the ode
        // function documentation
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
            odeInput.push_back(contact.isNotMovable);
            odeInput.push_back(contact.linearVelocity);

            for (const auto& corner : contact.corners)
            {
                odeInput.push_back(corner.force);
                odeInput.push_back(corner.isEnabled);
            }
        }

        // set the feedback
        this->opti.subject_to(this->optiVariables.comCurrent == com(Sl(), 0));
        this->opti.subject_to(this->optiVariables.dcomCurrent == dcom(Sl(), 0));
        this->opti.subject_to(this->optiVariables.angularMomentumCurrent
                              == angularMomentum(Sl(), 0));
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

        if (this->comLimits.enableZLimit)
        {
            auto h = -this->comLimits.cbfMultiplier * (com(2, Sl()) - this->comLimits.zMin)
                     * (com(2, Sl()) - this->comLimits.zMax);
            for (int i = 0; i < std::min(this->comLimits.cbfHorizon, this->optiSettings.horizon);
                 i++)
            {
                this->opti.subject_to(h(i + 1) + (this->comLimits.cbfGain - 1) * h(i) >= 0);
            }
        }

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
                {contact.corners[0].isEnabled * contact.corners[0].force(0, Sl())
                     / contact.corners.size(),
                 contact.corners[0].isEnabled * contact.corners[0].force(1, Sl())
                     / contact.corners.size(),
                 contact.corners[0].isEnabled * contact.corners[0].force(2, Sl())
                     / contact.corners.size()});
            for (int i = 1; i < contact.corners.size(); i++)
            {
                averageForce += casadi::MX::vertcat(
                    {contact.corners[i].isEnabled * contact.corners[i].force(0, Sl())
                         / contact.corners.size(),
                     contact.corners[i].isEnabled * contact.corners[i].force(1, Sl())
                         / contact.corners.size(),
                     contact.corners[i].isEnabled * contact.corners[i].force(2, Sl())
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
                cost += this->weights.force
                        * casadi::MX::sumsqr(
                            corner.force(2, Sl())
                            - BipedalLocomotion::Math::StandardAccelerationOfGravitation
                                  * contact.amountOfNormalForceRespectToRobotWeight
                                  / contact.corners.size());
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

        // if warm start is enabled we need to add the initial guess for the com and the angular
        // momentum
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
            concatenateInput(contact.isNotMovable, "contact_" + key + "_is_not_movable");
            concatenateInput(contact.amountOfNormalForceRespectToRobotWeight,
                             "contact_" + key + "_amount_of_normal_force_respect_to_robot_weight");
            concatenateInput(contact.upperLimitPosition,
                             "contact_" + key + "_upper_limit_position");
            concatenateInput(contact.lowerLimitPosition,
                             "contact_" + key + "_lower_limit_position");

            std::size_t cornerIndex = 0;
            for (const auto& corner : contact.corners)
            {
                concatenateInput(corner.isEnabled,
                                 "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                     + "_is_enable_in");
                cornerIndex++;
            }

            // if warm start is enabled we need to add the initial guess for the contact position
            // and the force
            if (this->optiSettings.isWarmStartEnabled)
            {
                concatenateInput(contact.position, "contact_" + key + "_position_warmstart");

                std::size_t cornerIndex = 0;
                for (const auto& corner : contact.corners)
                {
                    concatenateInput(corner.force,
                                     "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                         + "_force_warmstart");
                    cornerIndex++;
                }
            }

            concatenateOutput(contact.isEnabled, "contact_" + key + "_is_enable");
            concatenateOutput(contact.position, "contact_" + key + "_position");
            concatenateOutput(contact.orientation, "contact_" + key + "_orientation");

            cornerIndex = 0;
            for (const auto& corner : contact.corners)
            {
                concatenateOutput(corner.isEnabled,
                                  "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                      + "_is_enable");
                concatenateOutput(corner.force,
                                  "contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                      + "_force");
                cornerIndex++;
            }
        }

        concatenateOutput(this->optiVariables.com, "com");
        concatenateOutput(this->optiVariables.dcom, "dcom");
        concatenateOutput(this->optiVariables.angularMomentum, "angular_momentum");

        casadi::Dict toFunctionOptions, jitOptions;
        if (casadiVersionIsAtLeast360())
        {
            toFunctionOptions["cse"] = this->optiSettings.isCseEnabled;
        }

        return this->opti
            .to_function("controller", input, output, inputName, outputName, toFunctionOptions);
    }
};

bool CentroidalMPC::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
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

    if (!m_pimpl->resizeControllerInputs())
    {
        log()->error("{} Unable to resize the controller inputs.", errorPrefix);
        return false;
    }

    // Create the controller. The controller is a casadi function that takes several input and solve
    // the optimization problem. The input serialization is described in the Impl::createController
    // documentation.
    m_pimpl->controller = m_pimpl->createController();
    m_pimpl->fsm = Impl::FSM::Initialized;

    return true;
}

CentroidalMPC::~CentroidalMPC() = default;

CentroidalMPC::CentroidalMPC()
{
    m_pimpl = std::make_unique<Impl>();
}

const CentroidalMPCOutput& CentroidalMPC::getOutput() const
{
    return m_pimpl->output;
}

bool CentroidalMPC::isOutputValid() const
{
    return m_pimpl->fsm == Impl::FSM::OutputValid;
}

bool CentroidalMPC::advance()
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

    // we now deserialize the output. The order is specified in documentation of
    // Impl::createController
    auto isActive = [](double activeValue) -> bool { return activeValue > 0.5; };
    auto findIndexActiveContact = [&](casadi::DMVector::iterator activeContactVector) -> int {
        // this is required for toEigen
        using namespace BipedalLocomotion::Conversions;

        if (isActive(toEigen(*activeContactVector)(0)))
        {
            return 0;
        }

        int index = toEigen(*activeContactVector).size();
        const int size = toEigen(*activeContactVector).size();

        // find the index of the first element where the contact is active and the previous contact
        // was not active
        for (int i = 1; i < size; i++)
        {
            // read it as: "if the contact is active at a given time instant"
            if (isActive(toEigen(*activeContactVector)(i)))
            {
                // in this case we break if the contact is active and at the previous time
                // step it was not active
                if (!isActive(toEigen(*activeContactVector)(i - 1)))
                {
                    index = i;
                    break;
                }
            }
        }
        return index;
    };

    auto it = controllerOutput.begin();
    ContactListMap contactListMap = m_pimpl->output.contactPhaseList.lists();
    for (auto& [key, contact] : m_pimpl->output.contacts)
    {
        using namespace BipedalLocomotion::Conversions;
        ContactList& contactList = contactListMap.at(key);
        int activeContactVectorSize = toEigen(*it).size();

        const int index = findIndexActiveContact(it);
        //////////// Position
        std::advance(it, 1);
        contact.pose.translation(toEigen(*it).leftCols<1>());

        // In this case the contact is not active and there will be a next planned contact
        if (index < activeContactVectorSize)
        {
            const std::chrono::nanoseconds nextPlannedContactTime
                = m_pimpl->currentTime + m_pimpl->optiSettings.samplingTime * index;

            auto nextPlannedContact = contactList.getPresentContact(nextPlannedContactTime);
            if (nextPlannedContact == contactList.end())
            {
                log()->error("[CentroidalMPC::advance] Unable to get the next planned contact");
                return false;
            }

            PlannedContact modifiedNextPlannedContact = *nextPlannedContact;

            // only the position is modified by the MPC
            modifiedNextPlannedContact.pose.translation(toEigen(*it).col(index));

            if (!contactList.editContact(nextPlannedContact, modifiedNextPlannedContact))
            {
                log()->error("{} Unable to edit the next planned contact at time {}. The contact "
                             "list contains the following contacts: {}",
                             errorPrefix,
                             std::chrono::duration_cast<std::chrono::milliseconds>(
                                 nextPlannedContactTime),
                             contactList.toString());
                return false;
            }
        }

        ////// Orientation
        std::advance(it, 1);
        contact.pose.quat(Eigen::Quaterniond(
            Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

        ///// Get the forces (for each corner we check if is active and then we get the force. If
        /// not active the force is set to zero)
        std::advance(it, 1);
        for (std::size_t cornerIndex = 0; cornerIndex < contact.corners.size(); cornerIndex++)
        {
            // check if the corner is active
            const bool isCornerActive = isActive(toEigen(*it)(0));

            // get the force
            std::advance(it, 1);
            if (m_pimpl->optiSettings.isWarmStartEnabled)
            {
                toEigen(*m_pimpl->initialGuess.contactsInitialGuess[key].contactForce[cornerIndex])
                    .leftCols(m_pimpl->optiSettings.horizon - 1)
                    = toEigen(*it).rightCols(m_pimpl->optiSettings.horizon - 1);
                toEigen(*m_pimpl->initialGuess.contactsInitialGuess[key].contactForce[cornerIndex])
                    .rightCols<1>()
                    = toEigen(*it).rightCols<1>();
            }
            if (isCornerActive)
            {
                contact.corners[cornerIndex].force = toEigen(*it).leftCols<1>();
            } else
            {
                contact.corners[cornerIndex].force.setZero();
            }

            // we move it for the next iteration
            std::advance(it, 1);
        }
    }

    // update the contact phase list
    m_pimpl->output.contactPhaseList.setLists(contactListMap);

    ///// com
    for (int i = 0; i < m_pimpl->output.comTrajectory.size(); i++)
    {
        using namespace BipedalLocomotion::Conversions;
        m_pimpl->output.comTrajectory[i] = toEigen(*it).col(i);
    }

    ///// dcom
    std::advance(it, 1);
    for (int i = 0; i < m_pimpl->output.comVelocityTrajectory.size(); i++)
    {
        using namespace BipedalLocomotion::Conversions;
        m_pimpl->output.comVelocityTrajectory[i] = toEigen(*it).col(i);
    }

    ////// angular
    std::advance(it, 1);
    for (int i = 0; i < m_pimpl->output.angularMomentumTrajectory.size(); i++)
    {
        using namespace BipedalLocomotion::Conversions;
        m_pimpl->output.angularMomentumTrajectory[i] = toEigen(*it).col(i);
    }

    // advance the time
    m_pimpl->currentTime += m_pimpl->optiSettings.samplingTime;

    // Make the output valid
    m_pimpl->fsm = Impl::FSM::OutputValid;

    return true;
}

bool CentroidalMPC::setReferenceTrajectory(const std::vector<Eigen::Vector3d>& com,
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

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum)
{
    const Math::Wrenchd dummy = Math::Wrenchd::Zero();
    return this->setState(com, dcom, angularMomentum, dummy);
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                             const std::unordered_map<std::string, std::vector<bool>>& cornerStatus)
{
    const Math::Wrenchd dummy = Math::Wrenchd::Zero();
    return this->setState(com, dcom, angularMomentum, dummy, cornerStatus);
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                             const Math::Wrenchd& externalWrench,
                             const std::unordered_map<std::string, std::vector<bool>>& cornerStatus
                             /** = {}*/)
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

    for (const auto& [key, status] : cornerStatus)
    {
        auto contact = inputs.contacts.find(key);
        if (contact == inputs.contacts.end())
        {
            std::string contactNames;
            for (const auto& [key, _] : inputs.contacts)
            {
                contactNames += "'" + key + "' ";
            }

            log()->error("{} Unable to find the contact named {}. The contact saved in the "
                         "controller are: {}",
                         errorPrefix,
                         key,
                         contactNames);
            return false;
        }

        using namespace BipedalLocomotion::Conversions;
        auto contactIsEnabledVector(toEigen(*(contact->second.isEnabled)));

        // is disabled
        if (contactIsEnabledVector(0, 0) < 0.5)
        {
            continue;
        }
        for (std::size_t i = 0; i < status.size(); i++)
        {
            auto cornerIsEnabledVector(toEigen(*(contact->second.isCornerEnabled[i])));

            // find the index of the first element where the contact not active (lower than 0.5)
            int index = cornerIsEnabledVector.size();
            for (int j = 1; j < cornerIsEnabledVector.size(); j++)
            {
                if (cornerIsEnabledVector(0, j) < 0.5)
                {
                    index = j;
                    break;
                }
            }

            cornerIsEnabledVector.middleCols(0, index - 1).setConstant(status[i] ? 1.0 : 0.0);
        }
    }

    return true;
}

bool CentroidalMPC::setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList)
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

    for (const auto& [key, list] : contactPhaseList.lists())
    {
        if (!list.areContactsSampled(m_pimpl->optiSettings.samplingTime))
        {
            log()->error("{} The contact list {} is not sampled at the sampling time {}. Please "
                         "resample the contacts lists before calling this method.",
                         errorPrefix,
                         key,
                         std::chrono::duration_cast<std::chrono::milliseconds>(
                             m_pimpl->optiSettings.samplingTime));
            return false;
        }
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

        // The contact is movable if the contact is active
        toEigen(*inputs.contacts[key].isNotMovable).setZero();

        // Amount of normal force respect to the robot weight
        toEigen(*inputs.contacts[key].amountOfNormalForceRespectToRobotWeight).setZero();

        // The nominal contact position is a parameter that regularize the solution
        toEigen(*inputs.contacts[key].nominalPosition).setZero();

        for (auto& cornerEnabled : inputs.contacts[key].isCornerEnabled)
        {
            toEigen(*cornerEnabled).setZero();
        }
    }

    const std::chrono::nanoseconds absoluteTimeHorizon
        = m_pimpl->currentTime + m_pimpl->optiSettings.timeHorizon;

    // find the contactPhase associated to the current time
    auto initialPhase = contactPhaseList.getPresentPhase(m_pimpl->currentTime);
    if (initialPhase == contactPhaseList.end())
    {
        log()->error("{} Unable to find the contact phase related to the current at time {}. The "
                     "contact "
                     "list contains the following contacts: {}",
                     errorPrefix,
                     std::chrono::duration_cast<std::chrono::milliseconds>(m_pimpl->currentTime),
                     contactPhaseList.toString());
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

        const std::chrono::nanoseconds duration = tFinal - tInitial;
        const int numberOfSamples = duration / m_pimpl->optiSettings.samplingTime;

        const int numberOfActiveContacts = it->activeContacts.size();

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

            // set if the contact is movable or not. If the contact is active the contact is not
            // movable moreover we want that the contact is not movable at least 3 sample before the
            // beginning of the phase
            toEigen(*(inputContact->second.isNotMovable))
                .middleCols(index, numberOfSamples)
                .setConstant(isEnabled);

            if (this->m_pimpl->optiSettings.adjustmentPreventionKnots > 0)
            {
                int indexFromWhichTheContactCannotBeModified
                    = std::max(0, index - m_pimpl->optiSettings.adjustmentPreventionKnots);
                int numberOfSamplesFromWhichTheContactCannotBeModified
                    = index - indexFromWhichTheContactCannotBeModified;

                toEigen(*(inputContact->second.isNotMovable))
                    .middleCols(indexFromWhichTheContactCannotBeModified,
                                numberOfSamplesFromWhichTheContactCannotBeModified)
                    .setConstant(isEnabled);

                toEigen(*(inputContact->second.nominalPosition))
                    .middleCols(indexFromWhichTheContactCannotBeModified,
                                numberOfSamplesFromWhichTheContactCannotBeModified)
                    .colwise()
                    = contact->pose.translation();

                // this is required to reshape the matrix into a vector
                const Eigen::Matrix3d orientation = contact->pose.quat().toRotationMatrix();
                toEigen(*(inputContact->second.orientation))
                    .middleCols(indexFromWhichTheContactCannotBeModified,
                                numberOfSamplesFromWhichTheContactCannotBeModified)
                    .colwise()
                    = Eigen::Map<const Eigen::VectorXd>(orientation.data(), orientation.size());
            }

            toEigen(*(inputContact->second.amountOfNormalForceRespectToRobotWeight))
                .middleCols(index, numberOfSamples)
                .setConstant(1.0 / double(numberOfActiveContacts));

            // Set the corner to be enabled by default we assume that the contact is active
            // TODO(GR) if we want to consider planned contact where some corners are not active we
            // need to change the following for loop and the definition of planned contact.
            for (auto& cornerEnabled : inputContact->second.isCornerEnabled)
            {
                toEigen(*cornerEnabled).middleCols(index, numberOfSamples).setConstant(isEnabled);
            }
        }

        index += numberOfSamples;
    }

    // smooth the nominal amount of normal force respect to the robot weight
    // TD(GR) this is a temporary solution and it is valid only for bipedal robots. A more general
    // solution should be implemented
    for (auto& [key, contact] : inputs.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        auto amountOfNormalForceRespectToRobotWeight
            = toEigen(*contact.amountOfNormalForceRespectToRobotWeight);

        // the trajectory is composed by the following values:
        // - 0 if the contact is not active
        // - 1 if is the only contact active
        // - 1 / numberOfActiveContacts if there are more than one contact active
        // First of all I need to find the index of all the trajectories where there are more than
        // one contact active
        std::vector<int> indexes;
        for (int i = 0; i < amountOfNormalForceRespectToRobotWeight.cols(); i++)
        {
            if (amountOfNormalForceRespectToRobotWeight(0, i) > 0.1
                && amountOfNormalForceRespectToRobotWeight(0, i) < 0.9)
            {
                indexes.push_back(i);
            }
        }

        // now I need to split the indexes in groups of consecutive indexes
        std::vector<std::vector<int>> groups;
        if (indexes.size() > 0)
        {
            groups.push_back({indexes[0]});
            for (int i = 1; i < indexes.size(); i++)
            {
                if (indexes[i] == indexes[i - 1] + 1)
                {
                    groups.back().push_back(indexes[i]);
                } else
                {
                    groups.push_back({indexes[i]});
                }
            }
        }

        for (const auto& group : groups)
        {

            // print the group
            // std::string elements = "";
            // std::string groupValues = "";
            // for (const auto& i : group)
            // {
            //     elements += std::to_string(i) + " ";
            //     groupValues += std::to_string(amountOfNormalForceRespectToRobotWeight(0, i)) + " ";
            // }
            // log()->info("Contact {} - Group: {}", key, elements);
            // log()->info("Contact {} - Group Values: {}", key, groupValues);

            // there are several cases:
            // - the first element of the group is the first element of the trajectory but not the
            // last
            // - the last element of the group is the last element of the trajectory but not the
            // first
            // - the first element of the group is the first element of the trajectory and the last
            // is the last element of the trajectory
            // - the group is in the middle of the trajectory

            // the first element of the group is the first element of the trajectory but not the
            // last
            if (group.front() == 0
                && group.back() != amountOfNormalForceRespectToRobotWeight.cols() - 1)
            {
                const double endValue
                    = amountOfNormalForceRespectToRobotWeight(0, group.back() + 1);
                const double initialValue = contact.previousAmountOfNormalForceRespectToRobotWeight;
                int numberOfSamples = group.size() + 2; // we get the first and the last element

                for (int i = 0; i < group.size(); i++)
                {
                    amountOfNormalForceRespectToRobotWeight(0, group[i])
                        = endValue * ((i + 1) / double(numberOfSamples))
                          + initialValue * (1 - (i + 1) / double(numberOfSamples));
                }
            }

            // the last element of the group is the last element of the trajectory but not the first
            if (group.front() != 0
                && group.back() == amountOfNormalForceRespectToRobotWeight.cols() - 1)
            {
                const double initialValue
                    = amountOfNormalForceRespectToRobotWeight(0, group.front() - 1);

                // we smooth the first 5 samples
                int amountOfSamplesToSmooth = (group.size() < 6 ? group.size() - 1 : 5) + group.front();
                const double endValue = amountOfNormalForceRespectToRobotWeight(0, amountOfSamplesToSmooth);
                int numberOfSamples = group.size() + 2; // we get the first and the last element

                for (int i = 0; i < group.size(); i++)
                {
                    amountOfNormalForceRespectToRobotWeight(0, group[i])
                        = endValue * ((i + 1) / double(numberOfSamples))
                          + initialValue * (1 - (i + 1) / double(numberOfSamples));
                }
            }

            // the first element of the group is the first element of the trajectory and the last is
            // the last element of the trajectory
            if (group.front() == 0
                && group.back() == amountOfNormalForceRespectToRobotWeight.cols() - 1)
            {
                const double initialValue = contact.previousAmountOfNormalForceRespectToRobotWeight;
                const double endValue = amountOfNormalForceRespectToRobotWeight(
                    0, contact.lastTrajectoryNumberOfIndexForSmoothing);
                int numberOfSamples = group.size() + 2; // we get the first and the last element

                for (int i = 0; i < group.size(); i++)
                {
                    amountOfNormalForceRespectToRobotWeight(0, group[i])
                        = endValue * ((i + 1) / double(numberOfSamples))
                          + initialValue * (1 - (i + 1) / double(numberOfSamples));
                }

                contact.lastTrajectoryNumberOfIndexForSmoothing--;
                if (contact.lastTrajectoryNumberOfIndexForSmoothing == 0)
                {
                    contact.lastTrajectoryNumberOfIndexForSmoothing = 5;
                }
            }

            // the group is in the middle of the trajectory
            if (group.front() != 0
                && group.back() != amountOfNormalForceRespectToRobotWeight.cols() - 1)
            {
                const double initialValue
                    = amountOfNormalForceRespectToRobotWeight(0, group.front() - 1);
                const double endValue
                    = amountOfNormalForceRespectToRobotWeight(0, group.back() + 1);
                int numberOfSamples = group.size() + 2; // we get the first and the last element

                for (int i = 0; i < group.size(); i++)
                {
                    amountOfNormalForceRespectToRobotWeight(0, group[i])
                        = endValue * ((i + 1) / double(numberOfSamples))
                          + initialValue * (1 - (i + 1) / double(numberOfSamples));
                }
            }
        }

        // update the previous value
        contact.previousAmountOfNormalForceRespectToRobotWeight
            = amountOfNormalForceRespectToRobotWeight(0, 0);
    }

    // print the isEnable vector
    for (const auto& [key, contact] : inputs.contacts)
    {
        using namespace BipedalLocomotion::Conversions;

        log()->info("Contact: {}", key);
        log()->info("E: {}", toEigen(*contact.isEnabled));
        log()->info("M: {}", toEigen(*contact.isNotMovable));
        log()->info("N: {}", toEigen(*contact.amountOfNormalForceRespectToRobotWeight));
        // log()->info("P {}", toEigen(*contact.nominalPosition));
        // log()->info("O {}", toEigen(*contact.orientation));
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
            toEigen(*(m_pimpl->initialGuess.contactsInitialGuess[key].contactLocation))
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

    // we store the contact phase list for the output
    m_pimpl->output.contactPhaseList = contactPhaseList;

    return true;
}
