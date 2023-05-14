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
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::Contacts;

inline Eigen::Map<Eigen::MatrixXd> toEigen(casadi::DM& input)
{
    return Eigen::Map<Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
}

inline Eigen::Map<const Eigen::MatrixXd> toEigen(const casadi::DM& input)
{
    return Eigen::Map<const Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
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
    bool isInitialized{false};
    std::chrono::nanoseconds currentTime{std::chrono::nanoseconds::zero()};

    CentroidalMPCState state;
    Contacts::ContactPhaseList contactPhaseList;
    Math::LinearizedFrictionCone frictionCone;

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
        casadi::MX isEnable;
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
            this->isEnable = casadi::MX::sym(contactName + "_is_enable");

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
                                        (https://coin-or.github.io/Ipopt/OPTIONS.html) */
        int ipoptMaxIteration{3000}; /**< Maximum number of iteration */

        int horizon; /**<Number of samples used in the horizon */
        std::chrono::nanoseconds samplingTime; /**< Sampling time of the planner */
        std::chrono::nanoseconds timeHorizon; /**< Duration of the horizon */
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
        casadi::MX externalWrench;
    };
    OptimizationVariables optiVariables; /**< Optimization variables */

    struct ContactsInputs
    {
        casadi::DM currentPosition;
        casadi::DM orientation;
        casadi::DM nominalPosition;
        casadi::DM upperLimitPosition;
        casadi::DM lowerLimitPosition;
        casadi::DM isEnable;
    };
    struct ControllerInputs
    {
        std::map<std::string, ContactsInputs> contacts;

        casadi::DM comReference;
        casadi::DM angularMomentumReference;
        casadi::DM comCurrent;
        casadi::DM dcomCurrent;
        casadi::DM angularMomentumCurrent;
        casadi::DM externalWrench;
    };
    ControllerInputs controllerInputs;

    struct Weights
    {
        Eigen::Vector3d com;
        double contactPosition;
        Eigen::Vector3d forceRateOfChange;
        double angularMomentum;
    };
    Weights weights; /**< Settings */

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
        constexpr auto errorPrefix = "[CentroidalMPC::Impl::loadParameters]";

        if (!ptr->getParameter("controller_sampling_time", this->optiSettings.samplingTime))
        {
            log()->error("{} Unable to load the sampling time of the controller.", errorPrefix);
            return false;
        }

        if (!ptr->getParameter("controller_horizon", this->optiSettings.horizon))
        {
            log()->error("{} Unable to load the controller horizon the controller.", errorPrefix);
            return false;
        }

        this->optiSettings.timeHorizon
            = this->optiSettings.horizon * this->optiSettings.samplingTime;

        int numberOfMaximumContacts;
        if (!ptr->getParameter("number_of_maximum_contacts", numberOfMaximumContacts))
        {
            log()->error("{} Unable to load the maximum number of contacts.", errorPrefix);
            return false;
        }

        for (std::size_t i = 0; i < numberOfMaximumContacts; i++)
        {
            auto contactHandler = ptr->getGroup("CONTACT_" + std::to_string(i)).lock();

            if (contactHandler == nullptr)
            {
                log()->error("{} Unable to load the contact {}. Please be sure that CONTACT_{} "
                             "group exists.",
                             errorPrefix,
                             i,
                             i);
                return false;
            }

            std::string contactName;
            if (!contactHandler->getParameter("contact_name", contactName))
            {
                log()->error("{} Unable to get the name of the contact number {}.", errorPrefix, i);
                return false;
            }

            // set the contact name
            this->state.contacts[contactName].name = contactName;

            if (!contactHandler->getParameter("bounding_box_upper_limit",
                                              this->contactBoundingBoxes[contactName].upperLimit))
            {
                log()->error("{} Unable to load the bounding box upper limit of the contact number "
                             "{}.",
                             errorPrefix,
                             i);
                return false;
            }

            if (!contactHandler->getParameter("bounding_box_lower_limit",
                                              this->contactBoundingBoxes[contactName].lowerLimit))
            {
                log()->error("{} Unable to load the bounding box lower limit of the contact number "
                             "{}.",
                             errorPrefix,
                             i);
                return false;
            }

            if (!this->loadContactCorners(contactHandler, this->state.contacts[contactName]))
            {
                log()->error("{} Unable to load the contact corners for the contact {}.",
                             errorPrefix,
                             i);
                return false;
            }
        }

        if (!ptr->getParameter("linear_solver", this->optiSettings.ipoptLinearSolver))
        {
            log()->info("{} 'linear_solver' not found. The default parameter will be used: {}.",
                        errorPrefix,
                        this->optiSettings.ipoptLinearSolver);
        }

        if (!ptr->getParameter("ipopt_tolerance", this->optiSettings.ipoptTolerance))
        {
            log()->info("{} 'ipopt_tolerance' not found. The default parameter will be used: {}.",
                        errorPrefix,
                        this->optiSettings.ipoptTolerance);
        }

        if (!ptr->getParameter("ipopt_max_iteration", this->optiSettings.ipoptMaxIteration))
        {
            log()->info("{} 'ipopt_max_iteration' not found. The default parameter will be used: "
                        "{}.",
                        errorPrefix,
                        this->optiSettings.ipoptMaxIteration);
        }

        if (!ptr->getParameter("solver_verbosity", this->optiSettings.solverVerbosity))
        {
            log()->info("{} 'solver_verbosity' not found. The default parameter will be used: {}.",
                        errorPrefix,
                        this->optiSettings.solverVerbosity);
        }


        bool ok = true;
        ok = ok && ptr->getParameter("com_weight", this->weights.com);
        ok = ok && ptr->getParameter("contact_position_weight", this->weights.contactPosition);
        ok = ok
             && ptr->getParameter("force_rate_of_change_weight", this->weights.forceRateOfChange);
        ok = ok && ptr->getParameter("angular_momentum_weight", this->weights.angularMomentum);

        // initialize the friction cone
        ok = ok && frictionCone.initialize(ptr);

        if (!ok)
        {
            log()->error("{} Unable to load the weight of the cost function.", errorPrefix);
            return false;
        }

        return true;
    }

    casadi::Function ode()
    {
        // Convert DiscreteGeometryContact into a casadiContact object
        std::map<std::string, CasadiContact> casadiContacts;

        for (const auto& [key, contact] : this->state.contacts)
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

        casadi::MX ddcom = casadi::MX::sym("ddcom", 3);
        casadi::MX angularMomentumDerivative = casadi::MX::sym("angular_momentum_derivative", 3);

        casadi::DM gravity = casadi::DM::zeros(3);
        gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

        ddcom = gravity + externalForce / mass;
        angularMomentumDerivative = casadi::DM::zeros(3);

        std::vector<casadi::MX> input;
        input.push_back(externalForce);
        input.push_back(com);
        input.push_back(dcom);
        input.push_back(angularMomentum);

        for (const auto& [key, contact] : casadiContacts)
        {
            input.push_back(contact.position);
            input.push_back(contact.orientation);
            input.push_back(contact.isEnable);
            input.push_back(contact.linearVelocity);

            for (const auto& corner : contact.corners)
            {
                using namespace casadi;
                ddcom += contact.isEnable / mass * corner.force;
                angularMomentumDerivative
                    += contact.isEnable
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
            rhs.push_back(contact.position + (1 - contact.isEnable) * contact.linearVelocity * dT);
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

        // prepare the controller inputs struct
        this->controllerInputs.comCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.dcomCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.angularMomentumCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.comReference = casadi::DM::zeros(vector3Size, stateHorizon);
        this->controllerInputs.angularMomentumReference = casadi::DM::zeros(vector3Size, stateHorizon);
        this->controllerInputs.externalWrench = casadi::DM::zeros(vector3Size, //
                                                                  this->optiSettings.horizon);

        for (const auto& [key, contact] : this->state.contacts)
        {
            // The current position of the contact
            this->controllerInputs.contacts[key].currentPosition = casadi::DM::zeros(vector3Size);

            // The orientation is stored as a vectorized version of the rotation matrix
            this->controllerInputs.contacts[key].orientation
                = casadi::DM::zeros(9, this->optiSettings.horizon);

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].upperLimitPosition
                = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].lowerLimitPosition
                = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

            // Maximum admissible contact force. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].isEnable
                = casadi::DM::zeros(1, this->optiSettings.horizon);

            // The nominal contact position is a parameter that regularize the solution
            this->controllerInputs.contacts[key].nominalPosition
                = casadi::DM::zeros(vector3Size, stateHorizon);
        }
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
        for (const auto& [key, contact] : this->state.contacts)
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
            c.isEnable = this->opti.parameter(1, this->optiSettings.horizon);

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
        this->optiVariables.angularMomentumReference = this->opti.parameter(vector3Size, stateHorizon);
        this->optiVariables.externalWrench = this->opti.parameter(vector3Size, //
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
        ipoptOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;
        casadiOptions["expand"] = true;

        this->opti.solver("ipopt", casadiOptions, ipoptOptions);
    }

    casadi::Function createController()
    {
        using Sl = casadi::Slice;

        this->populateOptiVariables();

        //
        auto& com = this->optiVariables.com;
        auto& dcom = this->optiVariables.dcom;
        auto& angularMomentum = this->optiVariables.angularMomentum;
        auto& externalWrench = this->optiVariables.externalWrench;

        // prepare the input of the ode
        std::vector<casadi::MX> odeInput;
        odeInput.push_back(externalWrench);
        odeInput.push_back(com(Sl(), Sl(0, -1)));
        odeInput.push_back(dcom(Sl(), Sl(0, -1)));
        odeInput.push_back(angularMomentum(Sl(), Sl(0, -1)));
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            odeInput.push_back(contact.position(Sl(), Sl(0, -1)));
            odeInput.push_back(contact.orientation);
            odeInput.push_back(contact.isEnable);
            odeInput.push_back(contact.linearVelocity);

            for (const auto& corner : contact.corners)
            {
                odeInput.push_back(corner.force);
            }
        }

        // set the initial values
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
        // Assumption: the matrices as stored as column-major
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

        // (max - mix) * expo + mi n
        casadi::DM weightCoMZ = casadi::DM::zeros(1, com.columns());
        double min = this->weights.com(2) / 2;
        for (int i = 0; i < com.columns(); i++)
        {
            weightCoMZ(Sl(), i) = (this->weights.com(2) - min) * std::exp(-i) + min;
        }

        casadi::MX cost
            = this->weights.angularMomentum  * casadi::MX::sumsqr(angularMomentum - angularMomentumReference)
              + this->weights.com(0) * casadi::MX::sumsqr(com(0, Sl()) - comReference(0, Sl()))
              + this->weights.com(1) * casadi::MX::sumsqr(com(1, Sl()) - comReference(1, Sl()))
              + casadi::MX::sumsqr(weightCoMZ * (com(2, Sl()) - comReference(2, Sl())));

        casadi::MX averageForce;
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            cost += this->weights.contactPosition
                    * casadi::MX::sumsqr(contact.nominalPosition - contact.position);

            averageForce = casadi::MX::vertcat(
                {contact.isEnable * contact.corners[0].force(0, Sl()) / contact.corners.size(),
                 contact.isEnable * contact.corners[0].force(1, Sl()) / contact.corners.size(),
                 contact.isEnable * contact.corners[0].force(2, Sl()) / contact.corners.size()});
            for (int i = 1; i < contact.corners.size(); i++)
            {
                averageForce += casadi::MX::vertcat(
                    {contact.isEnable * contact.corners[i].force(0, Sl()) / contact.corners.size(),
                     contact.isEnable * contact.corners[i].force(1, Sl()) / contact.corners.size(),
                     contact.isEnable * contact.corners[i].force(2, Sl())
                         / contact.corners.size()});
            }

            for (const auto& corner : contact.corners)
            {
                auto forceRateOfChange = casadi::MX::diff(corner.force.T()).T();

                cost += 10 * casadi::MX::sumsqr(corner.force - averageForce);

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

        input.push_back(this->optiVariables.externalWrench);
        input.push_back(this->optiVariables.comCurrent);
        input.push_back(this->optiVariables.dcomCurrent);
        input.push_back(this->optiVariables.angularMomentumCurrent);
        input.push_back(this->optiVariables.comReference);
        input.push_back(this->optiVariables.angularMomentumReference);
        input.push_back(this->optiVariables.com);
        input.push_back(this->optiVariables.angularMomentum);

        inputName.push_back("external_wrench");
        inputName.push_back("com_current");
        inputName.push_back("dcom_current");
        inputName.push_back("angular_momentum_current");
        inputName.push_back("com_reference");
        inputName.push_back("angular_momentum_reference");
        inputName.push_back("com");
        inputName.push_back("angular_momentum");

        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            input.push_back(contact.currentPosition);
            input.push_back(contact.nominalPosition);
            input.push_back(contact.orientation);
            input.push_back(contact.isEnable);
            input.push_back(contact.upperLimitPosition);
            input.push_back(contact.lowerLimitPosition);


            inputName.push_back("contact_" + key + "_current_position");
            inputName.push_back("contact_" + key + "_nominal_position");
            inputName.push_back("contact_" + key + "_orientation_in");
            inputName.push_back("contact_" + key + "_is_enable_in");
            inputName.push_back("contact_" + key + "_upper_limit_position");
            inputName.push_back("contact_" + key + "_lower_limit_position");

            output.push_back(contact.isEnable);
            output.push_back(contact.position);
            output.push_back(contact.orientation);

            outputName.push_back("contact_" + key + "_is_enable");
            outputName.push_back("contact_" + key + "_position");
            outputName.push_back("contact_" + key + "_orientation");

            std::size_t cornerIndex = 0;
            for (const auto& corner : contact.corners)
            {
                output.push_back(corner.force);
                outputName.push_back("contact_" + key + "_corner_" + std::to_string(cornerIndex)
                                     + "_force");
                cornerIndex++;
            }
        }

        // controller:(com_current[3], dcom_current[3], angular_momentum_current[3],
        // com_reference[3x16], contact_left_foot_current_position[3],
        // contact_left_foot_nominal_position[3x16], contact_left_foot_orientation[9x15],
        // contact_left_foot_is_enable[1x15], contact_left_foot_upper_limit_position[3x15],
        // contact_left_foot_lower_limit_position[3x15], contact_right_foot_current_position[3],
        // contact_right_foot_nominal_position[3x16], contact_right_foot_orientation[9x15],
        // contact_right_foot_is_enable[1x15], contact_right_foot_upper_limit_position[3x15],
        // contact_right_foot_lower_limit_position[3x15]) -------------------->
        // (contact_left_foot_position[3x16],
        // contact_left_foot_is_enable[1x15], contact_left_foot_corner_0_force[3x15],
        // contact_left_foot_corner_1_force[3x15], contact_left_foot_corner_2_force[3x15],
        // contact_left_foot_corner_3_force[3x15], contact_right_foot_position[3x16],
        // contact_right_foot_is_enable[1x15], contact_right_foot_corner_0_force[3x15],
        // contact_right_foot_corner_1_force[3x15], contact_right_foot_corner_2_force[3x15],
        // contact_right_foot_corner_3_force[3x15])

        // casadi::Dict options;
        // options["jit"] = true;
        // options["compiler"] = "shell";

        // casadi::Dict jit_options;
        // jit_options["flags"] = {"-O3"};
        // jit_options["verbose"] = true;

        // options["jit_options"] = jit_options;

        return this->opti.to_function("controller", input, output, inputName, outputName);
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

    m_pimpl->resizeControllerInputs();
    m_pimpl->controller = m_pimpl->createController();
    m_pimpl->isInitialized = true;

    return true;
}

CentroidalMPC::~CentroidalMPC() = default;

CentroidalMPC::CentroidalMPC()
{
    m_pimpl = std::make_unique<Impl>();
}

const CentroidalMPCState& CentroidalMPC::getOutput() const
{
    return m_pimpl->state;
}

bool CentroidalMPC::isOutputValid() const
{
    return true;
}

bool CentroidalMPC::advance()
{
    constexpr auto errorPrefix = "[CentroidalMPC::advance]";
    assert(m_pimpl);

    using Sl = casadi::Slice;

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    // controller:(com_current[3], dcom_current[3], angular_momentum_current[3],
    // com_reference[3x16],
    //            contact_left_foot_current_position[3],  contact_left_foot_nominal_position[3x16],
    //            contact_left_foot_orientation[9x15], contact_left_foot_is_enable[1x15],
    //            contact_left_foot_upper_limit_position[3x15],
    //            contact_left_foot_lower_limit_position[3x15],
    //            contact_right_foot_current_position[3], contact_right_foot_nominal_position[3x16],
    //            contact_right_foot_orientation[9x15], contact_right_foot_is_enable[1x15],
    //            contact_right_foot_upper_limit_position[3x15],
    //            contact_right_foot_lower_limit_position[3x15])
    //             -------------------->
    //            (contact_left_foot_position[3x16], contact_left_foot_is_enable[1x15],
    //            contact_left_foot_corner_0_force[3x15],
    //                                                                                  contact_left_foot_corner_1_force[3x15],
    //                                                                                  contact_left_foot_corner_2_force[3x15],
    //                                                                                  contact_left_foot_corner_3_force[3x15],
    //             contact_right_foot_position[3x16], contact_right_foot_is_enable[1x15],
    //             contact_right_foot_corner_0_force[3x15],
    //                                                                                    contact_right_foot_corner_1_force[3x15],
    //                                                                                    contact_right_foot_corner_2_force[3x15],
    //                                                                                    contact_right_foot_corner_3_force[3x15])

    const auto& inputs = m_pimpl->controllerInputs;

    std::vector<casadi::DM> vectorizedInputs;
    vectorizedInputs.push_back(inputs.externalWrench);
    vectorizedInputs.push_back(inputs.comCurrent);
    vectorizedInputs.push_back(inputs.dcomCurrent);
    vectorizedInputs.push_back(inputs.angularMomentumCurrent);
    vectorizedInputs.push_back(inputs.comReference);
    vectorizedInputs.push_back(inputs.angularMomentumReference);
    vectorizedInputs.push_back(inputs.comReference);
    vectorizedInputs.push_back(inputs.angularMomentumReference);

    for (const auto& [key, contact] : inputs.contacts)
    {
        vectorizedInputs.push_back(contact.currentPosition);
        vectorizedInputs.push_back(contact.nominalPosition);
        vectorizedInputs.push_back(contact.orientation);
        vectorizedInputs.push_back(contact.isEnable);
        vectorizedInputs.push_back(contact.upperLimitPosition);
        vectorizedInputs.push_back(contact.lowerLimitPosition);
    }

    // compute the output
    auto controllerOutput = m_pimpl->controller(vectorizedInputs);

    // get the solution
    auto it = controllerOutput.begin();
    m_pimpl->state.nextPlannedContact.clear();
    for (auto& [key, contact] : m_pimpl->state.contacts)
    {
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

        double isEnable = toEigen(*it)(0);
        std::advance(it, 1);
        contact.pose.translation(toEigen(*it).leftCols<1>());

        if (index < size)
        {
            m_pimpl->state.nextPlannedContact[key].name = key;
            m_pimpl->state.nextPlannedContact[key].pose.translation(toEigen(*it).col(index));
        }

        std::advance(it, 1);
        contact.pose.quat(Eigen::Quaterniond(
            Eigen::Map<const Eigen::Matrix3d>(toEigen(*it).leftCols<1>().data())));

        if (index < size)
        {
            m_pimpl->state.nextPlannedContact[key].pose.quat(Eigen::Quaterniond(
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

            // log()->warn("[CentroidalMPC] key {} next planned contact pose {} activation time {} "
            //             "deactivation time {} next planned contact time {} index {}",
            //             key,
            //             m_pimpl->state.nextPlannedContact[key].pose.translation().transpose(),
            //             nextPlannedContact->activationTime,
            //             nextPlannedContact->deactivationTime,
            //             nextPlannedContactTime,
            //             index);

            auto& contact = m_pimpl->state.nextPlannedContact[key];
            contact.activationTime = nextPlannedContact->activationTime;
            contact.deactivationTime = nextPlannedContact->deactivationTime;
            contact.index = nextPlannedContact->index;
            contact.type = nextPlannedContact->type;
        }

        std::advance(it, 1);

        for (auto& corner : contact.corners)
        {
            // isEnable == 1 means that the contact is active
            if (isEnable > 0.5)
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
    return true;
}

bool CentroidalMPC::setReferenceTrajectory(Eigen::Ref<const Eigen::Matrix3Xd> com, Eigen::Ref<const Eigen::Matrix3Xd> angularMomentum)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setReferenceTrajectory]";
    assert(m_pimpl);

    const int stateHorizon = m_pimpl->optiSettings.horizon + 1;

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    if (com.cols() < stateHorizon)
    {
        log()->error("{} The CoM matrix should have at least {} columns. The number of columns is "
                     "equal to the horizon you set in the  initialization phase.",
                     errorPrefix,
                     m_pimpl->optiSettings.horizon);
        return false;
    }

    if (angularMomentum.cols() < stateHorizon)
    {
        log()->error("{} The angular momentum matrix should have at least {} columns. The number of columns is "
                     "equal to the horizon you set in the  initialization phase.",
                     errorPrefix,
                     m_pimpl->optiSettings.horizon);
        return false;
    }

    toEigen(m_pimpl->controllerInputs.comReference) = com.leftCols(stateHorizon);
    toEigen(m_pimpl->controllerInputs.angularMomentumReference) = angularMomentum.leftCols(stateHorizon);

    return true;
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum,
                             std::optional<Eigen::Ref<const Eigen::Vector3d>> externalWrench)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setState]";
    assert(m_pimpl);

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    auto& inputs = m_pimpl->controllerInputs;
    toEigen(inputs.comCurrent) = com;
    toEigen(inputs.dcomCurrent) = dcom;
    toEigen(inputs.angularMomentumCurrent) = angularMomentum;

    toEigen(inputs.externalWrench).setZero();
    m_pimpl->state.externalWrench = Eigen::Vector3d::Zero();

    if (externalWrench)
    {
        toEigen(inputs.externalWrench).leftCols<1>() = externalWrench.value();
        m_pimpl->state.externalWrench = externalWrench.value();
    }

    return true;
}

bool CentroidalMPC::setContactPhaseList(const Contacts::ContactPhaseList& contactPhaseList)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setContactPhaseList]";
    assert(m_pimpl);

    if (!m_pimpl->isInitialized)
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
    for (const auto& [key, contact] : m_pimpl->state.contacts)
    {
        // initialize the current contact pose to zero. If the contact is active the current
        // position will be set later on
        toEigen(inputs.contacts[key].currentPosition).setZero();

        // initialize all the orientation to the identity
        toEigen(inputs.contacts[key].orientation).colwise()
            = Eigen::Map<const Eigen::VectorXd>(identity.data(), identity.cols() * identity.rows());

        // Upper limit of the position of the contact. It is expressed in the contact body frame
        toEigen(inputs.contacts[key].upperLimitPosition).setZero();

        // Lower limit of the position of the contact. It is expressed in the contact body frame
        toEigen(inputs.contacts[key].lowerLimitPosition).setZero();

        // Maximum admissible contact force. It is expressed in the contact body frame
        toEigen(inputs.contacts[key].isEnable).setZero();

        // The nominal contact position is a parameter that regularize the solution
        toEigen(inputs.contacts[key].nominalPosition).setZero();
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
        // TODO check what it's going on if t horizon is greater then last endtime
        const std::chrono::nanoseconds tInitial = std::max(m_pimpl->currentTime, it->beginTime);
        const std::chrono::nanoseconds tFinal = std::min(absoluteTimeHorizon, it->endTime);

        const int numberOfSamples = (tFinal - tInitial) / m_pimpl->optiSettings.samplingTime;

        for (const auto& [key, contact] : it->activeContacts)
        {
            auto inputContact = inputs.contacts.find(key);
            if (inputContact == inputs.contacts.end())
            {
                log()->error("{} Unable to find the input contact named {}.", errorPrefix, key);
                return false;
            }

            toEigen(inputContact->second.nominalPosition)
                .middleCols(index, numberOfSamples + 1)
                .colwise()
                = contact->pose.translation();

            // this is required to reshape the matrix into a vector
            const Eigen::Matrix3d orientation = contact->pose.quat().toRotationMatrix();
            toEigen(inputContact->second.orientation).middleCols(index, numberOfSamples).colwise()
                = Eigen::Map<const Eigen::VectorXd>(orientation.data(), orientation.size());

            constexpr double maxForce = 1;
            toEigen(inputContact->second.isEnable)
                .middleCols(index, numberOfSamples)
                .setConstant(maxForce);
        }

        index += numberOfSamples;
    }

    assert(index == m_pimpl->optiSettings.horizon);

    // set the current contact position to for the active contact only
    for (auto& [key, contact] : inputs.contacts)
    {
        toEigen(contact.currentPosition) = toEigen(contact.nominalPosition).leftCols<1>();
    }

    // for (const auto& [key, contact] : inputs.contacts)
    // {
    //     log()->info("{} current Pose {}:", key, toEigen(contact.currentPosition).transpose());
    //     log()->info("{} nominal Pose {}:", key, toEigen(contact.nominalPosition));
    //     log()->info("{} maximumnormalforce  {}", key, toEigen(contact.isEnable));
    // }

    // TODO this part can be improved. For instance you do not need to fill the vectors every time.
    for (auto& [key, contact] : inputs.contacts)
    {
        const auto& boundingBox = m_pimpl->contactBoundingBoxes.at(key);
        toEigen(contact.upperLimitPosition).colwise() = boundingBox.upperLimit;
        toEigen(contact.lowerLimitPosition).colwise() = boundingBox.lowerLimit;
    }

    return true;
}
