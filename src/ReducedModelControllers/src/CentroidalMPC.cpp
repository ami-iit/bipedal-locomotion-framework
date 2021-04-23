/**
 * @file CentroidalMPC.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <casadi/casadi.hpp>

#include <BipedalLocomotion/ReducedModelControllers/CentroidalMPC.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::ReducedModelControllers;
using namespace BipedalLocomotion::Contacts;

Eigen::Map<Eigen::MatrixXd> toEigen(casadi::DM& input)
{
    return Eigen::Map<Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
}

Eigen::Map<const Eigen::MatrixXd> toEigen(const casadi::DM& input)
{
    return Eigen::Map<const Eigen::MatrixXd>(input.ptr(), input.rows(), input.columns());
}

struct CentroidalMPC::Impl
{
    casadi::Opti opti; /**< CasADi opti stack */
    casadi::Function controller;
    bool isInitialized{false};
    double currentTime{0};

    CentroidalMPCState state;

    Math::LinearizedFrictionCone frictionCone;

    struct CasadiCorner
    {
        casadi::DM position;
        casadi::MX force;

        CasadiCorner() = default;
        CasadiCorner(const Corner& other)
        {
            this->operator=(other);
        }

        CasadiCorner& operator=(const Corner& other)
        {
            this->position.resize(3, 1);
            this->position(0,0) = other.position(0);
            this->position(1,0) = other.position(1);
            this->position(2,0) = other.position(2);

            this->force = casadi::MX::sym("force", other.force.size(), 1);

            return *this;
        }
    };

    struct CasadiContact
    {
        casadi::MX position;
        casadi::MX orientation;
        std::vector<CasadiCorner> corners;

        CasadiContact() = default;

        CasadiContact& operator=(const ContactWithCorners& other)
        {
            corners.resize(other.corners.size());

            for (int i = 0; i < other.corners.size(); i++)
            {
                this->corners[i] = other.corners[i];
            }

            this->orientation = casadi::MX::sym("orientation", 3, 3);
            this->position = casadi::MX::sym("position", 3, 1);

            return *this;
        }

        CasadiContact(const ContactWithCorners& other)
        {
            this->operator=(other);
        }
    };


    struct CasadiContactWithConstraints : CasadiContact
    {
        casadi::MX nominalPosition;
        casadi::MX upperLimitPosition;
        casadi::MX lowerLimitPosition;
        casadi::MX limitVelocity;
        casadi::MX maximumNormalForce;
    };

    struct OptimizationSettings
    {
        unsigned long solverVerbosity{1}; /**< Verbosity of ipopt */
        std::string ipoptLinearSolver{"mumps"}; /**< Linear solved used by ipopt */
        double ipoptTolerance{1e-8}; /**< Tolerance of ipopt
                                        (https://coin-or.github.io/Ipopt/OPTIONS.html) */

        double samplingTime; /**< Sampling time of the planner in seconds */
        int horizon; /**<Number of samples used in the horizon */
        double timeHorizon;
    };

    OptimizationSettings optiSettings; /**< Settings */

    struct OptimizationSolution
    {
        std::unique_ptr<casadi::OptiSol> solution; /**< Pointer to the solution of the optimization
                                                      problem */
        casadi::DM dcm; /**< Optimal DCM trajectory */
        casadi::DM omega; /**< Optimal omega trajectory */
        casadi::DM vrp; /**< Optimal VRP trajectory */
        casadi::DM omegaDot; /**< Optimal omega rate of change trajectory */
    };
    OptimizationSolution optiSolution; /**< Solution of the optimization problem */


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
        casadi::MX comCurrent;
        casadi::MX dcomCurrent;
        casadi::MX angularMomentumCurrent;
    };
    OptimizationVariables optiVariables; /**< Optimization variables */


    struct ContactsInputs
    {
        casadi::DM orientation;
        casadi::DM nominalPosition;
        casadi::DM upperLimitPosition;
        casadi::DM lowerLimitPosition;
        casadi::DM limitVelocity;
        casadi::DM maximumNormalForce;
    };
    struct ControllerInputs
    {
        std::map<std::string, ContactsInputs> contacts;

        casadi::DM comReference;
        casadi::DM comCurrent;
        casadi::DM dcomCurrent;
        casadi::DM angularMomentumCurrent;
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

    bool loadContactCorners(std::shared_ptr<const ParametersHandler::IParametersHandler> ptr,
                            ContactWithCorners& contact)
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

        this->optiSettings.timeHorizon = this->optiSettings.horizon * this->optiSettings.samplingTime;

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
            log()->info("{} The default linear solver will be used: {}.",
                        errorPrefix,
                        this->optiSettings.ipoptLinearSolver);
        }

        if (!ptr->getParameter("ipopt_tolerance", this->optiSettings.ipoptTolerance))
        {
            log()->info("{} The default ipopt tolerance will be used: {}.",
                        errorPrefix,
                        this->optiSettings.ipoptTolerance);
        }

        bool ok = true;
        ok = ok && ptr->getParameter("com_weight", this->weights.com);
        ok = ok && ptr->getParameter("contact_position_weight", this->weights.contactPosition);
        ok = ok && ptr->getParameter("force_rate_of_change_weight", this->weights.forceRateOfChange);
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
        // Convert BipedalLocomotion::ReducedModelControllers::ContactWithCorners into a
        // casadiContact object
        std::map<std::string, CasadiContact> casadiContacts(this->state.contacts.begin(),
                                                            this->state.contacts.end());

        // we assume mass equal to 1
        constexpr double mass = 1;

        casadi::MX com = casadi::MX::sym("com", 3);
        casadi::MX dcom = casadi::MX::sym("dcom", 3);
        casadi::MX ddcom = casadi::MX::sym("ddcom", 3);
        casadi::MX angularMomentum = casadi::MX::sym("angular_momentum", 3);
        casadi::MX angularMomentumDerivative = casadi::MX::sym("angular_momentum_derivative", 3);

        casadi::DM gravity = casadi::DM::zeros(3);
        gravity(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;

        ddcom = gravity;
        angularMomentumDerivative = casadi::DM::zeros(3);

        std::vector<casadi::MX> input;
        input.push_back(com);
        input.push_back(dcom);
        input.push_back(angularMomentum);

        for(const auto& [key, contact] : casadiContacts)
        {
            input.push_back(contact.position);
            input.push_back(casadi::MX::reshape(contact.orientation, 9, 1));
            for(const auto& corner : contact.corners)
            {

                ddcom += 1/mass * casadi::MX::mtimes(contact.orientation, corner.force);

                angularMomentumDerivative
                    += casadi::MX::cross(casadi::MX::mtimes(contact.orientation, corner.position)
                                             + contact.position - com,
                                         casadi::MX::mtimes(contact.orientation, corner.force));

                input.push_back(corner.force);
            }
        }

        casadi::MX rhs = casadi::MX::vertcat(
            {com + dcom * this->optiSettings.samplingTime,
             dcom + ddcom * this->optiSettings.samplingTime,
             angularMomentum + angularMomentumDerivative * this->optiSettings.samplingTime});

        return casadi::Function("centroidal_dynamics",
                                input,
                                {com + dcom * this->optiSettings.samplingTime,
                                 dcom + ddcom * this->optiSettings.samplingTime,
                                 angularMomentum
                                     + angularMomentumDerivative
                                           * this->optiSettings.samplingTime});
    }


    casadi::Function contactPositionError()
    {
        casadi::MX contactPosition = casadi::MX::sym("contact_position", 3);
        casadi::MX nominalContactPosition = casadi::MX::sym("nominal_contact_position", 3);
        casadi::MX contactOrientation = casadi::MX::sym("contact_orientation", 3 * 3);

        // the orientation is stored as a vectorized version of the matrix. We need to reshape it
        casadi::MX rhs = casadi::MX::mtimes(casadi::MX::reshape(contactOrientation, 3, 3),
                                            contactPosition - nominalContactPosition);

        return casadi::Function("contact_position_error",
                                {contactPosition, nominalContactPosition, contactOrientation},
                                {rhs});
    }

    void resizeControllerInputs()
    {
        constexpr int vector3Size = 3;

        // prepare the controller inputs struct
        this->controllerInputs.comCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.dcomCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.angularMomentumCurrent = casadi::DM::zeros(vector3Size);
        this->controllerInputs.comReference = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

        for (const auto& [key, contact] : this->state.contacts)
        {
            // The orientation is stored as a vectorized version of the rotation matrix
            this->controllerInputs.contacts[key].orientation
                = casadi::DM::zeros(9, this->optiSettings.horizon);

            // This is the admissible velocity of the contact (It will be different to zero only
            // when a new contact is created)
            this->controllerInputs.contacts[key].limitVelocity
                = casadi::DM::zeros(vector3Size, this->optiSettings.horizon - 1);

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].upperLimitPosition
                = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].lowerLimitPosition
                = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);

            // Maximum admissible contact force. It is expressed in the contact body frame
            this->controllerInputs.contacts[key].maximumNormalForce
                = casadi::DM::zeros(1, this->optiSettings.horizon);

            // The nominal contact position is a parameter that regularize the solution
            this->controllerInputs.contacts[key].nominalPosition
                = casadi::DM::zeros(vector3Size, this->optiSettings.horizon);
        }
    }

    void populateOptiVariables()
    {
        constexpr int vector3Size = 3;

        // create the variables for the state
        this->optiVariables.com = this->opti.variable(vector3Size, this->optiSettings.horizon + 1);
        this->optiVariables.dcom = this->opti.variable(vector3Size, this->optiSettings.horizon + 1);
        this->optiVariables.angularMomentum
            = this->opti.variable(vector3Size, this->optiSettings.horizon + 1);

        // the casadi contacts depends on the maximum number of contacts
        for (const auto& [key, contact] : this->state.contacts)
        {
            // each contact has a different number of corners
            this->optiVariables.contacts[key].corners.resize(contact.corners.size());

            // the position of the contact is an optimization variable
            this->optiVariables.contacts[key].position
                = this->opti.variable(vector3Size, this->optiSettings.horizon);

            // the orientation is a parameter. The orientation is stored as a vectorized version of
            // the rotation matrix
            this->optiVariables.contacts[key].orientation
                = this->opti.parameter(9, this->optiSettings.horizon);

            // This is the admissible velocity of the contact (It will be different to zero only
            // when a new contact is created)
            this->optiVariables.contacts[key].limitVelocity
                = this->opti.parameter(vector3Size, this->optiSettings.horizon - 1);

            // Upper limit of the position of the contact. It is expressed in the contact body frame
            this->optiVariables.contacts[key].upperLimitPosition
                = this->opti.parameter(vector3Size, this->optiSettings.horizon);

            // Lower limit of the position of the contact. It is expressed in the contact body frame
            this->optiVariables.contacts[key].lowerLimitPosition
                = this->opti.parameter(vector3Size, this->optiSettings.horizon);

            // Maximum admissible contact force. It is expressed in the contact body frame
            this->optiVariables.contacts[key].maximumNormalForce
                = this->opti.parameter(1, this->optiSettings.horizon);

            // The nominal contact position is a parameter that regularize the solution
            this->optiVariables.contacts[key].nominalPosition
                = this->opti.parameter(vector3Size, this->optiSettings.horizon);

            for (int j = 0; j < contact.corners.size(); j++)
            {
                this->optiVariables.contacts[key].corners[j].force
                    = this->opti.variable(vector3Size, this->optiSettings.horizon);

                this->optiVariables.contacts[key].corners[j].position = casadi::DM(
                    std::vector<double>(this->state.contacts[key].corners[j].position.data(),
                                        this->state.contacts[key].corners[j].position.data()
                                            + this->state.contacts[key].corners[j].position.size()));
            }
        }

        this->optiVariables.comCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.dcomCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.angularMomentumCurrent = this->opti.parameter(vector3Size);
        this->optiVariables.comReference = this->opti.parameter(vector3Size, this->optiSettings.horizon);
    }

    /**
     * Setup the optimization problem options
     */
    void setupOptiOptions()
    {
        // casadi::Dict casadiOptions;
        // casadi::Dict ipoptOptions;

        // if (this->optiSettings.solverVerbosity != 0)
        // {
        //     casadi_int ipoptVerbosity = static_cast<long long>(optiSettings.solverVerbosity - 1);
        //     // ipoptOptions["print_level"] = ipoptVerbosity;
        //     casadiOptions["print_time"] = false;
        // } else
        // {
        //     // ipoptOptions["print_level"] = 0;
        //     casadiOptions["print_time"] = false;
        // }
        // casadiOptions["qpsol"] = "osqp";
        // // casadiOptions["print_iteration"] = false;
        // // casadiOptions["print_header"] = false;
        // casadiOptions["convexify_strategy"] = "regularize";

        // // ipoptOptions["tol"] = this->optiSettings.ipoptTolerance;
        // casadiOptions["expand"] = true;
        // // casadiOptions["warm"] = true;
        // ipoptOptions["verbose"] = false; //

        // casadiOptions["qpsol_options"] = ipoptOptions;

        // this->opti.solver("sqpmethod", casadiOptions);

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
            casadiOptions["print_time"] = true;
        }
        // casadiOptions["qpsol"] = "qrqp";
        // casadiOptions["print_iteration"] = false;
        // casadiOptions["print_header"] = false;
        // casadiOptions["convexify_strategy"] = "regularize";

        ipoptOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;
        casadiOptions["expand"] = true;
        // casadiOptions["warm"] = true;
        // ipoptOptions["verbose"] = false; //

        // casadiOptions["qpsol_options"] = ipoptOptions;

        this->opti.solver("ipopt", casadiOptions, ipoptOptions);
    }

    casadi::Function createController()
    {
        using Sl = casadi::Slice;

        this->populateOptiVariables();

        std::vector<casadi::MX> controlInput;
        for(const auto& [key, contact] : this->optiVariables.contacts)
        {
            controlInput.push_back(contact.position);
            controlInput.push_back(contact.orientation);

            for(const auto& corner : contact.corners)
            {
                controlInput.push_back(corner.force);
            }
        }

        //
        auto& com = this->optiVariables.com;
        auto& dcom = this->optiVariables.dcom;
        auto& angularMomentum = this->optiVariables.angularMomentum;

        // set the initial values
        this->opti.subject_to(this->optiVariables.comCurrent == com(Sl(), 0));
        this->opti.subject_to(this->optiVariables.dcomCurrent == dcom(Sl(), 0));
        this->opti.subject_to(this->optiVariables.angularMomentumCurrent
                              == angularMomentum(Sl(), 0));

        // system dynamics
        std::vector<casadi::MX> tmp;
        tmp.push_back(com(Sl(), Sl(0, -1)));
        tmp.push_back(dcom(Sl(), Sl(0, -1)));
        tmp.push_back(angularMomentum(Sl(), Sl(0, -1)));
        tmp.insert(tmp.end(), controlInput.begin(), controlInput.end());

        auto dynamics = this->ode().map(this->optiSettings.horizon);

        auto fullTrajectory = dynamics(tmp);
        this->opti.subject_to(com(Sl(), Sl(1, com.columns())) == fullTrajectory[0]);
        this->opti.subject_to(dcom(Sl(), Sl(1, com.columns())) == fullTrajectory[1]);
        this->opti.subject_to(angularMomentum(Sl(), Sl(1, angularMomentum.columns())) == fullTrajectory[2]);

        // add constraints for the contacts
        auto contactPositionErrorMap = this->contactPositionError().map(this->optiSettings.horizon);

        // convert the eigen matrix into casadi
        // please check https://github.com/casadi/casadi/issues/2563 and
        // https://groups.google.com/forum/#!topic/casadi-users/npPcKItdLN8
        // Assumption: the matrices as stored as column-major
        casadi::DM frictionConeMatrix = casadi::DM::zeros(frictionCone.getA().rows(), frictionCone.getA().cols());
        std::memcpy(frictionConeMatrix.ptr(),
                    frictionCone.getA().data(),
                    sizeof(double) * frictionCone.getA().rows() * frictionCone.getA().cols());

        const casadi::DM zero = casadi::DM::zeros(frictionCone.getA().rows(), 1);
        casadi::MX constraintFrictionCone;

        for(const auto& [key, contact] : this->optiVariables.contacts)
        {

            auto error = contactPositionErrorMap({contact.position,
                                                  contact.nominalPosition,
                                                  contact.orientation});

            this->opti.subject_to(contact.lowerLimitPosition <= error[0] <= contact.upperLimitPosition);

            this->opti.subject_to(-contact.limitVelocity
                                  <= casadi::MX::diff(contact.position.T()).T()
                                  <= contact.limitVelocity);

            // TODO please if you want to add heel to toe motion you should define a
            // contact.maximumNormalForce for each corner. At this stage is too premature.
            for (const auto& corner : contact.corners)
            {
                constraintFrictionCone = casadi::MX::mtimes(frictionConeMatrix, corner.force);
                for (int i = 0; i < corner.force.columns(); i++)
                {
                    this->opti.subject_to(constraintFrictionCone(Sl(), i) <= zero);
                }

                // limit on the normal force
                this->opti.subject_to(casadi::DM::zeros(1, corner.force.columns())
                                      <= corner.force(2, Sl()) <= contact.maximumNormalForce);
            }
        }

        // create the cost function
        casadi::MX cost = this->weights.angularMomentum
                              * casadi::MX::sumsqr(this->optiVariables.angularMomentum)
                          + this->weights.com(0)
                                * casadi::MX::sumsqr(com(0, Sl(1, com.columns()))
                                                     - this->optiVariables.comReference(0, Sl()))
                          + this->weights.com(1)
                                * casadi::MX::sumsqr(com(1, Sl(1, com.columns()))
                                                     - this->optiVariables.comReference(1, Sl()))
                          + this->weights.com(2)
                                * casadi::MX::sumsqr(com(2, Sl(1, com.columns()))
                                                     - this->optiVariables.comReference(2, Sl()));

        casadi::MX averageForce;
        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            cost += this->weights.contactPosition
                    * casadi::MX::sumsqr(contact.nominalPosition - contact.position);

            averageForce = contact.corners[0].force / contact.corners.size();
            for (int i = 1; i < contact.corners.size(); i++)
            {
                averageForce += contact.corners[i].force / contact.corners.size();
            }

            for (const auto& corner : contact.corners)
            {
                auto forceRateOfChange = casadi::MX::diff(corner.force.T()).T();

                cost += 10 * casadi::MX::sumsqr(corner.force - averageForce);

                cost += this->weights.forceRateOfChange(0) * casadi::MX::sumsqr(forceRateOfChange(0, Sl()));
                cost += this->weights.forceRateOfChange(1) * casadi::MX::sumsqr(forceRateOfChange(1, Sl()));
                cost += this->weights.forceRateOfChange(2) * casadi::MX::sumsqr(forceRateOfChange(2, Sl()));
            }
        }

        this->opti.minimize(cost);

        this->setupOptiOptions();

        // prepare the casadi function
        std::vector<casadi::MX> input;
        std::vector<casadi::MX> output;

        input.push_back(this->optiVariables.comCurrent);
        input.push_back(this->optiVariables.dcomCurrent);
        input.push_back(this->optiVariables.angularMomentumCurrent);
        input.push_back(this->optiVariables.comReference);

        for (const auto& [key, contact] : this->optiVariables.contacts)
        {
            input.push_back(contact.nominalPosition);
            input.push_back(contact.orientation);
            input.push_back(contact.maximumNormalForce);
            input.push_back(contact.upperLimitPosition);
            input.push_back(contact.lowerLimitPosition);
            input.push_back(contact.limitVelocity);

            output.push_back(contact.position);

            for (const auto& corner : contact.corners)
            {
                output.push_back(corner.force);
            }
        }
        return this->opti.to_function("controller", input, output);
    }
};

bool CentroidalMPC::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[CentroidalMPC::initialize]";
    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The parameter handler is not valid.",  errorPrefix);
        return false;
    }

    if (!m_pimpl->loadParameters(ptr))
    {
        log()->error("{} Unable to load the parameters.",  errorPrefix);
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

    // controller:(i0[3],i1[3],i2[3],i3[3x10],i4[3x10],i5[9x10],i6[1x10],i7[3x10],i8[3x10],i9[3x9],i10[3x10],i11[9x10],i12[1x10],i13[3x10],i14[3x10],i15[3x9])->(o0[3x10],o1[3x10],o2[3x10],o3[3x10],o4[3x10],o5[3x10],o6[3x10],o7[3x10],o8[3x10],o9[3x10])

    const auto& inputs = m_pimpl->controllerInputs;
    // m_pimpl->opti.set_value(m_pimpl->optiVariables.comCurrent, inputs.comCurrent);
    // m_pimpl->opti.set_value(m_pimpl->optiVariables.dcomCurrent, inputs.dcomCurrent);
    // m_pimpl->opti.set_value(m_pimpl->optiVariables.angularMomentumCurrent,
    //                         inputs.angularMomentumCurrent);

    // m_pimpl->opti.set_value(m_pimpl->optiVariables.comReference, inputs.comReference);

    // for (const auto& [key, contact] : inputs.contacts)
    // {
    //     m_pimpl->opti.set_value(m_pimpl->optiVariables.contacts[key].nominalPosition,
    //                             contact.nominalPosition);
    //     m_pimpl->opti.set_value(m_pimpl->optiVariables.contacts[key].orientation,
    //                             contact.orientation);

    //     m_pimpl->opti.set_value(m_pimpl->optiVariables.contacts[key].maximumNormalForce,
    //                             contact.maximumNormalForce);
    //     m_pimpl->opti.set_value(m_pimpl->optiVariables.contacts[key].upperLimitPosition,
    //                             contact.upperLimitPosition);

    //     m_pimpl->opti.set_value(m_pimpl->optiVariables.contacts[key].lowerLimitPosition,
    //                             contact.lowerLimitPosition);

    //     m_pimpl->opti.set_value(m_pimpl->optiVariables.contacts[key].limitVelocity,
    //                             contact.limitVelocity);
    // }

    //
    // m_pimpl->opti.set_initial(m_pimpl->optiVariables.com(Sl(),
    //                                                      Sl(1,
    //                                                         m_pimpl->optiVariables.com.columns())),
    //                           inputs.comReference);

    std::vector<casadi::DM> vectorizedInputs;
    vectorizedInputs.push_back(inputs.comCurrent);
    vectorizedInputs.push_back(inputs.dcomCurrent);
    vectorizedInputs.push_back(inputs.angularMomentumCurrent);
    vectorizedInputs.push_back(inputs.comReference);

    for (const auto & [key, contact] : inputs.contacts)
    {
        vectorizedInputs.push_back(contact.nominalPosition);
        vectorizedInputs.push_back(contact.orientation);
        vectorizedInputs.push_back(contact.maximumNormalForce);
        vectorizedInputs.push_back(contact.upperLimitPosition);
        vectorizedInputs.push_back(contact.lowerLimitPosition);
        vectorizedInputs.push_back(contact.limitVelocity);
    }

    // try
    // {
    //     m_pimpl->optiSolution.solution = std::make_unique<casadi::OptiSol>(m_pimpl->opti.solve());
    // } catch (const std::exception& e)
    // {
    //     log()->error("{} Unable to solve the optimization problem. The following exception has "
    //                  "been thrown by the solver: {}.",
    //                  errorPrefix,
    //                  e.what());

    //     return false;
    // }

    // // get the solution
    // for (auto& [key, contact] : m_pimpl->state.contacts)
    // {
    //     contact.pose.translation(toEigen(
    //         m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.contacts[key].position)));

    //     for (int i = 0; i < contact.corners.size(); i++)
    //     {
    //         contact.corners[i].force = toEigen(m_pimpl->optiSolution.solution->value(
    //             m_pimpl->optiVariables.contacts[key].corners[i].force));
    //     }
    // }

    // compute the output
    auto controllerOutput = m_pimpl->controller(vectorizedInputs);

    // get the solution
    auto it = controllerOutput.begin();
    for (auto & [key, contact] : m_pimpl->state.contacts)
    {
        // the first output is the position of the contact
        contact.pose.translation(toEigen(*it).leftCols<1>());
        std::advance(it, 1);
        for(auto& corner : contact.corners)
        {
            corner.force = toEigen(*it).leftCols<1>();
            std::advance(it, 1);
        }
    }

    // advance the time
    m_pimpl->currentTime += m_pimpl->optiSettings.samplingTime;
    return true;
}

bool CentroidalMPC::setReferenceTrajectory(Eigen::Ref<const Eigen::MatrixXd> com)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setReferenceTrajectory]";
    assert(m_pimpl);

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    if (com.rows() != 3)
    {
        log()->error("{} The CoM matrix should have three rows.", errorPrefix);
        return false;
    }

    if (com.cols() < m_pimpl->optiSettings.horizon)
    {
        log()->error("{} The CoM matrix should have at least {} columns. The number of columns is "
                     "equal to the horizon you set in the  initialization phase.",
                     errorPrefix,
                     m_pimpl->optiSettings.horizon);
        return false;
    }

    toEigen(m_pimpl->controllerInputs.comReference) = com.leftCols(m_pimpl->optiSettings.horizon);

    return true;
}

bool CentroidalMPC::setState(Eigen::Ref<const Eigen::Vector3d> com,
                             Eigen::Ref<const Eigen::Vector3d> dcom,
                             Eigen::Ref<const Eigen::Vector3d> angularMomentum)
{
    constexpr auto errorPrefix = "[CentroidalMPC::setState]";
    assert(m_pimpl);

    if (!m_pimpl->isInitialized)
    {
        log()->error("{} The controller is not initialized please call initialize() method.",
                     errorPrefix);
        return false;
    }

    auto & inputs = m_pimpl->controllerInputs;
    toEigen(inputs.comCurrent) = com;
    toEigen(inputs.dcomCurrent) = dcom;
    toEigen(inputs.angularMomentumCurrent) = angularMomentum;

    return true;
}

bool CentroidalMPC::setContactPhaseList(const Contacts::ContactPhaseList &contactPhaseList)
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

    // clear previous data
    for (const auto& [key, contact] : m_pimpl->state.contacts)
    {

        // The orientation is stored as a vectorized version of the rotation matrix
        toEigen(m_pimpl->controllerInputs.contacts[key].orientation).setZero();

        // M_Pimpl is the admissible velocity of the contact (It will be different to zero only
        // when a new contact is created)
        toEigen(m_pimpl->controllerInputs.contacts[key].limitVelocity).setZero();

        // Upper limit of the position of the contact. It is expressed in the contact body frame
        toEigen(m_pimpl->controllerInputs.contacts[key].upperLimitPosition).setZero();

        // Lower limit of the position of the contact. It is expressed in the contact body frame
        toEigen(m_pimpl->controllerInputs.contacts[key].lowerLimitPosition).setZero();

        // Maximum admissible contact force. It is expressed in the contact body frame
        toEigen(m_pimpl->controllerInputs.contacts[key].maximumNormalForce).setZero();

        // The nominal contact position is a parameter that regularize the solution
        toEigen(m_pimpl->controllerInputs.contacts[key].nominalPosition).setZero();
    }

    const double absoluteTimeHorizon = m_pimpl->currentTime + m_pimpl->optiSettings.timeHorizon;

    // find the contactPhase associated to the current time
    auto initialPhase
        = std::find_if(contactPhaseList.begin(), contactPhaseList.end(), [=](const auto& phase) {
              return phase.beginTime <= m_pimpl->currentTime
                     && phase.endTime > m_pimpl->currentTime;
          });

    if (initialPhase == contactPhaseList.end())
    {
        log()->error("{} Unable to find the contact phase related to the current time.",
                     errorPrefix);
        return false;
    }

    // find the contactPhase associated to the end time
    auto finalPhase
        = std::find_if(contactPhaseList.begin(), contactPhaseList.end(), [=](const auto& phase) {
              return phase.beginTime <= absoluteTimeHorizon && phase.endTime > absoluteTimeHorizon;
          });

    // if the list is not found the latest contact phase is considered
    if (finalPhase == contactPhaseList.end())
    {
        finalPhase = std::prev(contactPhaseList.end());
        std::cerr << "error" << std::endl;
        return false;
    }

    int index = 0;
    for (auto it = initialPhase; it != std::next(finalPhase); std::advance(it, 1))
    {
        // TODO check what it's going on if t horizon is greater then last endtime
        const double tInitial = std::max(m_pimpl->currentTime, it->beginTime);
        const double tFinal = std::min(absoluteTimeHorizon, it->endTime);

        const int numberOfSamples
            = std::round((tFinal - tInitial) / m_pimpl->optiSettings.samplingTime);

        for (const auto& [key, contact] : it->activeContacts)
        {
            auto inputContact = m_pimpl->controllerInputs.contacts.find(key);
            if (inputContact == m_pimpl->controllerInputs.contacts.end())
            {
                log()->error("{} Unable to find the input contact named {}.", errorPrefix, key);
                return false;
            }

            toEigen(inputContact->second.nominalPosition)
                .middleCols(index, numberOfSamples)
                .colwise()
                = contact->pose.translation();

            // this is required to reshape the matrix into a vector
            const Eigen::Matrix3d orientation = contact->pose.quat().toRotationMatrix();
            toEigen(inputContact->second.orientation).middleCols(index, numberOfSamples).colwise()
                = Eigen::Map<const Eigen::VectorXd>(orientation.data(), orientation.size());

            constexpr double maxForce = 1e5;
            toEigen(inputContact->second.maximumNormalForce)
                .middleCols(index, numberOfSamples)
                .setConstant(maxForce);
        }

        index += numberOfSamples;
    }

    assert(index == m_pimpl->optiSettings.horizon);

    for (const auto& [key, contact] : m_pimpl->controllerInputs.contacts)
    {

        log()->info("{} maximumnormalforce  {}",
                    key,
                    toEigen(contact.maximumNormalForce));
    }
    // TODO Implement this part only if you want to add the push recovery
    for (auto& [key, contact] : m_pimpl->controllerInputs.contacts)
    {
        constexpr double maxVelocity = 1e5;
        toEigen(contact.upperLimitPosition).setZero();
        toEigen(contact.lowerLimitPosition).setZero();
        toEigen(contact.limitVelocity).setConstant(maxVelocity);
    }

    return true;
}
