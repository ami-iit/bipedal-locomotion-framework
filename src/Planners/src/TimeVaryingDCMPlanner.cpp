/**
 * @file TimeVaryingDCMPlanner.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <casadi/casadi.hpp>

#include <BipedalLocomotion/Math/Constants.h>
#include <BipedalLocomotion/Planners/ConvexHullHelper.h>
#include <BipedalLocomotion/Planners/QuinticSpline.h>
#include <BipedalLocomotion/Planners/TimeVaryingDCMPlanner.h>
#include <BipedalLocomotion/System/DynamicalSystem.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Planners;
using namespace BipedalLocomotion::Contacts;

/**
 * Private implementation of the TimeVaryingDCMPlanner class
 */
struct TimeVaryingDCMPlanner::Impl
{
    static constexpr std::size_t dcmVectorSize = 3;
    static constexpr std::size_t vrpVectorSize = 3;

    bool isTrajectoryComputed{false}; /**< True if the trajectory has been computed */
    bool isPlannerInitialized{false}; /**< True if the planner has been initialized */

    ConvexHullHelper convexHullHelper; /**< Convex hull helper. It is used to compute the
                                          constraints related to the position of the Zero Moment
                                          Point (ZMP) */

    BipedalLocomotion::Planners::QuinticSpline dcmRef;/**< Spline used to compute the internal DCM. This is used only if
                                                         useExternalDCMReference is false. */
    Eigen::MatrixXd externalDCMTrajectory; /**< External DCM trajectory. This is used only if
                                              useExternalDCMReference is true. */

    casadi::Opti opti; /**< CasADi opti stack */
    casadi::MX costFunction; /**< Cost function of the optimization problem */

    /**
     * OptimizationVariables contains the optimization variables expressed as CasADi elements.
     */
    struct OptimizationVariables
    {
        casadi::MX dcm;
        casadi::MX omega;
        casadi::MX vrp;
        casadi::MX omegaDot;
    };
    OptimizationVariables optiVariables; /**< Optimization variables */

    /**
     * OptimizationParameters contains some parameters used in the optimization problem. A parameter
     * is a constant value considered in the optimization problem. Before solving the problem the
     * parameters have to be filled with numerical values.
     */
    struct OptimizationParameters
    {
        casadi::MX dcmInitialPosition; /**< Initial boundary condition on the DCM position */
        casadi::MX dcmFinalPosition; /**< Final boundary condition on the DCM position */
        casadi::MX dcmInitialVelocity; /**< Initial boundary condition on the DCM velocity */
        casadi::MX dcmFinalVelocity; /**< Final boundary condition on the DCM velocity */
        casadi::MX omegaInitialValue; /**< Initial boundary condition on the omega */
        casadi::MX omegaFinalValue; /**< Final boundary condition on the omega */
        casadi::MX dcmRefererenceTraj; /** DCM trajectory. This is trajectory is used as
                                          regularization parameter while solving the optimization
                                          problem */
    };
    OptimizationParameters optiParameters; /**< Optimization parameters */

    /**
     * Utilities function used in the optimization problem
     */
    struct OptimizationFunctions
    {
        casadi::Function dcmDynamics; /** DCM dynamics */
        casadi::Function ecmp; /** Enhanced centroidal moment pivot (eCMP) equation */
    };
    OptimizationFunctions optiFunctions; /**< Collection of useful functions */

    /**
     * OptimizationSolution contains some elements required to retrieve the solution of the
     * optimization problem.
     */
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

    DCMPlannerState trajectory; /**< State of the planner at the current state */
    std::size_t trajectoryIndex; /**< Current index of the trajectory */
    std::size_t numberOfTrajectorySamples; /**< Number of the samples considered by the optimization
                                              problem */

    /**
     * Setting of the optimization problem
     */
    struct OptimizationSettings
    {
        unsigned long solverVerbosity{1}; /**< Verbosity of ipopt */
        std::string ipoptLinearSolver{"mumps"}; /**< Linear solved used by ipopt */

        double plannerSamplingTime; /**< Sampling time of the planner in seconds */
        std::vector<Eigen::Vector3d> footCorners; /**< Position of the corner of the foot
                                                    expressed in a frame placed in the center of the
                                                    foot. The X axis points forward and the z
                                                    upword.*/

        double omegaDotWeight; /**< Weight related to the omega dot */
        double dcmTrackingWeight; /**< Weight to the tracking of the DCM */
        double omegaDotRateOfChangeWeight; /**< Weight related to rate of change of omega dot */
        double vrpRateOfChangeWeight; /**< Weight related to the rate of change of the VRP */
        double dcmRateOfChangeWeight; /**< Weight related to the rate of change of the DCM */

        bool useExternalDCMReference{false}; /**< If true an external DCM reference is expected by
                                                the planner */

        double gravity{BipedalLocomotion::Math::StandardAccelerationOfGravitation}; /**< Gravity */
    };
    OptimizationSettings optiSettings; /**< Settings */

    struct InitialValue
    {
        casadi::DM dcm; /**< initial guess for the DCM trajectory */
        casadi::DM omega; /**< initial guess omega trajectory */
    };

    InitialValue initialValue; /**< Initial value for the optimizer */

    /**
     * Get the eCMP from VRP and omega
     */
    casadi::Function getECMP()
    {
        casadi::MX omega = casadi::MX::sym("omega");
        casadi::MX vrp = casadi::MX::sym("vrp", 3);
        casadi::MX omegaDot = casadi::MX::sym("omega_dot");

        casadi::MX rhs = casadi::MX::vertcat(
            {vrp(casadi::Slice(0, 2)),
             vrp(2) - this->optiSettings.gravity / (casadi::MX::pow(omega, 2) - omegaDot)});

        return casadi::Function("eCMP", {omega, vrp, omegaDot}, {rhs});
    }

    casadi::Function getDynamics()
    {
        casadi::MX state = casadi::MX::sym("state", 4);
        const auto& dcm = state(casadi::Slice(0,3));
        const auto& omega = state(3);

        casadi::MX vrp = casadi::MX::sym("vrp", 3);
        casadi::MX omegaDot = casadi::MX::sym("omega_dot");

        casadi::MX rhs = casadi::MX::vertcat(
            {dcm + (omega - omegaDot / omega) * (dcm - vrp) * optiSettings.plannerSamplingTime,
             omega + omegaDot * optiSettings.plannerSamplingTime});

        return casadi::Function("DCMDynamics", {state, vrp, omegaDot}, {rhs});
    }

    /**
     * Clear the optimization problem
     */
    void clear()
    {
        opti = casadi::Opti();
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
        ipoptOptions["linear_solver"] = this->optiSettings.ipoptLinearSolver;
        casadiOptions["expand"] = true;

        this->opti.solver("ipopt", casadiOptions, ipoptOptions);
    }

    /**
     * Setup opti stack
     */
    void setupOpti(const std::size_t& horizonSampling)
    {
        using Sl = casadi::Slice;

        // define the size of the optimization variables
        this->optiVariables.dcm = this->opti.variable(this->dcmVectorSize, horizonSampling + 1);
        this->optiVariables.omega = this->opti.variable(1, horizonSampling + 1);
        this->optiVariables.vrp = this->opti.variable(this->vrpVectorSize, horizonSampling);
        this->optiVariables.omegaDot = this->opti.variable(1, horizonSampling);

        // set the initial constraints
        this->optiParameters.dcmInitialPosition = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.dcmFinalPosition = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.dcmInitialVelocity = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.dcmFinalVelocity = this->opti.parameter(this->dcmVectorSize);
        this->optiParameters.omegaInitialValue = this->opti.parameter();
        this->optiParameters.omegaFinalValue = this->opti.parameter();

        // set constraints and cost functions whose equations does not change in the optimization
        // problem
        auto& dcm = this->optiVariables.dcm;
        auto& vrp = this->optiVariables.vrp;
        auto& omega = this->optiVariables.omega;
        auto& omegaDot = this->optiVariables.omegaDot;

        // initial values
        this->opti.subject_to(omega(Sl(), 0) == this->optiParameters.omegaInitialValue);
        this->opti.subject_to(dcm(Sl(), 0) == this->optiParameters.dcmInitialPosition);


        auto initialDcmVelocity = (omega(0) - omegaDot(0) / omega(0)) * (dcm(Sl(), 0) - vrp(Sl(), 0));
        auto initialOmegaVelocity = omegaDot(0);

        this->opti.subject_to(casadi::MX::vertcat({initialDcmVelocity, initialOmegaVelocity})
                              == casadi::MX::vertcat({this->optiParameters.dcmInitialVelocity, 0}));

        // final values
        this->opti.subject_to(omega(Sl(), -1) == this->optiParameters.omegaFinalValue);
        this->opti.subject_to(dcm(Sl(), -1) == this->optiParameters.dcmFinalPosition);

        auto finalDcmVelocity = (omega(-1) - omegaDot(-1) / omega(-1)) * (dcm(Sl(), -1) - vrp(Sl(), -1));
        auto finalOmegaVelocity = omegaDot(-1);

        this->opti.subject_to(casadi::MX::vertcat({finalDcmVelocity, finalOmegaVelocity})
                              == casadi::MX::vertcat({this->optiParameters.dcmFinalVelocity, 0}));

        auto trajectory = this->optiFunctions.dcmDynamics.map(horizonSampling);

        this->opti.subject_to(
            casadi::MX::vertcat(
                {dcm(Sl(), Sl(1, dcm.columns())), omega(Sl(), Sl(1, omega.columns()))})
            == trajectory({casadi::MX::vertcat({dcm(Sl(), Sl(0, -1)), omega(Sl(), Sl(0, -1))}),
                           vrp,
                           omegaDot})[0]);

        // omega^2 - omegaDot has to be positive
        this->opti.subject_to(casadi::MX::pow(omega(Sl(), Sl(0, -1)), 2) - omegaDot
                              > casadi::DM::zeros(omegaDot.rows(), omegaDot.columns()));

        // omega has to be positive
        this->opti.subject_to(omega > casadi::DM::zeros(omega.rows(), omega.columns()));

        this->optiParameters.dcmRefererenceTraj = this->opti.parameter(this->dcmVectorSize, horizonSampling + 1);

        this->costFunction
            = this->optiSettings.omegaDotWeight * casadi::MX::sumsqr(this->optiVariables.omegaDot)
              + this->optiSettings.dcmTrackingWeight
                    * casadi::MX::sumsqr(this->optiVariables.dcm
                                         - this->optiParameters.dcmRefererenceTraj)
              + this->optiSettings.omegaDotRateOfChangeWeight
                    * casadi::MX::sumsqr(casadi::MX::diff(this->optiVariables.omegaDot))
              + this->optiSettings.vrpRateOfChangeWeight
                    * casadi::MX::sumsqr(casadi::MX::diff(this->optiVariables.vrp.T()))
              + this->optiSettings.dcmRateOfChangeWeight
                    * casadi::MX::sumsqr(casadi::MX::diff(this->optiVariables.dcm.T()));

        this->opti.minimize(costFunction);

        // set the opti object options
        this->setupOptiOptions();
    }

    /**
     * Get the CasADi functions
     */
    void setupCasADiFunctions()
    {
        optiFunctions.ecmp = getECMP();
        optiFunctions.dcmDynamics = getDynamics();
    }

    bool computeConstraintElementsECMP(const ContactPhase& contactPhase,
                                       casadi::DM& ecmpConstraintA,
                                       casadi::DM& ecmpConstraintB)
    {
        // TODO we may optimize here (DYNAMIC MEMORY ALLOCATION)
        Eigen::MatrixXd points;

        // check if the points belongs to the same plane
        int numberOfCoordinates = 3;

        if (contactPhase.activeContacts.size() == 1)
        {
            numberOfCoordinates = 2;
        }
        else if (contactPhase.activeContacts.size() == 2)
        {
            auto contactIt = contactPhase.activeContacts.cbegin();
            double foot1Height = contactIt->second->pose.translation()[2];
            std::advance(contactIt, 1);
            double foot2Height = contactIt->second->pose.translation()[2];

            if (foot2Height == foot1Height)
            {
                numberOfCoordinates = 2;
            }
        }

        const std::size_t numberOfPoints
            = contactPhase.activeContacts.size() * this->optiSettings.footCorners.size();

        points.resize(numberOfCoordinates, numberOfPoints);

        // compute the convex hull only once
        int columnIndex = 0;
        Eigen::Vector3d point;
        for (const auto& [contactName, activeContact] : contactPhase.activeContacts)
        {
            for (const auto& footCorner : this->optiSettings.footCorners)
            {
                point = activeContact->pose.act(footCorner);
                points.col(columnIndex) = point.head(numberOfCoordinates);
                columnIndex++;
            }
        }
        this->convexHullHelper.buildConvexHull(points);

        // TODO PLEASE OPTIMIZE ME
        // please check https://github.com/casadi/casadi/issues/2563 and
        // https://groups.google.com/forum/#!topic/casadi-users/npPcKItdLN8
        // Assumption: the matrices as stored as column-major
        const auto& A = this->convexHullHelper.getA();
        ecmpConstraintA.resize(A.rows(), A.cols());
        ecmpConstraintA = casadi::DM::zeros(A.rows(), A.cols());
        std::memcpy(ecmpConstraintA.ptr(),
                    A.data(),
                    sizeof(double) * A.rows() * A.cols());

        const auto& b = this->convexHullHelper.getB();
        ecmpConstraintB.resize(b.size(), 1);
        ecmpConstraintB = casadi::DM::zeros(b.size(), 1);
        std::memcpy(ecmpConstraintB.ptr(), b.data(), sizeof(double) * b.size());

        return numberOfCoordinates == 2;
    }


    void computeVRPConstraint(const ContactPhaseList& contactPhaseList)
    {
        using Sl = casadi::Slice;

        casadi::DM ecmpConstraintA;
        casadi::DM ecmpConstraintB;

        const auto& vrp = this->optiVariables.vrp;
        const auto& omega = this->optiVariables.omega;
        const auto& omegaDot = this->optiVariables.omegaDot;

        auto contactPhaseListIt = contactPhaseList.cbegin();
        bool feetAreInSamePlane = computeConstraintElementsECMP(*contactPhaseListIt,
                                                                ecmpConstraintA,
                                                                ecmpConstraintB);

        for (std::size_t k = 0; k < this->optiVariables.vrp.columns(); k++)
        {
            // if the
            if(k * this->optiSettings.plannerSamplingTime > contactPhaseListIt->endTime)
            {
                std::advance(contactPhaseListIt, 1);
                feetAreInSamePlane = computeConstraintElementsECMP(*contactPhaseListIt,
                                                                   ecmpConstraintA,
                                                                   ecmpConstraintB);
            }

            if (!feetAreInSamePlane)
            {
                this->opti.subject_to(
                    casadi::MX::mtimes(ecmpConstraintA,
                                       casadi::MX::vertcat(this->optiFunctions.ecmp(
                                           {omega(Sl(), k), vrp(Sl(), k), omegaDot(Sl(), k)})))
                    <= ecmpConstraintB);
            } else
            {
                this->opti.subject_to(casadi::MX::mtimes(ecmpConstraintA, vrp(Sl(0, 2), k))
                                      <= ecmpConstraintB);

                this->opti.subject_to(
                    vrp(2, k)
                        - this->optiSettings.gravity
                              / (casadi::MX::pow(omega(Sl(), k), 2) - omegaDot(Sl(), k))
                    == contactPhaseListIt->activeContacts.begin()->second->pose.translation()[2]);
            }
        }
    }

    bool computeDCMRegularization(const ContactPhaseList& contactPhaseList,
                                  const DCMPlannerState& initialState)
    {
        std::vector<Eigen::VectorXd> dcmKnots;
        std::vector<double> timeKnots;

        // first point
        timeKnots.push_back(contactPhaseList.cbegin()->beginTime);
        dcmKnots.push_back(initialState.dcmPosition);

        double averageDCMHeight = initialState.dcmPosition[2];
        auto contactPhaseListIt = contactPhaseList.cbegin();

        for (std::size_t k = 0; k < this->optiVariables.vrp.columns(); k++)
        {
            // if the
            if (k * this->optiSettings.plannerSamplingTime > contactPhaseListIt->endTime)
            {
                std::advance(contactPhaseListIt, 1);

                if (contactPhaseListIt->activeContacts.size() == 2
                    && contactPhaseListIt != contactPhaseList.begin()
                    && contactPhaseListIt != contactPhaseList.lastPhase())
                {
                    timeKnots.emplace_back(
                        (contactPhaseListIt->endTime + contactPhaseListIt->beginTime) / 2);

                    auto contactIt = contactPhaseListIt->activeContacts.cbegin();
                    const Eigen::Vector3d p1 = contactIt->second->pose.translation();
                    std::advance(contactIt, 1);
                    const Eigen::Vector3d p2 = contactIt->second->pose.translation();

                    Eigen::Vector3d desiredDCMPosition = (p1 + p2) / 2.0;
                    desiredDCMPosition(2) += averageDCMHeight;

                    dcmKnots.emplace_back(desiredDCMPosition);
                }

                // TODO please try to make it more versatile
                // read it as if the robot is in the last double support phase
                else if (contactPhaseListIt->activeContacts.size() == 2
                         && contactPhaseListIt == contactPhaseList.lastPhase())
                {
                    timeKnots.push_back(contactPhaseListIt->endTime);
                    auto contactIt = contactPhaseListIt->activeContacts.cbegin();
                    const Eigen::Vector3d p1 = contactIt->second->pose.translation();
                    std::advance(contactIt, 1);
                    const Eigen::Vector3d p2 = contactIt->second->pose.translation();

                    Eigen::Vector3d desiredDCMPosition = (p1 + p2) / 2.0;
                    desiredDCMPosition(2) += averageDCMHeight;

                    dcmKnots.emplace_back(desiredDCMPosition);
                }
            }
        }
        dcmRef.setInitialConditions(initialState.dcmVelocity, Eigen::Vector3d::Zero());
        dcmRef.setFinalConditions(Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());
        dcmRef.setKnots(dcmKnots, timeKnots);

        // populate the reference
        this->initialValue.dcm = casadi::DM::zeros(3, this->optiVariables.dcm.columns());
        Eigen::Vector3d velocity, acceleration;
        Eigen::Map<Eigen::MatrixXd> initialValueDCMEigenMap(initialValue.dcm.ptr(),
                                                            initialValue.dcm.rows(),
                                                            initialValue.dcm.columns());

        for (int i = 0; i < this->optiVariables.dcm.columns(); i++)
        {
            const double currentTime = i * this->optiSettings.plannerSamplingTime;
            if (!dcmRef.evaluatePoint(currentTime,
                                     initialValueDCMEigenMap.col(i),
                                     velocity,
                                     acceleration))
            {
                log()->error("[TimeVaryingDCMPlanner::Impl::computeDCMRegularization] Unable to "
                             "evaluate the dcm reference trajectory.");
                return false;
            }
        }
        return true;
    }

    bool setupOptimizationProblem(const ContactPhaseList& contactPhaseList,
                                  const DCMPlannerState& initialState)
    {
        constexpr auto logPrefix = "[TimeVaryingDCMPlanner::Impl::setupOptimizationProblem]";

        using Sl = casadi::Slice;

        // set the initial conditions
        this->opti.set_value(this->optiParameters.dcmInitialPosition,
                             casadi::DM(std::vector<double>(initialState.dcmPosition.data(),
                                                            initialState.dcmPosition.data()
                                                                + this->dcmVectorSize)));
        this->opti.set_value(this->optiParameters.dcmInitialVelocity,
                             casadi::DM(std::vector<double>(initialState.dcmVelocity.data(),
                                                            initialState.dcmVelocity.data()
                                                                + this->dcmVectorSize)));
        this->opti.set_value(this->optiParameters.omegaInitialValue, initialState.omega);

        this->computeVRPConstraint(contactPhaseList);

        if (!this->optiSettings.useExternalDCMReference)
        {
            if (!this->computeDCMRegularization(contactPhaseList, initialState))
            {
                log()->error("{} Unable to compute the DCM regularization term.", logPrefix);
                return false;
            }
        } else
        {
            if (this->externalDCMTrajectory.rows() != this->dcmVectorSize
                || this->externalDCMTrajectory.cols() != this->optiVariables.dcm.columns())
            {
                log()->error("{} Wrong size of the dcmReference. Expected matrix: (3 x {}). Passed "
                             "matrix: ({} x {}).",
                             logPrefix,
                             this->optiVariables.dcm.columns(),
                             this->externalDCMTrajectory.rows(),
                             this->externalDCMTrajectory.cols());
                return false;
            }

            this->initialValue.dcm = casadi::DM::zeros(3, this->optiVariables.dcm.columns());
            Eigen::Map<Eigen::MatrixXd>(this->initialValue.dcm.ptr(),
                                        this->initialValue.dcm.rows(),
                                        this->initialValue.dcm.columns())
                = this->externalDCMTrajectory;
        }

        // set last values
        this->opti.set_value(this->optiParameters.dcmFinalPosition,
                             this->initialValue.dcm(Sl(), -1));

        this->opti.set_value(this->optiParameters.dcmFinalVelocity,
                             casadi::DM::zeros(this->dcmVectorSize));

        // TODO the final omega is equal to the initial omega
        this->opti.set_value(this->optiParameters.omegaFinalValue, initialState.omega);
        this->initialValue.omega = sqrt(this->optiSettings.gravity / this->initialValue.dcm(2, Sl()));
        this->opti.set_value(this->optiParameters.dcmRefererenceTraj, this->initialValue.dcm);

        this->opti.set_initial(this->optiVariables.dcm, this->initialValue.dcm);
        this->opti.set_initial(this->optiVariables.vrp, this->initialValue.dcm(Sl(), Sl(0, -1)));
        this->opti.set_initial(this->optiVariables.omega, this->initialValue.omega);
        this->opti.set_initial(this->optiVariables.omegaDot,
                               casadi::DM::zeros(1, this->optiVariables.omegaDot.columns()));

        return true;
    }

    void prepareSolution()
    {
        using Sl = casadi::Slice;

        std::memcpy(this->trajectory.dcmPosition.data(),
                    optiSolution.dcm(Sl(), this->trajectoryIndex).ptr(),
                    sizeof(double) * this->dcmVectorSize);

        std::memcpy(this->trajectory.vrpPosition.data(),
                    optiSolution.vrp(Sl(), this->trajectoryIndex).ptr(),
                    sizeof(double) * this->dcmVectorSize);

        trajectory.omega = static_cast<double>(optiSolution.omega(0, this->trajectoryIndex));

        trajectory.omegaDot = static_cast<double>(optiSolution.omegaDot(0, this->trajectoryIndex));

        trajectory.dcmVelocity = (trajectory.omega - trajectory.omegaDot / trajectory.omega)
                                 * (trajectory.dcmPosition - trajectory.vrpPosition);
    }
};

TimeVaryingDCMPlanner::TimeVaryingDCMPlanner()
    : DCMPlanner()
{
    m_pimpl = std::make_unique<Impl>();
    assert(m_pimpl);
}

TimeVaryingDCMPlanner::~TimeVaryingDCMPlanner() = default;

bool TimeVaryingDCMPlanner::initialize(std::weak_ptr<ParametersHandler::IParametersHandler> handler)
{
    constexpr auto logPrefix = "[TimeVaryingDCMPlanner::initialize]";
    assert(m_pimpl);

    auto ptr = handler.lock();

    if (ptr == nullptr)
    {
        log()->error("{} The handler has to point to an already initialized IParametershandler.",
                     logPrefix);
        return false;
    }

    if (!ptr->getParameter("planner_sampling_time", m_pimpl->optiSettings.plannerSamplingTime))
    {
        log()->error("{} Unable to load the sampling time of the planner.", logPrefix);
        return false;
    }

    int numberOfFootCorners;
    if (!ptr->getParameter("number_of_foot_corners", numberOfFootCorners))
    {
        log()->error("{} Unable to load the number of foot corners.", logPrefix);
        return false;
    }

    m_pimpl->optiSettings.footCorners.resize(numberOfFootCorners);

    for (std::size_t i = 0; i < numberOfFootCorners; i++)
    {
        if (!ptr->getParameter("foot_corner_" + std::to_string(i),
                               m_pimpl->optiSettings.footCorners[i]))
        {
            std::string errorMessage
                = "Unable to load get the foot corner number: " + std::to_string(i)
                  + ". Please provide the foot corners having the following "
                    "names: ";
            for (std::size_t j = 0; j < numberOfFootCorners; j++)
            {
                errorMessage += "foot_corner_" + std::to_string(j) + " ";
            }
            errorMessage += ".";

            log()->error("{} {}", logPrefix, errorMessage);

            return false;
        }
    }

    // get the linear solver used by ipopt. This parameter is optional. The default value is mumps
    std::string linearSolver;
    if (ptr->getParameter("linear_solver", linearSolver))
    {
        m_pimpl->optiSettings.ipoptLinearSolver = linearSolver;
    } else
    {
        log()->info("{} linear_solver not found. The following parameter will be used {}.",
                    logPrefix,
                    m_pimpl->optiSettings.ipoptLinearSolver);
    }

    bool ok = true;
    ok = ok && ptr->getParameter("omega_dot_weight", m_pimpl->optiSettings.omegaDotWeight);
    ok = ok && ptr->getParameter("dcm_tracking_weight", m_pimpl->optiSettings.dcmTrackingWeight);
    ok = ok && ptr->getParameter("omega_dot_rate_of_change_weight",
                                         m_pimpl->optiSettings.omegaDotRateOfChangeWeight);
    ok = ok && ptr->getParameter("vrp_rate_of_change_weight",
                                         m_pimpl->optiSettings.vrpRateOfChangeWeight);

    ok = ok && ptr->getParameter("dcm_rate_of_change_weight",
                                         m_pimpl->optiSettings.dcmRateOfChangeWeight);

    if (!ok)
    {
        log()->error("{} Unable to load weights of the cost function", logPrefix);
        return false;
    }

    /////// Optional parameters

    // if this option is chosen an external dcm reference must be provided
    m_pimpl->optiSettings.useExternalDCMReference = false;
    if (!ptr->getParameter("use_external_dcm_reference",
                           m_pimpl->optiSettings.useExternalDCMReference))
    {

        log()->info("{} use_external_dcm_reference not found. The following parameter will be used "
                    "{}.",
                    logPrefix,
                    m_pimpl->optiSettings.useExternalDCMReference);
    }

    if (ptr->getParameter("gravity", m_pimpl->optiSettings.gravity))
    {
        if (m_pimpl->optiSettings.gravity <= 0)
        {
            log()->error("{} The gravity should be a strictly positive number. If you do not know "
                         "which value use you can avoid to set "
                         "this parameter. The default value will be used. Default value: {}.",
                         logPrefix,
                         BipedalLocomotion::Math::StandardAccelerationOfGravitation);
            return false;
        }
    } else
    {
        log()->info("{} gravity not found. The following parameter will be used {}.",
                    logPrefix,
                    BipedalLocomotion::Math::StandardAccelerationOfGravitation);
    }

    // the casadi functions are initialized only once
    m_pimpl->setupCasADiFunctions();

    m_pimpl->isPlannerInitialized = true;
    return true;
}

bool TimeVaryingDCMPlanner::computeTrajectory()
{
    assert(m_pimpl);

    constexpr auto logPrefix = "[TimeVaryingDCMPlanner::initialize]";

    if (!m_pimpl->isPlannerInitialized)
    {
        log()->error("{} Please initialize the planner before computing the trajectory.",
                     logPrefix);
        return false;
    }

    if (m_pimpl->isTrajectoryComputed)
    {
        log()->error("{} The trajectory has been already computed.", logPrefix);
        return false;
    }

    // clear the solver and the solution computed
    m_pimpl->clear();

    const double& initialTrajectoryTime = m_contactPhaseList.cbegin()->beginTime;
    const double& endTrajectoryTime = m_contactPhaseList.lastPhase()->endTime;

    m_pimpl->numberOfTrajectorySamples = std::ceil((endTrajectoryTime - initialTrajectoryTime)
                                                   / m_pimpl->optiSettings.plannerSamplingTime);

    m_pimpl->setupOpti(m_pimpl->numberOfTrajectorySamples);

    if (!m_pimpl->setupOptimizationProblem(m_contactPhaseList, m_initialState))
    {
        log()->error("{} Unable to setup the optimization problem.", logPrefix);
        return false;
    }

    // this is how casadi works
    try
    {
        m_pimpl->optiSolution.solution = std::make_unique<casadi::OptiSol>(m_pimpl->opti.solve());
    } catch (const std::exception& e)
    {
        log()->error("{} Unable to solve the optimization problem. The following exception has "
                     "been thrown by the solver: {}.",
                     logPrefix,
                     e.what());

        return false;
    }

    m_pimpl->optiSolution.dcm = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.dcm);
    m_pimpl->optiSolution.vrp = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.vrp);
    m_pimpl->optiSolution.omega = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.omega);
    m_pimpl->optiSolution.omegaDot = m_pimpl->optiSolution.solution->value(m_pimpl->optiVariables.omegaDot);

    // reinitialize the trajectory
    m_pimpl->trajectoryIndex = 0;

    m_pimpl->prepareSolution();

    m_pimpl->isTrajectoryComputed = true;

    return true;
}

bool TimeVaryingDCMPlanner::setContactPhaseList(const Contacts::ContactPhaseList &contactPhaseList)
{
    assert(m_pimpl);

    if(!m_pimpl->isPlannerInitialized)
    {
        log()->error("[TimeVaryingDCMPlanner::setContactPhaseList] Please initialize the planner "
                     "before computing the trajectory.");
        return false;
    }

    // store the contact phase list
    m_contactPhaseList = contactPhaseList;

    // We have to recompute the trajectory
    m_pimpl->isTrajectoryComputed = false;
    return true;
}

bool TimeVaryingDCMPlanner::setDCMReference(Eigen::Ref<const Eigen::MatrixXd> dcmReference)
{
    constexpr auto logPrefix = "[TimeVaryingDCMPlanner::setDCMReference]";

    assert(m_pimpl);
    if(!m_pimpl->optiSettings.useExternalDCMReference)
    {
        log()->error("{} You cannot call this function if you configure the planner with "
                     "use_external_dcm_reference equal to false.", logPrefix);
        return false;
    }

    if (dcmReference.rows() != m_pimpl->dcmVectorSize)
    {
        log()->error("{} The dcmReference should be a matrix with three rows. Expected rows: {}, "
                     "Passed matrix rows: {}.",
                     logPrefix,
                     m_pimpl->dcmVectorSize,
                     dcmReference.rows());
        return false;
    }

    m_pimpl->externalDCMTrajectory = dcmReference;

    m_pimpl->isTrajectoryComputed = false;

    return true;
}

void TimeVaryingDCMPlanner::setInitialState(const DCMPlannerState &initialState)
{
    assert(m_pimpl);

    m_initialState = initialState;

    // We have to recompute the trajectory
    m_pimpl->isTrajectoryComputed = false;
}

const DCMPlannerState& TimeVaryingDCMPlanner::get() const
{
    assert(m_pimpl);
    return m_pimpl->trajectory;
}

bool TimeVaryingDCMPlanner::isValid() const
{
    assert(m_pimpl);
    return m_pimpl->isTrajectoryComputed;
}

bool TimeVaryingDCMPlanner::advance()
{
    constexpr auto logPrefix = "[TimeVaryingDCMPlanner::advance]";

    if (!isValid())
    {
        log()->error("{} The data are not valid it is not possible to advance.", logPrefix);
        return false;
    }

    m_pimpl->trajectoryIndex = std::min(m_pimpl->trajectoryIndex + 1, m_pimpl->numberOfTrajectorySamples - 1);

    m_pimpl->prepareSolution();

    return true;
}
