/**
 * @file InvariantEKFBaseEstimator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>
#include <iDynTree/EigenHelpers.h>

#include <manif/manif.h>

using namespace BipedalLocomotion::Estimators;

class InvariantEKFBaseEstimator::Impl
{
public:
    /**
     * Struct containing skew-symmetric matrices
     */
    struct SkewSymContainter
    {
        Eigen::Matrix3d gCross;  /**< skew of acceleretion due to gravity */
        Eigen::Matrix3d vCross;  /**< skew of linear velocity of IMU frame in mixed-representation */
        Eigen::Matrix3d pCross;  /**< skew of position of IMU frame origin */
        Eigen::Matrix3d prCross; /**< skew of position of right foot contact frame origin */
        Eigen::Matrix3d plCross; /**< skew of position of left foot contact frame origin */
    };

    /**
     * Construct internal state object given,
     *  - state matrix \f[ X \in SE_2+2(3) \f] and
     *  - parameters vector \f[ \theta \in \mathbb{R}^6 \f]
     */
    bool constructState(const Eigen::MatrixXd& X,
                        const Eigen::Matrix<double, 6, 1>& theta,
                        FloatingBaseEstimators::InternalState& state);

    /**
     * Extract
     *  - state matrix \f[ X \in SE_2+2(3) \f] and
     *  - parameters vector \f[ \theta \in \mathbb{R}^6 \f],
     * from internal state object
     */
    void extractState(const FloatingBaseEstimators::InternalState& state,
                      Eigen::MatrixXd& X,
                      Eigen::Matrix<double, 6, 1>& theta);

    /**
     * Construct  the state covariance matrix from the
     * internal state standard deviation object,
     */
    void constuctStateVar(const FloatingBaseEstimators::StateStdDev& stateStdDev,
                          const bool& estimateBias,
                          Eigen::MatrixXd& P);

    /**
     * Extract internal state standard deviation object,
     * from the diagonal elements of the state covariance matrix
     */
    void extractStateVar(const Eigen::MatrixXd& P,
                         const bool& estimateBias,
                         FloatingBaseEstimators::StateStdDev& stateStdDev);

    /**
     * Propagate internal state of the estimator,
     * given previous IMU measurements and predicted foot poses relative to IMU
     */
    bool propagateStates(const FloatingBaseEstimators::Measurements& meas,
                         const double& dt, const Eigen::Vector3d& g,
                         const manif::SE3d& IMU_H_RF,
                         const manif::SE3d& IMU_H_LF,
                         FloatingBaseEstimators::InternalState& X);

    /**
     * Perform the Kalman filter update step given measurements and Jacobians
     */
    bool updateStates(const Eigen::VectorXd& obs,
                      const Eigen::MatrixXd measModelJacobian,
                      const Eigen::MatrixXd& measNoiseVar,
                      const Eigen::MatrixXd& auxMat,
                      FloatingBaseEstimators::InternalState& state,
                      Eigen::MatrixXd& P);

    /**
     * Compute exponential map of \f[SE_2+2(3)\f]
     * this is a composition of a hat and a exp operator
     * Exp: R9 to SE_2+2(3)
     * The order of vector,
     * (w, a, v, vr, vl) where
     * w - angular velocity IMU
     * a - linear acceleration IMU
     * v - linear velocity IMU
     * vr - linear velocity of right foot contact freame
     * vl - linear velocity of left foot contact freame
     * We choose this order to be consistent with original paper implementation
     */
    bool calcExpHatX(const Eigen::VectorXd& vec,
                     Eigen::MatrixXd& X);

    /**
     * Compute Adjoint matrix of \f[SE_2+2(3)\f]
     * The order of vector,
     * (w, a, v, vr, vl) where
     * w - angular velocity IMU
     * a - linear acceleration IMU
     * v - linear velocity IMU
     * vr - linear velocity of right foot contact freame
     * vl - linear velocity of left foot contact freame
     * We choose this order to be consistent with original paper implementation
     */
    void calcAdjointX(const FloatingBaseEstimators::InternalState& X,
                      const SkewSymContainter& skew,
                      Eigen::MatrixXd& AdjX);

    /**
     * Copy the state matrix X along block diagonal.
     * This is used for multiplying the stacked observations
     */
    bool copyDiagX(const Eigen::MatrixXd& X,
                   const int& n,
                   Eigen::MatrixXd& BigX);

    /**
     * Compute continuous time system noise covariance matrix
     * using the predicted internal state estimates
     */
    void calcQc(const FloatingBaseEstimators::InternalState& X,
                const FloatingBaseEstimators::SensorsStdDev& sensStdDev,
                const FloatingBaseEstimators::Measurements& meas,
                const bool& estimateBias,
                Eigen::MatrixXd& Qc);

    /**
     * Compute transformation matrix  for the
     * system noise using the predicted internal state estimates
     */
    void calcLc(const FloatingBaseEstimators::InternalState& X,
                const SkewSymContainter& skew,
                const bool& estimateBias,
                Eigen::MatrixXd& Lc);

    /**
     * Compute continuous time linearized state propagation matrix
     */
    void calcFc(const FloatingBaseEstimators::InternalState& X,
                const SkewSymContainter& skew,
                const bool& estimateBias,
                Eigen::MatrixXd& Fc);

    /**
     * Compute skew symmetric matrices given states
     */
    void calcSkewSymAtCurrenState(const FloatingBaseEstimators::InternalState& X,
                                  SkewSymContainter& skew);

    /**
     * Dynamically update filter matrices depending on bias estimation flag
     */
    void updateFilterMatrixDimensions(const bool& estimateBias);

    SkewSymContainter m_skew; /**< skew symmetric matrix container */

    Eigen::MatrixXd m_P;      /**< state covariance matrix */
    Eigen::MatrixXd m_Fc, m_Qc, m_Lc; /**< continuous time system propagation matrices */
    Eigen::MatrixXd m_Fk, m_Qk; /**< discrete time system propagation matrices */
    Eigen::MatrixXd m_In;  /** identity matrix with dimensions of state manifold tangent space */
    Eigen::MatrixXd m_PHT, m_S, m_K, m_IminusKH; /** measurement update matrices **/
    Eigen::VectorXd m_obs; /**< observation vector */
    Eigen::MatrixXd m_measModelJacobian, m_measNoiseVar, m_auxMat; /**< measurement model matrices */
    Eigen::MatrixXd m_BigX, m_innovation; /**< invariant error update related matrices */
    Eigen::VectorXd m_delta; /**< correction term*/
    Eigen::MatrixXd m_X, m_dX; /**< placeholder for state update variables */
    Eigen::Matrix<double, 6, 1> m_theta, m_deltaTheta; /**< placeholder for parameter update variables */

    manif::SE3d IMU_H_LF, IMU_H_RF; /**< buffers for relative transforms */
    Eigen::MatrixXd J_IMULF, J_IMURF; /**< buffers for relative Jacobians */

    const size_t m_vecSizeWOBias{15}; /**< Tangent space vector size without considering IMU biases */
    const size_t m_vecSizeWBias{21};  /**< Tangent space vector size considering IMU biases */

    struct {
    const size_t imuOrientation{0};
    const size_t imuLinearVel{3};
    const size_t imuPosition{6};
    const size_t rContactFramePosition{9};
    const size_t lContactFramePosition{12};
    const size_t gyroBias{15};
    const size_t accBias{18};
    } m_vecOffsets;  /**< Tangent space vector offsets */

    bool m_prevBiasEstimationFlag{false};

    friend class InvariantEKFBaseEstimator;
};

InvariantEKFBaseEstimator::InvariantEKFBaseEstimator()
    : m_pimpl(std::make_unique<Impl>())
{
    m_isInvEKF = true;
    m_state.imuOrientation.setIdentity();
    m_state.imuPosition.setZero();
    m_state.imuLinearVelocity.setZero();
    m_state.rContactFrameOrientation.setIdentity();
    m_state.rContactFramePosition.setZero();
    m_state.lContactFrameOrientation.setIdentity();
    m_state.lContactFramePosition.setZero();
    m_state.accelerometerBias.setZero();
    m_state.gyroscopeBias.setZero();

    m_statePrev = m_state;
    m_estimatorOut.state = m_state;

    m_meas.acc.setZero();
    m_meas.gyro.setZero();
    m_meas.lfInContact = false;
    m_meas.rfInContact = false;

    m_measPrev = m_meas;

    m_stateStdDev.imuOrientation.setZero();
    m_stateStdDev.imuPosition.setZero();
    m_stateStdDev.imuLinearVelocity.setZero();
    m_stateStdDev.rContactFrameOrientation.setZero();
    m_stateStdDev.rContactFramePosition.setZero();
    m_stateStdDev.lContactFrameOrientation.setZero();
    m_stateStdDev.lContactFramePosition.setZero();
    m_stateStdDev.accelerometerBias.setZero();
    m_stateStdDev.gyroscopeBias.setZero();

    m_priors = m_stateStdDev;
    m_estimatorOut.stateStdDev = m_stateStdDev;

    m_sensorsDev.gyroscopeNoise.setZero();
    m_sensorsDev.accelerometerNoise.setZero();
    m_sensorsDev.accelerometerBiasNoise.setZero();
    m_sensorsDev.gyroscopeBiasNoise.setZero();
    m_sensorsDev.contactFootLinvelNoise.setZero();
    m_sensorsDev.contactFootAngvelNoise.setZero();
    m_sensorsDev.swingFootLinvelNoise.setZero();
    m_sensorsDev.swingFootAngvelNoise.setZero();
}

InvariantEKFBaseEstimator::~InvariantEKFBaseEstimator() = default;

bool InvariantEKFBaseEstimator::customInitialization(std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    if (handle == nullptr)
    {
        std::cerr << "[InvariantEKFBaseEstimator::customInitialization] The parameter handler has expired. Please check its scope."
        << std::endl;
        return false;
    }

    // setup options related entities
    auto optionsHandle = handle->getGroup("Options");
    if (!setupOptions(optionsHandle))
    {
        std::cerr << "[InvariantEKFBaseEstimator::customInitialization] Could not load options related parameters."
        << std::endl;
        return false;
    }

    // setup sensor standard deviations
    auto sensorDevHandle = handle->getGroup("SensorsStdDev");
    if (!setupSensorDevs(sensorDevHandle))
    {
        std::cerr << "[InvariantEKFBaseEstimator::customInitialization] Could not load sensor stddev related parameters."
        << std::endl;
        return false;
    }

    // setup initial states
    auto initStateHandle = handle->getGroup("InitialStates");
    if (!setupInitialStates(initStateHandle))
    {
        std::cerr << "[InvariantEKFBaseEstimator::customInitialization] Could not load initial states related parameters."
        << std::endl;
        return false;
    }

    // setup initial state standard deviations
    auto priorDevHandle = handle->getGroup("PriorsStdDev");
    if (!setupPriorDevs(priorDevHandle))
    {
        std::cerr << "[InvariantEKFBaseEstimator::customInitialization] Could not load prior stddev related parameters."
        << std::endl;
        return false;
    }

    m_pimpl->m_skew.gCross = iDynTree::skew(m_options.accelerationDueToGravity);
    m_pimpl->updateFilterMatrixDimensions(m_options.imuBiasEstimationEnabled);
    m_pimpl->m_prevBiasEstimationFlag = m_options.imuBiasEstimationEnabled;
    m_pimpl->constuctStateVar(m_priors, m_options.imuBiasEstimationEnabled, m_pimpl->m_P); // construct priors

    size_t baseDim{6};
    m_pimpl->J_IMULF.resize(baseDim, m_modelComp.nrJoints());
    m_pimpl->J_IMURF.resize(baseDim, m_modelComp.nrJoints());
    return true;
}

bool InvariantEKFBaseEstimator::resetEstimator(const FloatingBaseEstimators::InternalState& newState,
                                               const FloatingBaseEstimators::StateStdDev& newPriorDev)
{
    m_state = newState;
    m_stateStdDev = newPriorDev;
    m_priors = newPriorDev;

    m_pimpl->constuctStateVar(m_priors, m_options.imuBiasEstimationEnabled, m_pimpl->m_P); // construct priors
    return true;
}

bool InvariantEKFBaseEstimator::resetEstimator(const FloatingBaseEstimators::InternalState& newState,
                                               const FloatingBaseEstimators::StateStdDev& newPriorDev,
                                               const FloatingBaseEstimators::SensorsStdDev& newSensorsDev)
{
    m_sensorsDev = newSensorsDev;
    resetEstimator(newState, newPriorDev);
    return true;
}

void BipedalLocomotion::Estimators::InvariantEKFBaseEstimator::Impl::updateFilterMatrixDimensions(const bool& estimateBias)
{
    if (estimateBias)
    {
        m_In = Eigen::MatrixXd::Identity(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        m_In = Eigen::MatrixXd::Identity(m_vecSizeWOBias, m_vecSizeWOBias);
    }
}

bool InvariantEKFBaseEstimator::predictState(const FloatingBaseEstimators::Measurements& meas,
                                             const double& dt)
{
    m_pimpl->calcSkewSymAtCurrenState(m_statePrev, m_pimpl->m_skew); // compute skews at priori state

    // update foot position predictions with previous measures
    if (!m_modelComp.getIMU_H_feet(meas.encoders, m_pimpl->IMU_H_LF, m_pimpl->IMU_H_RF))
    {
        return false;
    }

    // m_state is now predicted state after this function call
    if (!m_pimpl->propagateStates(meas, dt, m_options.accelerationDueToGravity, m_pimpl->IMU_H_RF, m_pimpl->IMU_H_LF, m_state))
    {
        return false;
    }

    if (m_pimpl->m_prevBiasEstimationFlag != m_options.imuBiasEstimationEnabled)
    {
        m_pimpl->updateFilterMatrixDimensions(m_options.imuBiasEstimationEnabled);
    }

    m_pimpl->m_prevBiasEstimationFlag = m_options.imuBiasEstimationEnabled;

    m_pimpl->calcFc(m_statePrev, m_pimpl->m_skew, m_options.imuBiasEstimationEnabled, m_pimpl->m_Fc); // compute Fc at priori state
    m_pimpl->calcQc(m_statePrev, m_sensorsDev, meas,  m_options.imuBiasEstimationEnabled, m_pimpl->m_Qc); // compute Qc at priori state and previous measure
    m_pimpl->calcLc(m_statePrev, m_pimpl->m_skew, m_options.imuBiasEstimationEnabled, m_pimpl->m_Lc); // compute Lc at priori state

    // discretize linearized dynamics and propagate covariance
    m_pimpl->m_Fk = m_pimpl->m_In + (m_pimpl->m_Fc*dt);  // read as Fk = I + (Fc*dt)
    m_pimpl->m_Qk = (m_pimpl->m_Fk*m_pimpl->m_Lc*m_pimpl->m_Qc*(m_pimpl->m_Lc.transpose())*(m_pimpl->m_Fk.transpose()))*dt; // read as Qk = Fk Lc Qc Lc.T Fk.T
    m_pimpl->m_P = m_pimpl->m_Fk*m_pimpl->m_P*(m_pimpl->m_Fk.transpose()) + m_pimpl->m_Qk; // read as P = Fk P Fk.T + Qk
    m_pimpl->extractStateVar(m_pimpl->m_P,m_options.imuBiasEstimationEnabled, m_stateStdDev); // unwrap state covariance matrix diagonal

    return true;
}

bool InvariantEKFBaseEstimator::updateKinematics(FloatingBaseEstimators::Measurements& meas,
                                                 const double& dt)
{
    Eigen::Matrix3d A_R_IMU = m_state.imuOrientation.toRotationMatrix();

    m_modelComp.getIMU_H_feet(meas.encoders, m_pimpl->IMU_H_LF, m_pimpl->IMU_H_RF, m_pimpl->J_IMULF, m_pimpl->J_IMURF);

    Eigen::Vector3d sIMU_p_LF = m_pimpl->IMU_H_LF.translation();
    Eigen::Vector3d sIMU_p_RF = m_pimpl->IMU_H_RF.translation();

    Eigen::VectorXd encodersVar = m_sensorsDev.encodersNoise.array().square();
    Eigen::MatrixXd Renc = static_cast<Eigen::MatrixXd>(encodersVar.asDiagonal());

    constexpr int firstFootPositionVecOffsetObs{0};
    constexpr int secondFootPositionVecOffsetObs{3};

    // The right invariant observation from the forward kinematics has structure Y = X^{-1} b + V
    // See Section III C. of the original paper
    // [ h(s) ]     [R.T -R.T v  -R.T p  -R.T d ] [  0_{3x1}]   [ Jv(s) w_s]
    // [   0  ]     [         1                 ] [  0      ]   [       0  ]
    // [   1  ]  =  [                 1         ] [  1      ] + [       0  ]
    // [  -1  ]     [                          1] [ -1      ]   [       0  ]
    // s is the encoder measurements and Jv(s) the relevant manipulator Jacobian
    // Note that the above equation describes only equations for one foot.
    //
    // The measurement model Jacobian becomes (shown for one foot),
    // H = [0_{3x3}  0_{3x3} -I  I]
    // I is the 3d identity matrix
    //
    // The measurement noise covariance,
    // N = R Jv(s) Cov(w_s) Jv(s).T R.T
    //
    // An auxiliary matrix is used along with the Kalman gain due to a reduced form of state update equations
    // Pi = [I  0_{3x3}]
    if (meas.lfInContact && meas.rfInContact)
    {
        constexpr int observationDimension{14};
        constexpr int linearizedDimension{6};
        // prepare observation vector Y
        m_pimpl->m_obs.resize(observationDimension);
        m_pimpl->m_obs << sIMU_p_RF, 0, 1, -1, 0,
                          sIMU_p_LF, 0, 1,  0, -1;

        // prepare measurement model Jacobian H
        if (m_options.imuBiasEstimationEnabled)
        {
            m_pimpl->m_measModelJacobian.resize(linearizedDimension, m_pimpl->m_vecSizeWBias);
        }
        else
        {
            m_pimpl->m_measModelJacobian.resize(linearizedDimension, m_pimpl->m_vecSizeWOBias);
        }

        m_pimpl->m_measModelJacobian.topLeftCorner<6, 6>().setZero();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.rContactFramePosition).setIdentity();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.lContactFramePosition).setZero();
        m_pimpl->m_measModelJacobian.block<3, 3>(secondFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        m_pimpl->m_measModelJacobian.block<3, 3>(secondFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.rContactFramePosition).setZero();
        m_pimpl->m_measModelJacobian.block<3, 3>(secondFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.lContactFramePosition).setIdentity();

        if (m_options.imuBiasEstimationEnabled)
        {
            m_pimpl->m_measModelJacobian.block<6, 6>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.gyroBias).setZero();
        }

        // prepare auxiliary gain matrix Pi
        constexpr int firstFootIGainOffset{0};
        constexpr int firstFootZeroGainOffset{3};
        constexpr int secondFootIGainOffset{7};
        constexpr int secondFootZeroGainOffset{10};
        m_pimpl->m_auxMat.resize(linearizedDimension, observationDimension);
        m_pimpl->m_auxMat.block<3, 3>(firstFootPositionVecOffsetObs, firstFootIGainOffset) = m_pimpl->m_auxMat.block<3, 3>(secondFootPositionVecOffsetObs, secondFootIGainOffset) = Eigen::Matrix3d::Identity();
        m_pimpl->m_auxMat.block<3, 11>(firstFootPositionVecOffsetObs, firstFootZeroGainOffset).setZero();
        m_pimpl->m_auxMat.block<3, 7>(secondFootPositionVecOffsetObs, firstFootIGainOffset).setZero();
        m_pimpl->m_auxMat.block<3, 4>(secondFootPositionVecOffsetObs, secondFootZeroGainOffset).setZero();

        // prepare measurement noise covariance R
        m_pimpl->m_measNoiseVar.resize(linearizedDimension, linearizedDimension);
        m_pimpl->m_measNoiseVar.topLeftCorner<3, 3>() = A_R_IMU*m_pimpl->J_IMURF.topRows<3>()*Renc*(m_pimpl->J_IMURF.topRows<3>().transpose())*(A_R_IMU.transpose());
        m_pimpl->m_measNoiseVar.topRightCorner<3, 3>().setZero();
        m_pimpl->m_measNoiseVar.bottomRightCorner<3, 3>() = A_R_IMU*m_pimpl->J_IMULF.topRows<3>()*Renc*(m_pimpl->J_IMULF.topRows<3>().transpose())*(A_R_IMU.transpose());
        m_pimpl->m_measNoiseVar.bottomLeftCorner<3, 3>().setZero();
        m_pimpl->m_measNoiseVar /= dt;
    }
    else if (meas.rfInContact)
    {
        constexpr int observationDimension{7};
        constexpr int linearizedDimension{3};
        // prepare observation vector Y
        m_pimpl->m_obs.resize(observationDimension);
        m_pimpl->m_obs << sIMU_p_RF, 0, 1, -1, 0;

        // prepare measurement model Jacobian H
        if (m_options.imuBiasEstimationEnabled)
        {
            m_pimpl->m_measModelJacobian.resize(linearizedDimension, m_pimpl->m_vecSizeWBias);
        }
        else
        {
            m_pimpl->m_measModelJacobian.resize(linearizedDimension, m_pimpl->m_vecSizeWOBias);
        }

        m_pimpl->m_measModelJacobian.topLeftCorner<3, 6>().setZero();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.rContactFramePosition).setIdentity();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.lContactFramePosition).setZero();

        if (m_options.imuBiasEstimationEnabled)
        {
            m_pimpl->m_measModelJacobian.block<3, 6>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.gyroBias).setZero();
        }

        // prepare auxiliary gain matrix Pi
        constexpr int firstFootIGainOffset{0};
        constexpr int firstFootZeroGainOffset{3};
        m_pimpl->m_auxMat.resize(linearizedDimension, observationDimension);
        m_pimpl->m_auxMat.block<3, 3>(firstFootPositionVecOffsetObs, firstFootIGainOffset) = Eigen::Matrix3d::Identity();
        m_pimpl->m_auxMat.block<3, 4>(firstFootPositionVecOffsetObs, firstFootZeroGainOffset).setZero();

        // prepare measurement noise covariance R
        m_pimpl->m_measNoiseVar.resize(linearizedDimension, linearizedDimension);
        m_pimpl->m_measNoiseVar.topLeftCorner<3, 3>() = A_R_IMU*m_pimpl->J_IMURF.topRows<3>()*Renc*(m_pimpl->J_IMURF.topRows<3>().transpose())*(A_R_IMU.transpose());
        m_pimpl->m_measNoiseVar /= dt;
    }
    else if (meas.lfInContact)
    {
        constexpr int observationDimension{7};
        constexpr int linearizedDimension{3};
        // prepare observation vector Y
        m_pimpl->m_obs.resize(observationDimension);
        m_pimpl->m_obs << sIMU_p_LF, 0, 1,  0, -1;

        // prepare measurement model Jacobian H
        if (m_options.imuBiasEstimationEnabled)
        {
            m_pimpl->m_measModelJacobian.resize(linearizedDimension, m_pimpl->m_vecSizeWBias);
        }
        else
        {
            m_pimpl->m_measModelJacobian.resize(linearizedDimension, m_pimpl->m_vecSizeWOBias);
        }

        m_pimpl->m_measModelJacobian.topLeftCorner<3, 6>().setZero();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.rContactFramePosition).setZero();
        m_pimpl->m_measModelJacobian.block<3, 3>(firstFootPositionVecOffsetObs, m_pimpl->m_vecOffsets.lContactFramePosition).setIdentity();

        if (m_options.imuBiasEstimationEnabled)
        {
            m_pimpl->m_measModelJacobian.block<3, 6>(0, m_pimpl->m_vecOffsets.gyroBias).setZero();
        }

        // prepare auxiliary gain matrix Pi
        constexpr int firstFootIGainOffset{0};
        constexpr int firstFootZeroGainOffset{3};
        m_pimpl->m_auxMat.resize(linearizedDimension, observationDimension);
        m_pimpl->m_auxMat.block<3, 3>(firstFootPositionVecOffsetObs, firstFootIGainOffset) = Eigen::Matrix3d::Identity();
        m_pimpl->m_auxMat.block<3, 4>(firstFootPositionVecOffsetObs, firstFootZeroGainOffset).setZero();

        // prepare measurement noise covariance R
        m_pimpl->m_measNoiseVar.resize(linearizedDimension, linearizedDimension);
        m_pimpl->m_measNoiseVar.topLeftCorner<3, 3>() = A_R_IMU*m_pimpl->J_IMULF.topRows<3>()*Renc*(m_pimpl->J_IMULF.topRows<3>().transpose())*(A_R_IMU.transpose());
        m_pimpl->m_measNoiseVar /= dt;
    }

    if (meas.lfInContact || meas.rfInContact)
    {
        if (!m_pimpl->updateStates(m_pimpl->m_obs, m_pimpl->m_measModelJacobian, m_pimpl->m_measNoiseVar, m_pimpl->m_auxMat, m_state, m_pimpl->m_P))
        {
            return false;
        }
        m_pimpl->extractStateVar(m_pimpl->m_P,m_options.imuBiasEstimationEnabled, m_stateStdDev); // unwrap state covariance matrix diagonal
    }

    // should we handle removing old contacts and adding new contacts? or let it be as it is.
    return true;
}


bool InvariantEKFBaseEstimator::Impl::propagateStates(const FloatingBaseEstimators::Measurements& meas,
                                                      const double& dt,
                                                      const Eigen::Vector3d& g,
                                                      const manif::SE3d& IMU_H_RF,
                                                      const manif::SE3d& IMU_H_LF,
                                                      FloatingBaseEstimators::InternalState& X)
{
    Eigen::Vector3d acc_unbiased = meas.acc - X.accelerometerBias;
    Eigen::Vector3d gyro_unbiased = meas.gyro - X.gyroscopeBias;

    Eigen::Matrix3d R = X.imuOrientation.toRotationMatrix();
    Eigen::Vector3d v = X.imuLinearVelocity;
    Eigen::Vector3d p = X.imuPosition;
    Eigen::Vector3d acc = (R*acc_unbiased) + g;

    manif::SO3Tangentd omega_skew_dt(gyro_unbiased*dt);
    Eigen::Matrix3d R_pred = R*(omega_skew_dt.exp().rotation());

    X.imuOrientation = Eigen::Quaterniond(R_pred);
    X.imuLinearVelocity = v + acc*dt;
    X.imuPosition = p + (v*dt) + (acc*(0.5*dt*dt));

    if (!meas.lfInContact)
    {
        X.lContactFramePosition = X.imuPosition + (R*IMU_H_LF.translation());
    }

    if (!meas.rfInContact)
    {
        X.rContactFramePosition = X.imuPosition + (R*IMU_H_RF.translation());
    }

    return true;
}

bool InvariantEKFBaseEstimator::Impl::updateStates(const Eigen::VectorXd& obs,
                                                   const Eigen::MatrixXd measModelJacobian,
                                                   const Eigen::MatrixXd& measNoiseVar,
                                                   const Eigen::MatrixXd& auxMat,
                                                   FloatingBaseEstimators::InternalState& state,
                                                   Eigen::MatrixXd& P)
{
    if (measModelJacobian.cols() != P.rows())
    {
        std::cerr << "[InvariantEKFBaseEstimator::updateStates] Measurement model Jacobian size mismatch" << std::endl;
        return false;
    }

    if (measModelJacobian.rows() != measNoiseVar.rows())
    {
        std::cerr << "[InvariantEKFBaseEstimator::updateStates] Measurement noise covariance matrix size mismatch" << std::endl;
        return false;
    }

    bool estimateBias;
    if (P.rows() == m_vecSizeWBias)
    {
        estimateBias = true;
    }
    else
    {
        estimateBias = false;
    }

    m_PHT = P*measModelJacobian.transpose();
    m_S = measModelJacobian*m_PHT + measNoiseVar;
    m_K = m_PHT*(m_S.inverse());

    // compute innovation delta
    extractState(state, m_X, m_theta);

    int copyTimes = obs.rows()/m_X.cols();
    if (!copyDiagX(m_X, copyTimes, m_BigX))
    {
        return false;
    }

    m_innovation = m_BigX*obs; // minus bias terms which are zero in reduced form (Section III C of the paper)
    m_delta = m_K*auxMat*m_innovation;

    m_deltaTheta.setZero();
    if (estimateBias)
    {
        m_deltaTheta = m_delta.segment<6>(m_vecSizeWOBias);
    }

    // update state
    if (!calcExpHatX(m_delta.segment(0, m_vecSizeWOBias), m_dX))
    {
        std::cerr << "[InvariantEKFBaseEstimator::updateStates] Could not compute state update";
        return false;
    }

    if (!constructState( m_dX*m_X, m_theta+m_deltaTheta, state) ) // right invariant update
    {
        std::cerr << "[InvariantEKFBaseEstimator::updateStates] Could not update state";
        return false;
    }
    // update covariance
    m_IminusKH = Eigen::MatrixXd::Identity(P.rows(), P.cols()) - m_K*measModelJacobian;
    P = m_IminusKH*P*(m_IminusKH.transpose()) + m_K*measNoiseVar*(m_K.transpose());
    return true;
}

void InvariantEKFBaseEstimator::Impl::extractStateVar(const Eigen::MatrixXd& P,
                                                      const bool& estimateBias,
                                                      FloatingBaseEstimators::StateStdDev& stateStdDev)
{
    stateStdDev.imuOrientation =  P.block<3, 3>(m_vecOffsets.imuOrientation, m_vecOffsets.imuOrientation).diagonal().array().sqrt();
    stateStdDev.imuLinearVelocity =  P.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuLinearVel).diagonal().array().sqrt();
    stateStdDev.imuPosition =  P.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.imuPosition).diagonal().array().sqrt();
    stateStdDev.rContactFramePosition =  P.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.rContactFramePosition).diagonal().array().sqrt();
    stateStdDev.lContactFramePosition =  P.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.lContactFramePosition).diagonal().array().sqrt();
    if (estimateBias)
    {
        stateStdDev.gyroscopeBias =  P.block<3, 3>(m_vecOffsets.gyroBias, m_vecOffsets.gyroBias).diagonal().array().sqrt();
        stateStdDev.accelerometerBias =  P.block<3, 3>(m_vecOffsets.accBias, m_vecOffsets.accBias).diagonal().array().sqrt();
    }
}

void InvariantEKFBaseEstimator::Impl::constuctStateVar(const FloatingBaseEstimators::StateStdDev& stateStdDev,
                                                       const bool& estimateBias,
                                                       Eigen::MatrixXd& P)
{
    if (estimateBias)
    {
        P.resize(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        P.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    }

    P.setZero();
    Eigen::Vector3d temp;

    temp = stateStdDev.imuOrientation.array().square();
    P.block<3, 3>(m_vecOffsets.imuOrientation, m_vecOffsets.imuOrientation) = temp.asDiagonal();
    temp = stateStdDev.imuLinearVelocity.array().square();
    P.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuLinearVel) = temp.asDiagonal();
    temp = stateStdDev.imuPosition.array().square();
    P.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.imuPosition) = temp.asDiagonal();
    temp = stateStdDev.rContactFramePosition.array().square();
    P.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.rContactFramePosition) = temp.asDiagonal();
    temp = stateStdDev.lContactFramePosition.array().square();
    P.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.lContactFramePosition) = temp.asDiagonal();

    if (estimateBias)
    {
        temp = stateStdDev.gyroscopeBias.array().square();
        P.block<3, 3>(m_vecOffsets.gyroBias, m_vecOffsets.gyroBias) = temp.asDiagonal();
        temp = stateStdDev.accelerometerBias.array().square();
        P.block<3, 3>(m_vecOffsets.accBias, m_vecOffsets.accBias) = temp.asDiagonal();
    }
}

void InvariantEKFBaseEstimator::Impl::extractState(const FloatingBaseEstimators::InternalState& state,
                                                   Eigen::MatrixXd& X,
                                                   Eigen::Matrix<double, 6, 1>& theta)
{
    // X =    |R  v  p pr pl|
    //        |   1         |
    //        |      1      |
    //        |         1   |
    //        |            1|
    X.resize(7, 7);
    X.block<3, 3>(0, 0) = state.imuOrientation.toRotationMatrix();
    X.block<3, 1>(0, 3) = state.imuLinearVelocity;
    X.block<3, 1>(0, 4) = state.imuPosition;
    X.block<3, 1>(0, 5) = state.rContactFramePosition;
    X.block<3, 1>(0, 6) = state.lContactFramePosition;
    X.bottomLeftCorner<4, 3>().setZero();
    X.bottomRightCorner<4, 4>() = Eigen::Matrix4d::Identity();

    theta.segment<3>(0) = state.gyroscopeBias;
    theta.segment<3>(3) = state.accelerometerBias;
}

bool InvariantEKFBaseEstimator::Impl::constructState(const Eigen::MatrixXd& X,
                                                     const Eigen::Matrix<double, 6, 1>& theta,
                                                     FloatingBaseEstimators::InternalState& state)
{
    if (X.rows() != 7 && X.cols() != 7)
    {
        std::cerr << "[InvariantEKFBaseEstimator::constructState] State matrix does not seem to have expected size of 7x7." << std::endl;
        return false;
    }

    state.imuOrientation = Eigen::Quaterniond(X.block<3, 3>(0, 0));
    state.imuLinearVelocity = X.block<3, 1>(0, 3);
    state.imuPosition = X.block<3, 1>(0, 4);
    state.rContactFramePosition = X.block<3, 1>(0, 5) ;
    state.lContactFramePosition = X.block<3, 1>(0, 6) ;
    state.gyroscopeBias = theta.segment<3>(0);
    state.accelerometerBias = theta.segment<3>(3);
    return true;
}

bool InvariantEKFBaseEstimator::Impl::calcExpHatX(const Eigen::VectorXd& vec,
                                                  Eigen::MatrixXd& X)
{
    // Exp(vec) = Exp([ w]) =    |ExpSO3(w)  JlSO3(w)v  JlSO3(w)p  JlSO3(w)pr  JlSO3(w)pl|
    //               ([ a])      |                   1                                   |
    //               ([ v])      |                              1                        |
    //               ([vr])      |                                         1             |
    //               ([vl])      |                                                      1|
    // where JlSO3 is the left Jacobian of SO(3)
    if (vec.size() != 15)
    {
        std::cerr << "[InvariantEKFBaseEstimator::calcExpHatX] State vector does not seem to have expected size of 15x1." << std::endl;
        return false;
    }

    X.resize(7, 7);
    auto omega = manif::SO3Tangentd(vec.segment<3>(m_vecOffsets.imuOrientation));
    X.topLeftCorner<3, 3>() = omega.exp().rotation();
    X.block<3, 1>(0, 3) = omega.ljac()*vec.segment<3>(m_vecOffsets.imuLinearVel);
    X.block<3, 1>(0, 4) = omega.ljac()*vec.segment<3>(m_vecOffsets.imuPosition);
    X.block<3, 1>(0, 5) = omega.ljac()*vec.segment<3>(m_vecOffsets.rContactFramePosition);
    X.block<3, 1>(0, 6) = omega.ljac()*vec.segment<3>(m_vecOffsets.lContactFramePosition);

    X.bottomLeftCorner<4, 3>().setZero();
    X.bottomRightCorner<4, 4>() = Eigen::Matrix4d::Identity();
    return true;
}

void InvariantEKFBaseEstimator::Impl::calcAdjointX(const FloatingBaseEstimators::InternalState& X,
                                                   const SkewSymContainter& skew,
                                                   Eigen::MatrixXd& AdjX)
{
    // AdjX = |   R         |
    //        | vxR  R      |
    //        | pxR    R    |
    //        |prxR      R  |
    //        |plxR        R|
    AdjX.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    AdjX.setZero();
    Eigen::Matrix3d R = X.imuOrientation.toRotationMatrix();

    AdjX.block<3, 3>(m_vecOffsets.imuOrientation, m_vecOffsets.imuOrientation) =
    AdjX.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuLinearVel) =
    AdjX.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.imuPosition) =
    AdjX.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.rContactFramePosition) =
    AdjX.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.lContactFramePosition) = R;

    AdjX.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuOrientation) = (skew.vCross)*R;
    AdjX.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.imuOrientation) = (skew.pCross)*R;
    AdjX.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.imuOrientation) = (skew.prCross)*R;
    AdjX.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.imuOrientation) = (skew.plCross)*R;
}

void InvariantEKFBaseEstimator::Impl::calcFc(const FloatingBaseEstimators::InternalState& X,
                                             const SkewSymContainter& skew,
                                             const bool& estimateBias, Eigen::MatrixXd& Fc)
{
    // When biases are enabled,
    // Fc = [   0   0   0   0       -R   0]
    //      [ S(g)  0   0   0   -S(v)R  -R]
    //      [   0   I   0   0   -S(p)R   0]
    //      [   0   0   0   0  -S(pr)R   0]
    //      [   0   0   0   0  -S(pl)R   0]
    //      [   0   0   0   0        0   0]
    //      [   0   0   0   0        0   0]
    // when biases are disabled, ignore last two rows and columns
    if (estimateBias)
    {
        Fc.resize(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        Fc.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    }
    Fc.setZero();

    Eigen::Matrix3d I3 = Eigen::Matrix3d::Identity();
    Fc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuOrientation) = skew.gCross;
    Fc.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.imuLinearVel) = I3;

    if (estimateBias)
    {
        Eigen::Matrix3d R = X.imuOrientation.toRotationMatrix();

        Fc.block<3, 3>(m_vecOffsets.imuOrientation, m_vecOffsets.gyroBias) = -R;

        Fc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.gyroBias) = -(skew.vCross)*R;
        Fc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.accBias) = -R;

        Fc.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.gyroBias) = -(skew.pCross)*R;
        Fc.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.gyroBias) = -(skew.prCross)*R;
        Fc.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.gyroBias) = -(skew.plCross)*R;
    }
}

void InvariantEKFBaseEstimator::Impl::calcQc(const FloatingBaseEstimators::InternalState& X,
                                             const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                             const FloatingBaseEstimators::Measurements& meas,
                                             const bool& estimateBias,
                                             Eigen::MatrixXd& Qc)
{
    // When biases are enabled,
    // Qc = blkdiag(Qg, Qa, 0, Qrf, Qlf, Qbg, Qba)
    // Qrf and Qlf depend on the feet contact states
    if (estimateBias)
    {
        Qc.resize(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        Qc.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    }
    Qc.setZero();

    Eigen::Vector3d Qa = sensDev.accelerometerNoise.array().square();
    Eigen::Vector3d Qg = sensDev.gyroscopeNoise.array().square();

    Qc.block<3, 3>(m_vecOffsets.imuOrientation, m_vecOffsets.imuOrientation) = static_cast<Eigen::Matrix3d>(Qg.asDiagonal());
    Qc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuLinearVel) = static_cast<Eigen::Matrix3d>(Qa.asDiagonal());

    if (estimateBias)
    {
        Eigen::Vector3d Qba = sensDev.accelerometerBiasNoise.array().square();
        Eigen::Vector3d Qbg = sensDev.gyroscopeBiasNoise.array().square();
        Qc.block<3, 3>(m_vecOffsets.gyroBias, m_vecOffsets.gyroBias) = static_cast<Eigen::Matrix3d>(Qbg.asDiagonal());
        Qc.block<3, 3>(m_vecOffsets.accBias, m_vecOffsets.accBias) = static_cast<Eigen::Matrix3d>(Qba.asDiagonal());
    }

    Eigen::Matrix3d R = X.imuOrientation.toRotationMatrix();
    Eigen::Vector3d Qlf, Qrf;
    if (meas.lfInContact)
    {
        (Qlf = sensDev.contactFootLinvelNoise.array().square());
    }
    else
    {
        (Qlf = sensDev.swingFootLinvelNoise.array().square());
    }

    if (meas.rfInContact)
    {
        (Qrf = sensDev.contactFootLinvelNoise.array().square());
    }
    else
    {
        (Qrf = sensDev.swingFootLinvelNoise.array().square());
    }

    Qc.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.rContactFramePosition) = R*static_cast<Eigen::Matrix3d>(Qrf.asDiagonal())*(R.transpose());
    Qc.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.lContactFramePosition) = R*static_cast<Eigen::Matrix3d>(Qlf.asDiagonal())*(R.transpose());
}

void InvariantEKFBaseEstimator::Impl::calcLc(const FloatingBaseEstimators::InternalState& X,
                                             const SkewSymContainter& skew,
                                             const bool& estimateBias,
                                             Eigen::MatrixXd& Lc)
{
    // When biases are enabled,
    // Lc = blkdiag(AdjX(X), I, I)
    // when biases are disabledm
    // Lc = AdjX(X)
    if (estimateBias)
    {
        Lc.resize(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        Lc.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    }
    Eigen::MatrixXd AdjX;
    calcAdjointX(X, skew, AdjX);
    Lc.block(m_vecOffsets.imuOrientation, m_vecOffsets.imuOrientation, m_vecSizeWOBias, m_vecSizeWOBias) = AdjX;

    if (estimateBias)
    {
        Lc.block(m_vecOffsets.gyroBias, m_vecOffsets.gyroBias, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
    }
}

void InvariantEKFBaseEstimator::Impl::calcSkewSymAtCurrenState(const FloatingBaseEstimators::InternalState& X,
                                                               SkewSymContainter& skew)
{
    skew.vCross = iDynTree::skew(X.imuLinearVelocity);
    skew.pCross = iDynTree::skew(X.imuPosition);
    skew.plCross = iDynTree::skew(X.lContactFramePosition);
    skew.prCross = iDynTree::skew(X.rContactFramePosition);
}


bool InvariantEKFBaseEstimator::Impl::copyDiagX(const Eigen::MatrixXd& X,
                                                const int& n,
                                                Eigen::MatrixXd& BigX)
{
    if (X.rows() != 7 && X.cols() != 7)
    {
        std::cerr << "[InvariantEKFBaseEstimator::copyDiagX] State matrix does not seem to have expected size of 7x7." << std::endl;
        return false;
    }

    BigX.resize(0, 0);
    int dimX = X.cols();
    for (int i = 0; i < n; ++i)
    {
        int startIndex = BigX.rows();
        BigX.conservativeResize(startIndex + dimX, startIndex + dimX);
        BigX.block(startIndex, 0, dimX, startIndex) = Eigen::MatrixXd::Zero(dimX, startIndex);
        BigX.block(0, startIndex, startIndex, dimX) = Eigen::MatrixXd::Zero(startIndex, dimX);
        BigX.block(startIndex, startIndex, dimX, dimX) = X;
    }
    return true;
}

