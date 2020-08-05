/**
 * @file InvariantEKFBaseEstimator.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/InvariantEKFBaseEstimator.h>
#include <iDynTree/Core/EigenHelpers.h>

#include <manif/manif.h>

using namespace BipedalLocomotion::Estimators;

class InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl
{
public:
    struct SkewSymContainter
    {
        Eigen::Matrix3d gCross;
        Eigen::Matrix3d vCross;
        Eigen::Matrix3d pCross;
        Eigen::Matrix3d prCross;
        Eigen::Matrix3d plCross;
    };

    bool constructState(const Eigen::MatrixXd& X, const Eigen::Matrix<double, 6, 1>& theta, FloatingBaseEstimators::InternalState& state);

    void extractState(const FloatingBaseEstimators::InternalState& state, Eigen::MatrixXd& X, Eigen::Matrix<double, 6, 1>& theta);

    void constuctStateVar(const FloatingBaseEstimators::StateStdDev& stateStdDev, const bool& estimateBias, Eigen::MatrixXd& P);

    void extractStateVar(const Eigen::MatrixXd& P, const bool& estimateBias, FloatingBaseEstimators::StateStdDev& stateStdDev);

    bool propagateStates(const FloatingBaseEstimators::Measurements& meas,
                         const double& dt, const Eigen::Vector3d& g,
                         const iDynTree::Transform& IMU_H_RF,
                         const iDynTree::Transform& IMU_H_LF,
                         FloatingBaseEstimators::InternalState& X);

    bool calcExpHatSE_2_3(const Eigen::VectorXd& vec, Eigen::MatrixXd& X);

    void calcAdjointSE_2_3(const FloatingBaseEstimators::InternalState& X, const SkewSymContainter& skew, Eigen::MatrixXd& AdjX);

    bool copyDiagX(const Eigen::MatrixXd& X, const int& n, Eigen::MatrixXd& BigX);

    void calcQc(const FloatingBaseEstimators::InternalState& X, const FloatingBaseEstimators::SensorsStdDev& sensStdDev,
                const FloatingBaseEstimators::Measurements& meas, const bool& estimateBias, Eigen::MatrixXd& Qc);

    void calcLc(const FloatingBaseEstimators::InternalState& X, const SkewSymContainter& skew, const bool& estimateBias, Eigen::MatrixXd& Lc);

    void calcFc(const FloatingBaseEstimators::InternalState& X, const SkewSymContainter& skew, const bool& estimateBias,   Eigen::MatrixXd& Fc);

    void calcSkewSymAtCurrenState(const FloatingBaseEstimators::InternalState& X, SkewSymContainter& skew);



    bool updateStates(const Eigen::VectorXd& obs,
                      const Eigen::MatrixXd measModelJacobian, const Eigen::MatrixXd& measNoiseVar, const Eigen::MatrixXd& auxMat,
                      FloatingBaseEstimators::InternalState& state,
                      Eigen::MatrixXd& P);

    SkewSymContainter m_skew;

    Eigen::MatrixXd m_P;

    const size_t m_vecSizeWOBias{15};
    const size_t m_vecSizeWBias{21};

    struct {
    const size_t imuOrientation{0};
    const size_t imuLinearVel{3};
    const size_t imuPosition{6};
    const size_t rContactFramePosition{9};
    const size_t lContactFramePosition{12};
    const size_t gyroBias{15};
    const size_t accBias{18};
    } m_vecOffsets;

    friend class InvariantEKFBaseEstimator;
};

InvariantEKFBaseEstimator::InvariantEKFBaseEstimator() : m_pimpl(std::make_unique<InvariantEKFBaseEstimatorImpl>())
{
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

    m_pimpl->constuctStateVar(m_priors, m_options.imuBiasEstimationEnabled, m_pimpl->m_P); // construct priors
    return true;
}

bool InvariantEKFBaseEstimator::predictState(const FloatingBaseEstimators::Measurements& meas,
                                             const double& dt)
{
    m_pimpl->calcSkewSymAtCurrenState(m_statePrev, m_pimpl->m_skew); // compute skews at priori state

    // update foot position predictions with previous measures
    iDynTree::JointPosDoubleArray jointPos(meas.encoders.size());
    iDynTree::toEigen(jointPos) = meas.encoders;
    iDynTree::Transform IMU_H_LF, IMU_H_RF;
    if (!m_modelComp.getIMU_H_feet(jointPos, IMU_H_LF, IMU_H_RF))
    {
        return false;
    }

    // m_state is now predicted state after this function call
    if (!m_pimpl->propagateStates(meas, dt, m_options.accelerationDueToGravity, IMU_H_RF, IMU_H_LF, m_state))
    {
        return false;
    }

    Eigen::MatrixXd Fc, Qc, Lc;
    m_pimpl->calcFc(m_statePrev, m_pimpl->m_skew, m_options.imuBiasEstimationEnabled, Fc); // compute Fc at priori state
    m_pimpl->calcQc(m_statePrev, m_sensorsDev, meas,  m_options.imuBiasEstimationEnabled, Qc); // compute Qc at priori state and previous measure
    m_pimpl->calcLc(m_statePrev, m_pimpl->m_skew, m_options.imuBiasEstimationEnabled, Lc); // compute Lc at priori state

    // discretize linearized dynamics and propagate covariance
    size_t dims = Fc.rows();
    Eigen::MatrixXd I = Eigen::MatrixXd::Identity(dims, dims);

    auto Fk = I + (Fc*dt);
    auto Qk = (Fk*Lc*Qc*(Lc.transpose())*(Fk.transpose()))*dt;
    m_pimpl->m_P = Fk*m_pimpl->m_P*(Fk.transpose()) + Qk;
    m_pimpl->extractStateVar(m_pimpl->m_P,m_options.imuBiasEstimationEnabled, m_stateStdDev); // unwrap state covariance matrix diagonal

    return true;
}

bool InvariantEKFBaseEstimator::updateKinematics(const FloatingBaseEstimators::Measurements& meas,
                                                 const double& dt)
{
    Eigen::VectorXd obs;
    Eigen::MatrixXd measModelJacobian, measNoiseVar, auxMat;
    auto A_R_IMU = m_state.imuOrientation.toRotationMatrix();

    iDynTree::JointPosDoubleArray jointPos(meas.encoders.size());
    iDynTree::toEigen(jointPos) = meas.encoders;
    iDynTree::Transform IMU_H_LF, IMU_H_RF;
    iDynTree::MatrixDynSize J_IMULF, J_IMURF;
    m_modelComp.getIMU_H_feet(jointPos, IMU_H_LF, IMU_H_RF, J_IMULF, J_IMURF);

    Eigen::Vector3d sIMU_p_LF = iDynTree::toEigen(IMU_H_LF.getPosition());
    Eigen::Vector3d sIMU_p_RF = iDynTree::toEigen(IMU_H_RF.getPosition());
    auto Jl{iDynTree::toEigen(J_IMULF)};
    auto Jr{iDynTree::toEigen(J_IMURF)};

    Eigen::VectorXd encodersVar = m_sensorsDev.encodersNoise.array().square();
    Eigen::MatrixXd Renc = static_cast<Eigen::MatrixXd>(encodersVar.asDiagonal());

    if (meas.lfInContact && meas.rfInContact)
    {
        // prepare observation vector Y
        obs.resize(14);
        obs << sIMU_p_RF, 0, 1, -1, 0,
               sIMU_p_LF, 0, 1,  0, -1;

        // prepare measurement model Jacobian H
        if (m_options.imuBiasEstimationEnabled)
        {
            measModelJacobian.resize(6, m_pimpl->m_vecSizeWBias);
        }
        else
        {
            measModelJacobian.resize(6, m_pimpl->m_vecSizeWOBias);
        }

        measModelJacobian.topLeftCorner<6, 6>().setZero();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.rContactFramePosition).setIdentity();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.lContactFramePosition).setZero();
        measModelJacobian.block<3, 3>(3, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        measModelJacobian.block<3, 3>(3, m_pimpl->m_vecOffsets.rContactFramePosition).setZero();
        measModelJacobian.block<3, 3>(3, m_pimpl->m_vecOffsets.lContactFramePosition).setIdentity();

        if (m_options.imuBiasEstimationEnabled)
        {
            measModelJacobian.block<6, 6>(0, m_pimpl->m_vecOffsets.gyroBias).setZero();
        }

        // prepare auxiliary gain matrix Pi
        auxMat.resize(6, 14);
        auxMat.block<3, 3>(0, 0) = auxMat.block<3, 3>(3, 7) = Eigen::Matrix3d::Identity();
        auxMat.block<3, 11>(0, 3).setZero();
        auxMat.block<3, 7>(3, 0).setZero();
        auxMat.block<3, 4>(3, 10).setZero();

        // prepare measurement noise covariance R
        measNoiseVar.resize(6, 6);
        measNoiseVar.topLeftCorner<3, 3>() = A_R_IMU*Jr.topRows<3>()*Renc*(Jr.topRows<3>().transpose())*(A_R_IMU.transpose());
        measNoiseVar.topRightCorner<3, 3>().setZero();
        measNoiseVar.bottomRightCorner<3, 3>() = A_R_IMU*Jl.topRows<3>()*Renc*(Jl.topRows<3>().transpose())*(A_R_IMU.transpose());
        measNoiseVar.bottomLeftCorner<3, 3>().setZero();
        measNoiseVar /= dt;
    }
    else if (meas.rfInContact)
    {
        // prepare observation vector Y
        obs.resize(7);
        obs << sIMU_p_RF, 0, 1, -1, 0;

        // prepare measurement model Jacobian H
        if (m_options.imuBiasEstimationEnabled)
        {
            measModelJacobian.resize(3, m_pimpl->m_vecSizeWBias);
        }
        else
        {
            measModelJacobian.resize(3, m_pimpl->m_vecSizeWOBias);
        }

        measModelJacobian.topLeftCorner<3, 6>().setZero();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.rContactFramePosition).setIdentity();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.lContactFramePosition).setZero();

        if (m_options.imuBiasEstimationEnabled)
        {
            measModelJacobian.block<3, 6>(0, m_pimpl->m_vecOffsets.gyroBias).setZero();
        }

        // prepare auxiliary gain matrix Pi
        auxMat.resize(3, 7);
        auxMat.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        auxMat.block<3, 4>(0, 3).setZero();

        // prepare measurement noise covariance R
        measNoiseVar.resize(3, 3);
        measNoiseVar.topLeftCorner<3, 3>() = A_R_IMU*Jr.topRows<3>()*Renc*(Jr.topRows<3>().transpose())*(A_R_IMU.transpose());
        measNoiseVar /= dt;
    }
    else if (meas.lfInContact)
    {
        // prepare observation vector Y
        obs.resize(7);
        obs << sIMU_p_LF, 0, 1,  0, -1;

        // prepare measurement model Jacobian H
        if (m_options.imuBiasEstimationEnabled)
        {
            measModelJacobian.resize(3, m_pimpl->m_vecSizeWBias);
        }
        else
        {
            measModelJacobian.resize(3, m_pimpl->m_vecSizeWOBias);
        }

        measModelJacobian.topLeftCorner<3, 6>().setZero();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.imuPosition) = -Eigen::Matrix3d::Identity();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.rContactFramePosition).setZero();
        measModelJacobian.block<3, 3>(0, m_pimpl->m_vecOffsets.lContactFramePosition).setIdentity();

        if (m_options.imuBiasEstimationEnabled)
        {
            measModelJacobian.block<3, 6>(0, m_pimpl->m_vecOffsets.gyroBias).setZero();
        }

        // prepare auxiliary gain matrix Pi
        auxMat.resize(3, 7);
        auxMat.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
        auxMat.block<3, 4>(0, 3).setZero();

        // prepare measurement noise covariance R
        measNoiseVar.resize(3, 3);
        measNoiseVar.topLeftCorner<3, 3>() = A_R_IMU*Jl.topRows<3>()*Renc*(Jl.topRows<3>().transpose())*(A_R_IMU.transpose());
        measNoiseVar /= dt;
    }

    if (meas.lfInContact || meas.rfInContact)
    {
        if (!m_pimpl->updateStates(obs, measModelJacobian, measNoiseVar, auxMat, m_state, m_pimpl->m_P))
        {
            return false;
        }
        m_pimpl->extractStateVar(m_pimpl->m_P,m_options.imuBiasEstimationEnabled, m_stateStdDev); // unwrap state covariance matrix diagonal
    }

    // should we handle removing old contacts and adding new contacts? or let it be as it is.
    return true;
}


bool InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::propagateStates(const FloatingBaseEstimators::Measurements& meas,
                                                                               const double& dt,
                                                                               const Eigen::Vector3d& g,
                                                                               const iDynTree::Transform& IMU_H_RF,
                                                                               const iDynTree::Transform& IMU_H_LF,
                                                                               FloatingBaseEstimators::InternalState& X)
{
    auto acc_unbiased = meas.acc - X.accelerometerBias;
    auto gyro_unbiased = meas.gyro - X.gyroscopeBias;

    auto R = X.imuOrientation.toRotationMatrix();
    auto v = X.imuLinearVelocity;
    auto p = X.imuPosition;

    auto acc = (R*acc_unbiased) + g;

    manif::SO3Tangentd omega_skew_dt(gyro_unbiased*dt);
    auto R_pred = R*(omega_skew_dt.exp().rotation());

    X.imuOrientation = Eigen::Quaterniond(R_pred);
    X.imuLinearVelocity = v + acc*dt;
    X.imuPosition = p + (v*dt) + (acc*(0.5*dt*dt));

    if (!meas.lfInContact)
    {
        X.lContactFramePosition = X.imuPosition + (R*iDynTree::toEigen(IMU_H_LF.getPosition()));
    }

    if (!meas.rfInContact)
    {
        X.rContactFramePosition = X.imuPosition + (R*iDynTree::toEigen(IMU_H_RF.getPosition()));
    }

    return true;
}

bool InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::updateStates(const Eigen::VectorXd& obs,
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

    auto PHT = P*measModelJacobian.transpose();
    auto S = measModelJacobian*PHT + measNoiseVar;
    auto K = PHT*(S.inverse());

    // compute innovation delta
    Eigen::MatrixXd X;
    Eigen::Matrix<double, 6, 1> theta;
    extractState(state, X, theta);

    int copyTimes = obs.rows()/X.cols();
    Eigen::MatrixXd BigX;
    if (!copyDiagX(X, copyTimes, BigX))
    {
        return false;
    }

    Eigen::MatrixXd innovation = BigX*obs; // minus bias terms which are zero in reduced form (Section III C of the paper)
    Eigen::VectorXd delta = K*auxMat*innovation;

    Eigen::Matrix<double, 6, 1> deltaTheta;
    deltaTheta.setZero();
    if (estimateBias)
    {
        deltaTheta = delta.segment<6>(m_vecSizeWOBias);
    }

    // update state
    Eigen::MatrixXd dX;

    if (!calcExpHatSE_2_3(delta.segment(0, m_vecSizeWOBias), dX))
    {
        std::cerr << "[InvariantEKFBaseEstimator::updateStates] Could not compute state update";
        return false;
    }

    if (!constructState( dX*X, theta+deltaTheta, state) ) // right invariant update
    {
        std::cerr << "[InvariantEKFBaseEstimator::updateStates] Could not update state";
        return false;
    }
    // update covariance
    auto ImKH = Eigen::MatrixXd::Identity(P.rows(), P.cols()) - K*measModelJacobian;
    P = ImKH*P*(ImKH.transpose()) + K*measNoiseVar*(K.transpose());
    return true;
}

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::extractStateVar(const Eigen::MatrixXd& P, const bool& estimateBias, FloatingBaseEstimators::StateStdDev& stateStdDev)
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

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::constuctStateVar(const FloatingBaseEstimators::StateStdDev& stateStdDev, const bool& estimateBias, Eigen::MatrixXd& P)
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

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::extractState(const FloatingBaseEstimators::InternalState& state, Eigen::MatrixXd& X, Eigen::Matrix<double, 6, 1>& theta)
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

bool InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::constructState(const Eigen::MatrixXd& X, const Eigen::Matrix<double, 6, 1>& theta, FloatingBaseEstimators::InternalState& state)
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

bool InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::calcExpHatSE_2_3(const Eigen::VectorXd& vec,
                                                                                Eigen::MatrixXd& X)
{
    if (vec.size() != 15)
    {
        std::cerr << "[InvariantEKFBaseEstimator::calcExpHatSE_2_3] State vector does not seem to have expected size of 15x1." << std::endl;
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

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::calcAdjointSE_2_3(const FloatingBaseEstimators::InternalState& X,
                                                                                 const SkewSymContainter& skew,
                                                                                 Eigen::MatrixXd& AdjX)
{
    AdjX.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    AdjX.setZero();
    auto R = X.imuOrientation.toRotationMatrix();

    // AdjX = |   R         |
    //        | vxR  R      |
    //        | pxR    R    |
    //        |prxR      R  |
    //        |plxR        R|

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

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::calcFc(const FloatingBaseEstimators::InternalState& X,
                                                                      const SkewSymContainter& skew,
                                                                      const bool& estimateBias, Eigen::MatrixXd& Fc)
{
    if (estimateBias)
    {
        Fc.resize(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        Fc.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    }
    Fc.setZero();

    auto I3 = Eigen::Matrix3d::Identity();
    Fc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.imuOrientation) = skew.gCross;
    Fc.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.imuLinearVel) = I3;

    if (estimateBias)
    {
        auto R = X.imuOrientation.toRotationMatrix();

        Fc.block<3, 3>(m_vecOffsets.imuOrientation, m_vecOffsets.gyroBias) = -R;

        Fc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.gyroBias) = -(skew.vCross)*R;
        Fc.block<3, 3>(m_vecOffsets.imuLinearVel, m_vecOffsets.accBias) = -R;

        Fc.block<3, 3>(m_vecOffsets.imuPosition, m_vecOffsets.gyroBias) = -(skew.pCross)*R;
        Fc.block<3, 3>(m_vecOffsets.rContactFramePosition, m_vecOffsets.gyroBias) = -(skew.prCross)*R;
        Fc.block<3, 3>(m_vecOffsets.lContactFramePosition, m_vecOffsets.gyroBias) = -(skew.plCross)*R;
    }
}

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::calcQc(const FloatingBaseEstimators::InternalState& X,
                                                                      const FloatingBaseEstimators::SensorsStdDev& sensDev,
                                                                      const FloatingBaseEstimators::Measurements& meas,
                                                                      const bool& estimateBias,
                                                                      Eigen::MatrixXd& Qc)
{
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

    auto R = X.imuOrientation.toRotationMatrix();
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

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::calcLc(const FloatingBaseEstimators::InternalState& X,
                                                                      const SkewSymContainter& skew,
                                                                      const bool& estimateBias, Eigen::MatrixXd& Lc)
{
    if (estimateBias)
    {
        Lc.resize(m_vecSizeWBias, m_vecSizeWBias);
    }
    else
    {
        Lc.resize(m_vecSizeWOBias, m_vecSizeWOBias);
    }
    Eigen::MatrixXd AdjX;
    calcAdjointSE_2_3(X, skew, AdjX);
    Lc.block(m_vecOffsets.imuOrientation, m_vecOffsets.imuOrientation, m_vecSizeWOBias, m_vecSizeWOBias) = AdjX;

    if (estimateBias)
    {
        Lc.block(m_vecOffsets.gyroBias, m_vecOffsets.gyroBias, 6, 6) = Eigen::Matrix<double, 6, 6>::Identity();
    }
}

void InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::calcSkewSymAtCurrenState(const FloatingBaseEstimators::InternalState& X,
                                                                                        SkewSymContainter& skew)
{
    skew.vCross = iDynTree::skew(X.imuLinearVelocity);
    skew.pCross = iDynTree::skew(X.imuPosition);
    skew.plCross = iDynTree::skew(X.lContactFramePosition);
    skew.prCross = iDynTree::skew(X.rContactFramePosition);
}


bool InvariantEKFBaseEstimator::InvariantEKFBaseEstimatorImpl::copyDiagX(const Eigen::MatrixXd& X,
                                                                         const int& n,
                                                                         Eigen::MatrixXd& BigX)
{
    if (X.rows() != 7 && X.cols() != 7)
    {
        std::cerr << "[InvariantEKFBaseEstimator::copyDiagX] State matrix does not seem to have expected size of 7x7." << std::endl;
        return false;
    }

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

