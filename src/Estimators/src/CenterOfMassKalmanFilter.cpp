/**
 * @file CenterOfMassKalmanFilter.cpp
 * @authors Prashanth Ramadoss
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotion/FloatingBaseEstimators/CenterOfMassKalmanFilter.h>
#include <BipedalLocomotion/TextLogging/Logger.h>
#include <BipedalLocomotion/Math/Constants.h>
#include <iDynTree/Core/EigenHelpers.h>
#include <iDynTree/Model/Model.h>

using namespace BipedalLocomotion::Estimators;
using namespace BipedalLocomotion;

class CenterOfMassKalmanFilter::Impl
{
public:
    // KF matrices
    void prepareAandB();
    void prepareCandD();
    void prepareQandR();
    void prepareMeasurementVector();

    bool predict();
    bool update();

    Eigen::Matrix<double, 6, 1> x;

    Eigen::Matrix<double, 6, 6> A, I6;
    Eigen::Matrix<double, 6, 2> B;
    Eigen::Matrix<double, 4, 6> C;
    Eigen::Matrix<double, 4, 2> D;

    Eigen::Matrix<double, 6, 6> P;
    Eigen::Matrix<double, 6, 6> Q;
    Eigen::Matrix<double, 4, 4> R, S;
    Eigen::Matrix<double, 6, 4> K, CT;

    Eigen::Vector4d z, y;
    double sigma2_y_com, sigma2_y_ddcom;
    double sigma2_x_com, sigma2_dx_com, sigma2_xoff_com;

    Eigen::Matrix2d I2;
    double dt;
    double omegaSquared;
    Eigen::Vector3d A_g;

    CoMKFInput input;
    CoMKFState state;
    std::string imuName;

    std::shared_ptr<iDynTree::KinDynComputations> kinDyn;

    // maintain some iDynTree buffers
    // not using manif here because of depends on KinDyn
    Eigen::Matrix3d Sw; // S(omega)
    iDynTree::Position A_o_G;
    iDynTree::Transform A_H_IMU;
    iDynTree::Position A_o_IMU;
    iDynTree::Rotation A_R_IMU;
    iDynTree::Position IMU_o_G;
    bool initialized{false};
};

CenterOfMassKalmanFilter::CenterOfMassKalmanFilter() :
m_pimpl(std::make_unique<CenterOfMassKalmanFilter::Impl>())
{
    m_pimpl->I2.setIdentity();
    m_pimpl->I6.setIdentity();

    m_pimpl->A.setIdentity();
    m_pimpl->B.setZero();
    m_pimpl->C.setZero();
    m_pimpl->D.setZero();

    m_pimpl->P.setZero();
    m_pimpl->Q.setZero();
    m_pimpl->R.setZero();

    m_pimpl->CT.setZero();
    m_pimpl->S.setZero();
    m_pimpl->K.setZero();

    m_pimpl->z.setZero();
    m_pimpl->y.setZero();

    m_pimpl->A_g.setZero();
    m_pimpl->A_g(2) = -BipedalLocomotion::Math::StandardAccelerationOfGravitation;
}

CenterOfMassKalmanFilter::~CenterOfMassKalmanFilter()
{
}

bool CenterOfMassKalmanFilter::setInitialState(const CoMKFState& initialState)
{
    m_pimpl->x.segment<2>(0) = initialState.comPosition;
    m_pimpl->x.segment<2>(2) = initialState.comVelocity;
    m_pimpl->x.segment<2>(4) = initialState.comOffset;
    return true;
}

bool CenterOfMassKalmanFilter::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    auto handle = handler.lock();
    const std::string printPrefix{"[CenterOfMassKalmanFilter::initialize]"};
    if (handle == nullptr)
    {
        log()->error("{} The parameter handler has expired. "
                     "Please check its scope.", printPrefix);
        return false;
    }

    if (!handle->getParameter("sampling_time", m_pimpl->dt))
    {
        log()->error("{} Unable to initialize the sampling time.", printPrefix);
        return false;
    }

    if (!handle->getParameter("base_link_imu", m_pimpl->imuName))
    {
        log()->error("{} Unable to initialize the IMU name.", printPrefix);
        return false;
    }

    if (!handle->getParameter("com_position_measurement_noise_var", m_pimpl->sigma2_y_com))
    {
        log()->error("{} Unable to initialize the com position measurement noise variance.", printPrefix);
        return false;
    }

    if (!handle->getParameter("com_acceleration_measurement_noise_var", m_pimpl->sigma2_y_ddcom))
    {
        log()->error("{} Unable to initialize the IMU name.", printPrefix);
        return false;
    }

    if (!handle->getParameter("com_position_prediction_noise_var", m_pimpl->sigma2_x_com))
    {
        log()->error("{} Unable to initialize the IMU name.", printPrefix);
        return false;
    }

    if (!handle->getParameter("com_velocity_prediction_noise_var", m_pimpl->sigma2_dx_com))
    {
        log()->error("{} Unable to initialize the IMU name.", printPrefix);
        return false;
    }

    if (!handle->getParameter("com_offset_prediction_noise_var", m_pimpl->sigma2_xoff_com))
    {
        log()->error("{} Unable to initialize the IMU name.", printPrefix);
        return false;
    }

    m_pimpl->prepareQandR();

    return true;
}

bool CenterOfMassKalmanFilter::setKinDynObject(std::shared_ptr<iDynTree::KinDynComputations> kinDyn)
{
    const std::string printPrefix{"[CenterOfMassKalmanFilter::setKinDynObject]"};
    if (m_pimpl->initialized)
    {
        log()->error("{} Please initialize the estimator before calling this method.");
        return false;
    }

    if (!kinDyn->isValid())
    {
        log()->error("{} Invalid KinDynComputation object.");
        return false;
    }

    auto imuIdx = kinDyn->model().getFrameIndex(m_pimpl->imuName);
    if (imuIdx == iDynTree::FRAME_INVALID_INDEX)
    {
        log()->error("{} The configured IMU does not seem to be available in the loaded model.");
        return false;
    }

    auto baseLinkIdx = kinDyn->model().getFrameLink(imuIdx);
    if (kinDyn->model().getFrameName(baseLinkIdx) != kinDyn->getFloatingBase())
    {
        log()->error("{} The configured IMU does not seem to be associated with the base link.");
        return false;
    }

    m_pimpl->kinDyn = kinDyn;
    return true;
}

bool CenterOfMassKalmanFilter::advance()
{
    if (!isOutputValid())
    {
        log()->error("{} Please initialize the estimator and"
        "set KinDynComputations object before calling this method.");
        return false;
    }

    m_pimpl->A_o_G = m_pimpl->kinDyn->getCenterOfMassPosition();
    const auto& g = BipedalLocomotion::Math::StandardAccelerationOfGravitation;
    m_pimpl->omegaSquared = g/m_pimpl->A_o_G(2); // g/z_{com}

    if (!m_pimpl->predict())
    {
        log()->error("{} Failed to propagate the states.");
        return false;
    }

    if (!m_pimpl->update())
    {
        log()->error("{} Failed to update the states with measurements.");
        return false;
    }

    // update exposed states
    m_pimpl->state.comPosition = m_pimpl->x.head<2>();
    m_pimpl->state.comVelocity = m_pimpl->x.segment<2>(2);
    m_pimpl->state.comOffset = m_pimpl->x.tail<2>();

    return true;
}

bool CenterOfMassKalmanFilter::Impl::predict()
{
    prepareAandB();
    const Eigen::Vector2d& u = input.globalCoP;

    // KF prediction equations
    x = A*x + B*u;
    P = A*P*(A.transpose()) + Q;

    return true;
}

bool CenterOfMassKalmanFilter::Impl::update()
{
    prepareCandD();
    prepareMeasurementVector();

    // predicted measurement
    const Eigen::Vector2d& u = input.globalCoP;
    z = C*x + D*u;

    // KF update equations
    CT = C.transpose();
    S = C*P*CT + R;
    K = P*CT*(S.inverse());

    x += K*(y - z);
    P = (I6 - K*C)*P;

    return true;
}

void CenterOfMassKalmanFilter::Impl::prepareMeasurementVector()
{
    // assuming z axis of inertial frame perpendicular to gravity
    // get com horizontal position from kinematics
    // y = [    horizontal com position from kinematics        ]
    //     [horizontal com acceleration from kinematics and IMU]
    A_H_IMU = kinDyn->getWorldTransform(imuName);
    A_o_IMU = A_H_IMU.getPosition();
    A_R_IMU = A_H_IMU.getRotation();

    IMU_o_G = A_R_IMU.inverse()*(A_o_G - A_o_IMU);

    Sw = iDynTree::skew(input.gyro);

    // get Eigen refs
    auto R_imu = iDynTree::toEigen(A_R_IMU);
    auto r = iDynTree::toEigen(IMU_o_G);
    auto com = iDynTree::toEigen(A_o_G);

    Eigen::Vector3d A_odoubledot_G;
    A_odoubledot_G = R_imu*input.acc + A_g + (R_imu*Sw*Sw*r);

    // serialize measurements
    y.head<2>() = com.head<2>();
    y.tail<2>() =  A_odoubledot_G.head<2>();
}

void CenterOfMassKalmanFilter::Impl::prepareAandB()
{
    // A = [ I + (0.5 w^2 dT^2) I    dT I   (0.5*w^2 dT^2) I ]
    //     [         (w^2 dT^2) I       I       (w^2 dT^2) I ]
    //     [                    0       0                  I ]
    // I is a 2x2 identity matrix
    // w - LIPM eigenfrequency = sqrt(g/com_height)
    // dt - sampling period

    double halfOmegaSq{0.5*omegaSquared};
    double dtSq{dt*dt};

    // hoping these matrix block assignments are not
    // more expensive than scalar assignments
    A.topRightCorner<2, 2>() = (halfOmegaSq*dtSq)*I2;
    A.topLeftCorner<2, 2>() = I2 + A.topRightCorner<2, 2>();
    A.block<2, 2>(0, 2) = dt*I2;
    A.block<2, 2>(2, 0) = A.block<2, 2>(2, 2) = omegaSquared*dtSq*I2;

    // B = [ -(0.5 w^2 dT^2) I ]
    //     [     -(w^2 dT^2) I ]
    //     [                 0 ]
    B.topLeftCorner<2, 2>() = - A.topRightCorner<2, 2>();
    B.block<2, 2>(2, 0) = - A.block<2, 2>(2, 2);
}

void CenterOfMassKalmanFilter::Impl::prepareCandD()
{
    // C = [     I        0           0]
    //     [ w^2 I        0       w^2 I]
    C.topLeftCorner<2, 2>() = I2;
    C.bottomLeftCorner<2, 2>() = C.bottomRightCorner<2, 2>() = omegaSquared*I2;

    // D = [       0 ]
    //     [ - w^2 I ]
    D.bottomRightCorner<2, 2>() = -C.bottomRightCorner<2, 2>();
}

void CenterOfMassKalmanFilter::Impl::prepareQandR()
{
    Q.topRightCorner<2, 2>() = sigma2_x_com*I2;
    Q.block<2, 2>(2, 2) = sigma2_dx_com*I2;
    Q.bottomRightCorner<2, 2>() = sigma2_xoff_com*I2;

    R.topLeftCorner<2, 2>() = sigma2_y_com*I2;
    R.bottomRightCorner<2, 2>() = sigma2_y_ddcom*I2;
}

bool CenterOfMassKalmanFilter::setInput(const CoMKFInput& input)
{
    m_pimpl->input = input;
    return true;
}

bool CenterOfMassKalmanFilter::setGlobalCenterOfPressure(const double& copX, const double& copY)
{
    m_pimpl->input.globalCoP << copX, copY;
    return true;
}

bool CenterOfMassKalmanFilter::setBaseLinkIMUMeasurement(Eigen::Ref<Eigen::Vector3d> acc,
                                                         Eigen::Ref<Eigen::Vector3d> gyro)
{
    m_pimpl->input.acc = acc;
    m_pimpl->input.gyro = gyro;
    return true;
}

const CoMKFState& CenterOfMassKalmanFilter::getOutput() const
{
    return m_pimpl->state;
}

bool CenterOfMassKalmanFilter::isOutputValid() const
{
    return m_pimpl->initialized &&
           (m_pimpl->kinDyn != nullptr) &&
           m_pimpl->kinDyn->isValid();
}
