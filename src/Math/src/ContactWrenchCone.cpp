/**
 * @file ContactWrenchCone.cpp
 * @authors Giulio Romualdi
 * @copyright 2021 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <cmath>
#include <numeric>

#include <manif/manif.h>

#include <BipedalLocomotion/Math/ContactWrenchCone.h>
#include <BipedalLocomotion/Math/LinearizedFrictionCone.h>
#include <BipedalLocomotion/Math/Wrench.h>
#include <BipedalLocomotion/TextLogging/Logger.h>

using namespace BipedalLocomotion::Math;

bool ContactWrenchCone::initialize(std::weak_ptr<const ParametersHandler::IParametersHandler> handler)
{
    constexpr auto errorPrefix = "[ContactWrenchCone::initialize]";
    constexpr int wrenchSize = Wrench<double>::SizeAtCompileTime;

    LinearizedFrictionCone forceCone;

    auto ptr = handler.lock();
    if (ptr == nullptr)
    {
        log()->error("{} Invalid parameter handler.", errorPrefix);
        return false;
    }

    double staticFrictionCoefficient = -1;
    bool ok =  ptr->getParameter("static_friction_coefficient", staticFrictionCoefficient);
    ok = ok && staticFrictionCoefficient > 0;

    std::vector<double> limitsX, limitsY;
    ok = ok && ptr->getParameter("foot_limits_x", limitsX);
    ok = ok && ptr->getParameter("foot_limits_y", limitsY);
    ok = ok && (limitsX.size() == limitsY.size()) && (limitsX.size() == 2);

    if (!ok)
    {
        log()->error("{} Unable to retrieve all the parameters.", errorPrefix);
        return false;
    }

    if(!forceCone.initialize(handler))
    {
        log()->error("{} Unable to initialize the force friction cone.", errorPrefix);
        return false;
    }

    Eigen::Vector3d center;
    center << std::accumulate(limitsX.begin(), limitsX.end(), 0.0) / limitsX.size(),
              std::accumulate(limitsY.begin(), limitsY.end(), 0.0) / limitsY.size(),
              0;

    const double halfRectangleLength = (std::abs(limitsX[0]) + std::abs(limitsX[1])) / 2.0;
    const double halfRectangleWidth = (std::abs(limitsY[0]) + std::abs(limitsY[1])) / 2.0;

    constexpr std::size_t copConstraints = 2;
    constexpr std::size_t yawTorqueConstraints = 8;
    constexpr std::size_t torqueConstraints = 2 * copConstraints + yawTorqueConstraints;
    m_A.resize(forceCone.getB().size() + torqueConstraints, wrenchSize);
    m_A.setZero();
    m_A.topLeftCorner(forceCone.getA().rows(), forceCone.getA().cols()) = forceCone.getA();

    // take the lower part of the matrix A
    // this should simplify
    Eigen::Ref<Eigen::MatrixXd> ACoP = m_A.middleRows(forceCone.getB().size(), 2 * copConstraints);
    ACoP(0, 2) = limitsY[0];
    ACoP(1, 2) = -limitsY[1];
    ACoP(0, 3) = -1;
    ACoP(1, 3) = 1;
    ACoP(2, 2) = limitsX[0];
    ACoP(3, 2) = -limitsX[1];
    ACoP(2, 4) = 1;
    ACoP(3, 4) = -1;

    // please refer to the last 8 rows of the matrix U presented in
    // https://scaron.info/publications/icra-2015.html
    Eigen::MatrixXd AYawTorque(yawTorqueConstraints, 6);
    AYawTorque.row(0) << -halfRectangleWidth, -halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        staticFrictionCoefficient, staticFrictionCoefficient, -1;

    AYawTorque.row(1) << -halfRectangleWidth, halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        staticFrictionCoefficient, -staticFrictionCoefficient, -1;

    AYawTorque.row(2) << halfRectangleWidth, -halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        -staticFrictionCoefficient, staticFrictionCoefficient, -1;

    AYawTorque.row(3) << halfRectangleWidth, halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        -staticFrictionCoefficient, -staticFrictionCoefficient, -1;

    AYawTorque.row(4) << halfRectangleWidth, halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        staticFrictionCoefficient, staticFrictionCoefficient, 1;

    AYawTorque.row(5) << halfRectangleWidth, -halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        staticFrictionCoefficient, -staticFrictionCoefficient, 1;

    AYawTorque.row(6) << -halfRectangleWidth, halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        -staticFrictionCoefficient, staticFrictionCoefficient, 1;

    AYawTorque.row(7) << -halfRectangleWidth, -halfRectangleLength,
        -(halfRectangleWidth + halfRectangleLength) * staticFrictionCoefficient,
        -staticFrictionCoefficient, -staticFrictionCoefficient, 1;

    // AYawTorque expects a wrench expressed in the center of the contact surface. The following
    // adjoint transform brings the wrench expressed in the frame attached to the contact surface in
    // the center of the surface. The orientation of the two frames is identical and the translation
    // is given by the 3d vector named center.
    Eigen::Matrix<double, wrenchSize, wrenchSize> adjointTransform
        = Eigen::Matrix<double, wrenchSize, wrenchSize>::Identity();
    adjointTransform.bottomLeftCorner<3, 3>() = manif::skew(-center);

    m_A.bottomRows(yawTorqueConstraints).noalias() = AYawTorque * adjointTransform;

    m_b.resize(m_A.rows());
    m_b.head(forceCone.getB().size()) = forceCone.getB();
    m_b.tail(torqueConstraints).setZero();

    m_isIntialized = true;

    return true;
}

Eigen::Ref<const Eigen::MatrixXd> ContactWrenchCone::getA() const
{
    constexpr auto error = "[ContactWrenchCone::getA] Please initialize the class before.";

    if (!m_isIntialized)
    {
        log()->warn(error);
        assert(m_isIntialized && error);
    }

    return m_A;
}

Eigen::Ref<const Eigen::VectorXd> ContactWrenchCone::getB() const
{
    constexpr auto error = "[ContactWrenchCone::getB] Please initialize the class before.";
    if (!m_isIntialized)
    {
        log()->warn(error);
        assert(m_isIntialized && error);
    }

    return m_b;
}
