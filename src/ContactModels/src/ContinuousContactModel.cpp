/**
 * @file ContinuousContactModel.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#include <iDynTree/EigenHelpers.h>

#include <BipedalLocomotion/ContactModels/ContinuousContactModel.h>
#include <BipedalLocomotion/ParametersHandler/IParametersHandler.h>

using namespace BipedalLocomotion::ContactModels;
using namespace BipedalLocomotion::ParametersHandler;

ContinuousContactModel::ContinuousContactModel()
{
    m_controlMatrix.zero();
    m_autonomousDynamics.zero();
    m_regressor.resize(6, 2);
    m_regressor.zero();
}

bool ContinuousContactModel::initializePrivate(std::weak_ptr<ParametersHandler::IParametersHandler> weakHandler)
{
    auto handler = weakHandler.lock();
    if (handler == nullptr)
    {
        std::cerr << "[ContinuousContactModel::initialize] The parameter handler is corrupted. "
                     "Please make sure that the handler exists."
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("length", m_length))
    {
        std::cerr << "[ContinuousContactModel::initialize] Unable to get the variable named length."
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("width", m_width))
    {
        std::cerr << "[ContinuousContactModel::initialize] Unable to get the variable named width."
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("spring_coeff", m_springCoeff))
    {
        std::cerr << "[ContinuousContactModel::initialize] Unable to get the variable named "
                     "spring_coeff."
                  << std::endl;
        return false;
    }

    if (!handler->getParameter("damper_coeff", m_damperCoeff))
    {
        std::cerr << "[ContinuousContactModel::initialize] Unable to get the variable named "
                     "damper_coeff."
                  << std::endl;
        return false;
    }
    return true;
}

void ContinuousContactModel::setNullForceTransformPrivate(const iDynTree::Transform& transform)
{
    m_nullForceTransform = transform;
}

void ContinuousContactModel::setStatePrivate(const iDynTree::Twist& twist,
                                             const iDynTree::Transform& transform)
{
    m_twist = twist;
    m_frameTransform = transform;
}

void ContinuousContactModel::computeContactWrench()
{
    double area = m_length * m_width;

    auto force(iDynTree::toEigen(m_contactWrench.getLinearVec3()));
    auto torque(iDynTree::toEigen(m_contactWrench.getAngularVec3()));

    auto position(iDynTree::toEigen(m_frameTransform.getPosition()));
    auto rotation(iDynTree::toEigen(m_frameTransform.getRotation()));

    auto linearVelocity(iDynTree::toEigen(m_twist.getLinearVec3()));
    auto angularVelocity(iDynTree::toEigen(m_twist.getAngularVec3()));

    auto nullForcePosition(iDynTree::toEigen(m_nullForceTransform.getPosition()));
    auto nullForceRotation(iDynTree::toEigen(m_nullForceTransform.getRotation()));

    // compute the force
    force = std::abs(rotation(2, 2)) * area * (m_springCoeff * (nullForcePosition - position)
                                               - m_damperCoeff * linearVelocity);

    // compute the torque
    auto skewRe1 = iDynTree::skew(rotation.col(0));
    auto skewRe2 = iDynTree::skew(rotation.col(1));
    torque = std::abs(rotation(2, 2)) * area / 12 * (m_length * m_length
                                                     * (m_damperCoeff * skewRe1 * skewRe1 * angularVelocity
                                                        + m_springCoeff * skewRe1 * nullForceRotation.col(0))
                                                     + m_width * m_width
                                                     * (m_damperCoeff * skewRe2 * skewRe2 * angularVelocity
                                                        + m_springCoeff * skewRe2 * nullForceRotation.col(1)));
}

void ContinuousContactModel::computeAutonomousDynamics()
{
    double area = m_length * m_width;

    auto autonomousDynamics(iDynTree::toEigen(m_autonomousDynamics));

    auto position(iDynTree::toEigen(m_frameTransform.getPosition()));
    auto rotation(iDynTree::toEigen(m_frameTransform.getRotation()));

    auto linearVelocity(iDynTree::toEigen(m_twist.getLinearVec3()));
    auto angularVelocity(iDynTree::toEigen(m_twist.getAngularVec3()));

    auto nullForcePosition(iDynTree::toEigen(m_nullForceTransform.getPosition()));
    auto nullForceRotation(iDynTree::toEigen(m_nullForceTransform.getRotation()));

    Eigen::Matrix3d rotationRateOfChange = iDynTree::skew(angularVelocity) * rotation;

    autonomousDynamics.head(3) = area * (rotationRateOfChange(2,2)  * (m_springCoeff * (nullForcePosition - position)
                                                                       - m_damperCoeff * linearVelocity)
                                         - rotation(2,2) * m_springCoeff * linearVelocity);

    auto skewRe1 = iDynTree::skew(rotation.col(0));
    auto skewRe2 = iDynTree::skew(rotation.col(1));
    auto skewDotRe1 = iDynTree::skew(rotationRateOfChange.col(0));
    auto skewDotRe2 = iDynTree::skew(rotationRateOfChange.col(1));

    autonomousDynamics.tail(3)
        = area / 12 * (rotationRateOfChange(2, 2) * (m_length * m_length * (m_damperCoeff * skewRe1 * skewRe1 * angularVelocity
                                                                            + m_springCoeff * skewRe1 * nullForceRotation.col(0))
                                                     + m_width * m_width * (m_damperCoeff * skewRe2 * skewRe2 * angularVelocity
                                                                            + m_springCoeff * skewRe2 * nullForceRotation.col(1)))
                       + rotation(2, 2) * (m_length * m_length * (m_springCoeff * skewDotRe1 * nullForceRotation.col(0)
                                                                  + m_damperCoeff * (skewDotRe1 * skewRe1 + skewRe1 * skewDotRe1) * angularVelocity)
                                           + (m_width * m_width * (m_springCoeff * skewDotRe2 * nullForceRotation.col(1)
                                                                   + m_damperCoeff * (skewDotRe2 * skewRe2 + skewRe2 * skewDotRe2) * angularVelocity))));

}

void ContinuousContactModel::computeControlMatrix()
{
    double area = m_length * m_width;

    auto controlMatrix(iDynTree::toEigen(m_controlMatrix));

    auto position(iDynTree::toEigen(m_frameTransform.getPosition()));
    auto rotation(iDynTree::toEigen(m_frameTransform.getRotation()));

    auto linearVelocity(iDynTree::toEigen(m_twist.getLinearVec3()));
    auto angularVelocity(iDynTree::toEigen(m_twist.getAngularVec3()));

    auto nullForcePosition(iDynTree::toEigen(m_nullForceTransform.getPosition()));
    auto nullForceRotation(iDynTree::toEigen(m_nullForceTransform.getRotation()));

    Eigen::Matrix3d  rotationRateOfChange = iDynTree::skew(angularVelocity) * rotation;

    controlMatrix.topLeftCorner(3, 3).diagonal().array() = - area * m_damperCoeff * rotation(2,2);

    auto skewRe1 = iDynTree::skew(rotation.col(0));
    auto skewRe2 = iDynTree::skew(rotation.col(1));
    controlMatrix.bottomRightCorner(3, 3) = area / 12 * rotation(2,2) * m_damperCoeff *
        (m_length * m_length *  skewRe1 * skewRe1 + m_width * m_width *  skewRe2 * skewRe2);
}

iDynTree::Force ContinuousContactModel::getForceAtPoint(const double& x, const double& y)
{
    auto position(iDynTree::toEigen(m_frameTransform.getPosition()));
    auto rotation(iDynTree::toEigen(m_frameTransform.getRotation()));

    auto linearVelocity(iDynTree::toEigen(m_twist.getLinearVec3()));
    auto angularVelocity(iDynTree::toEigen(m_twist.getAngularVec3()));

    auto nullForcePosition(iDynTree::toEigen(m_nullForceTransform.getPosition()));
    auto nullForceRotation(iDynTree::toEigen(m_nullForceTransform.getRotation()));

    iDynTree::Force force;
    if (std::abs(x) > m_length / 2 || std::abs(y) > m_width / 2)
    {
        force.zero();
        return force;
    }

    iDynTree::Vector3 pointPosition;
    pointPosition(0) = x;
    pointPosition(1) = y;
    pointPosition(2) = 0;

    iDynTree::toEigen(force) =
        m_springCoeff * ((nullForcePosition - position)
                         + (nullForceRotation - rotation) * iDynTree::toEigen(pointPosition))
        - m_damperCoeff * (linearVelocity + iDynTree::skew(angularVelocity) * rotation * iDynTree::toEigen(pointPosition));

    return force;
}

iDynTree::Torque ContinuousContactModel::getTorqueGeneratedAtPoint(const double& x, const double& y)
{
    iDynTree::Torque torque;
    if (std::abs(x) > m_length / 2 || std::abs(y) > m_width / 2)
    {
        torque.zero();
        return torque;
    }

    auto rotation(iDynTree::toEigen(m_frameTransform.getRotation()));
    iDynTree::Vector3 pointPosition;
    pointPosition(0) = x;
    pointPosition(1) = y;
    pointPosition(2) = 0;

    iDynTree::toEigen(torque) = (rotation * iDynTree::toEigen(pointPosition)).cross(iDynTree::toEigen(getForceAtPoint(x, y)));
    return torque;
}

void ContinuousContactModel::computeRegressor()
{
    double area = m_length * m_width;
    auto position(iDynTree::toEigen(m_frameTransform.getPosition()));
    auto rotation(iDynTree::toEigen(m_frameTransform.getRotation()));

    auto linearVelocity(iDynTree::toEigen(m_twist.getLinearVec3()));
    auto angularVelocity(iDynTree::toEigen(m_twist.getAngularVec3()));

    auto nullForcePosition(iDynTree::toEigen(m_nullForceTransform.getPosition()));
    auto nullForceRotation(iDynTree::toEigen(m_nullForceTransform.getRotation()));

    auto skewRe1 = iDynTree::skew(rotation.col(0));
    auto skewRe2 = iDynTree::skew(rotation.col(1));

    auto regressor(iDynTree::toEigen(m_regressor));

    regressor.topLeftCorner<3, 1>()
        = std::abs(rotation(2, 2)) * area * (nullForcePosition - position);

    regressor.topRightCorner<3, 1>() = -std::abs(rotation(2, 2)) * area * linearVelocity;

    regressor.bottomLeftCorner<3, 1>()
        = area / 12.0 * std::abs(rotation(2, 2))
          * (m_length * m_length * skewRe1 * nullForceRotation.col(0)
             + m_width * m_width * skewRe2 * nullForceRotation.col(1));

    regressor.bottomRightCorner<3, 1>()
        = area / 12.0 * std::abs(rotation(2, 2))
          * (m_length * m_length * skewRe1 * skewRe1 + m_width * m_width * skewRe2 * skewRe2)
          * angularVelocity;
}

const double& ContinuousContactModel::springCoeff() const
{
    return m_springCoeff;
}

double& ContinuousContactModel::springCoeff()
{
    return m_springCoeff;
}

const double& ContinuousContactModel::damperCoeff() const
{
    return m_damperCoeff;
}

double& ContinuousContactModel::damperCoeff()
{
    return m_damperCoeff;
}
