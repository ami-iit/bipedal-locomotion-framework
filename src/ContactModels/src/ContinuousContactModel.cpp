/**
 * @file ContinuousContactModel.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/ContactModels/ContinuousContactModel.h>

using namespace BipedalLocomotionControllers::ContactModels;

void ContinuousContactModel::setState(const std::unordered_map<std::string, std::any>& state)
{
    if (!getVariable(state, "twist", m_twist))
        throw std::runtime_error("[ContinuousContactModel::setState] Unable to get the variable "
                                 "named twist");

    if (!getVariable(state, "frame_transform", m_frameTransform))
        throw std::runtime_error("[ContinuousContactModel::setState] Unable to get the variable "
                                 "named frame_transform");

    if (!getVariable(state, "null_force_transform", m_nullForceTransform))
        throw std::runtime_error("[ContinuousContactModel::setState] Unable to get the variable "
                                 "named null_force_transform");

    // the parameters has been update the previous quantities has to be evaluated again
    m_isContactWrenchComputed = false;
    m_isControlMatrixComputed = false;
    m_isAutonomusDynamicsComputed = false;
}

void ContinuousContactModel::setMutableParameters(
    const std::unordered_map<std::string, std::any>& parameters)
{
    if (!getVariable(parameters, "spring_coeff", m_springCoeff))
        throw std::runtime_error("[ContinuousContactModel::setMutableParameters] Unable to get the "
                                 "variable named spring_coeff");

    if (!getVariable(parameters, "damper_coeff", m_damperCoeff))
        throw std::runtime_error("[ContinuousContactModel::setMutableParameters] Unable to get the "
                                 "variable named damper_coeff");

    // the parameters has been update the previous quantities has to be evaluated again
    m_isContactWrenchComputed = false;
    m_isControlMatrixComputed = false;
    m_isAutonomusDynamicsComputed = false;
}

void ContinuousContactModel::setImmutableParameters(
    const std::unordered_map<std::string, std::any>& parameters)
{
    if (!getVariable(parameters, "length", m_length))
        throw std::runtime_error("[ContinuousContactModel::setImmutableParameters] Unable to get "
                                 "the variable named length");

    if (!getVariable(parameters, "width", m_width))
        throw std::runtime_error("[ContinuousContactModel::setImmutableParameters] Unable to get "
                                 "the variable named width");

    // the parameters has been update the previous quantities has to be evaluated again
    m_isContactWrenchComputed = false;
    m_isControlMatrixComputed = false;
    m_isAutonomusDynamicsComputed = false;
}

ContinuousContactModel::ContinuousContactModel(
    const std::unordered_map<std::string, std::any>& immutableParameters,
    const std::unordered_map<std::string, std::any>& mutableParameters)
    : m_twist(iDynTree::Twist::Zero())
    , m_frameTransform(iDynTree::Transform::Identity())
    , m_nullForceTransform(iDynTree::Transform::Identity())
{
    setMutableParameters(mutableParameters);
    setImmutableParameters(immutableParameters);

    // reset matrix and vectors
    m_autonomousDynamics.zero();
    m_controlMatrix.zero();
}

ContinuousContactModel::ContinuousContactModel(
    const std::unordered_map<std::string, std::any>& parameters)
    : m_twist(iDynTree::Twist::Zero())
    , m_frameTransform(iDynTree::Transform::Identity())
    , m_nullForceTransform(iDynTree::Transform::Identity())
{
    setMutableParameters(parameters);
    setImmutableParameters(parameters);

    // reset matrix and vectors
    m_autonomousDynamics.zero();
    m_controlMatrix.zero();
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
