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

    m_isContactWrenchComputed = false;
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

    m_isContactWrenchComputed = false;
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

    m_isContactWrenchComputed = false;
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
}

ContinuousContactModel::ContinuousContactModel(
    const std::unordered_map<std::string, std::any>& parameters)
    : m_twist(iDynTree::Twist::Zero())
    , m_frameTransform(iDynTree::Transform::Identity())
    , m_nullForceTransform(iDynTree::Transform::Identity())
{
    setMutableParameters(parameters);
    setImmutableParameters(parameters);
}

void ContinuousContactModel::computeContactWrench()
{
    if (m_isContactWrenchComputed)
        return;

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

    m_isContactWrenchComputed = true;
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
