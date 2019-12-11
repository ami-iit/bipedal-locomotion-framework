/**
 * @file PID.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/PID.h>

using namespace BipedalLocomotionControllers;

const iDynTree::Vector3& CartesianPID::getControllerOutput()
{
    evaluateControl();
    return m_controllerOutput;
}

void OrientationPID::setGains(const double& c0, const double& c1, const double& c2)
{
    m_c0 = c0;
    m_c1 = c1;
    m_c2 = c2;

    m_controllerOutputEvaluated = false;
}

void OrientationPID::setDesiredTrajectory(const iDynTree::Vector3& desiredAcceleration,
                                          const iDynTree::Vector3& desiredVelocity,
                                          const iDynTree::Rotation& desiredOrientation)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredOrientation = desiredOrientation;

    m_controllerOutputEvaluated = false;
}

void OrientationPID::setFeedback(const iDynTree::Vector3& velocity,
                                 const iDynTree::Rotation& orientation)
{
    m_velocity = velocity;
    m_orientation = orientation;

    m_controllerOutputEvaluated = false;
}

iDynTree::Matrix3x3 OrientationPID::skewSymmetric(const iDynTree::Matrix3x3& input)
{
    iDynTree::Matrix3x3 output;
    iDynTree::toEigen(output)
        = 0.5 * (iDynTree::toEigen(input) - iDynTree::toEigen(input).transpose());
    return output;
}

void OrientationPID::evaluateControl()
{
    if (m_controllerOutputEvaluated)
        return;

    iDynTree::Matrix3x3 errorAttitude;
    Eigen::Vector3d error;
    errorAttitude = skewSymmetric(m_orientation * m_desiredOrientation.inverse());
    error = iDynTree::unskew(iDynTree::toEigen(errorAttitude));

    Eigen::Vector3d dotError;
    iDynTree::Matrix3x3 dotErrorAttitude;

    Eigen::Matrix3d skewAngularVelocity = iDynTree::skew(iDynTree::toEigen(m_velocity));
    Eigen::Matrix3d skewDesiredAngularVelocity
        = iDynTree::skew(iDynTree::toEigen(m_desiredVelocity));

    iDynTree::toEigen(dotErrorAttitude)
        = skewAngularVelocity * iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse())
          - iDynTree::toEigen(m_orientation * m_desiredOrientation.inverse())
                * skewDesiredAngularVelocity;

    dotError = iDynTree::unskew(iDynTree::toEigen(skewSymmetric(dotErrorAttitude)));

    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput)
        = iDynTree::toEigen(m_desiredAcceleration) - m_c0 * dotError
          - m_c1 * (iDynTree::toEigen(m_velocity) - iDynTree::toEigen(m_desiredVelocity))
          - m_c2 * error;

    m_controllerOutputEvaluated = true;
}

void PositionPID::setGains(const double& kp, const double& kd)
{
    for (int i = 0; i < 3; i++)
    {
        m_kp(i) = kp;
        m_kd(i) = kd;
    }

    m_controllerOutputEvaluated = false;
}

void PositionPID::setGains(const iDynTree::Vector3& kp, const iDynTree::Vector3& kd)
{
    m_kp = kp;
    m_kd = kd;

    m_controllerOutputEvaluated = false;
}

void PositionPID::setDesiredTrajectory(const iDynTree::Vector3& desiredAcceleration,
                                       const iDynTree::Vector3& desiredVelocity,
                                       const iDynTree::Vector3& desiredPosition)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredPosition = desiredPosition;

    m_controllerOutputEvaluated = false;
}

void PositionPID::setFeedback(const iDynTree::Vector3& velocity, const iDynTree::Vector3& position)
{
    m_velocity = velocity;
    m_position = position;

    m_controllerOutputEvaluated = false;
}

void PositionPID::evaluateControl()
{
    if (m_controllerOutputEvaluated)
        return;

    iDynTree::toEigen(m_error)
        = iDynTree::toEigen(m_desiredPosition) - iDynTree::toEigen(m_position);
    iDynTree::toEigen(m_dotError)
        = iDynTree::toEigen(m_desiredVelocity) - iDynTree::toEigen(m_velocity);

    iDynTree::toEigen(m_controllerOutput)
        = iDynTree::toEigen(m_desiredAcceleration)
          + iDynTree::toEigen(m_kp).asDiagonal() * iDynTree::toEigen(m_error)
          + iDynTree::toEigen(m_kd).asDiagonal() * iDynTree::toEigen(m_dotError);

    m_controllerOutputEvaluated = true;
}

void OneDegreePID::evaluateControl()
{
    if (m_controllerOutputEvaluated)
        return;

    m_controllerOutput = m_desiredAcceleration + m_kp * (m_desiredPosition - m_position)
                         + m_kd * (m_desiredVelocity - m_velocity);

    m_controllerOutputEvaluated = true;
}

void OneDegreePID::setGains(const double& kp, const double& kd)
{
    m_kp = kp;
    m_kd = kd;

    m_controllerOutputEvaluated = false;
}

void OneDegreePID::setDesiredTrajectory(const double& desiredAcceleration,
                                        const double& desiredVelocity,
                                        const double& desiredPosition)
{
    m_desiredAcceleration = desiredAcceleration;
    m_desiredVelocity = desiredVelocity;
    m_desiredPosition = desiredPosition;

    m_controllerOutputEvaluated = false;
}

void OneDegreePID::setFeedback(const double& velocity, const double& position)
{
    m_velocity = velocity;
    m_position = position;

    m_controllerOutputEvaluated = false;
}

const double& OneDegreePID::getControllerOutput()
{
    evaluateControl();
    return m_controllerOutput;
}
