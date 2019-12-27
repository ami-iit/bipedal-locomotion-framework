/**
 * @file PDController.cpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

// iDynTree
#include <iDynTree/Core/EigenHelpers.h>

#include <BipedalLocomotionControllers/OptimalControlUtilities/PDController.h>

using namespace BipedalLocomotionControllers::OptimalControlUtilities;

void OrientationPD::setGains(const double& c0, const double& c1, const double& c2)
{
    m_c0 = c0;
    m_c1 = c1;
    m_c2 = c2;

    m_controllerOutputEvaluated = false;
}

iDynTree::Matrix3x3 OrientationPD::skewSymmetric(const iDynTree::Matrix3x3& input)
{
    iDynTree::Matrix3x3 output;
    iDynTree::toEigen(output)
        = 0.5 * (iDynTree::toEigen(input) - iDynTree::toEigen(input).transpose());
    return output;
}

void OrientationPD::evaluateControl()
{
    if (m_controllerOutputEvaluated)
        return;

    iDynTree::Matrix3x3 errorAttitude;
    Eigen::Vector3d error;
    errorAttitude = skewSymmetric(m_state * m_reference.inverse());
    error = iDynTree::unskew(iDynTree::toEigen(errorAttitude));

    Eigen::Vector3d dotError;
    iDynTree::Matrix3x3 dotErrorAttitude;

    Eigen::Matrix3d skewAngularVelocity = iDynTree::skew(iDynTree::toEigen(m_stateDerivative));
    Eigen::Matrix3d skewDesiredAngularVelocity
        = iDynTree::skew(iDynTree::toEigen(m_referenceDerivative));

    iDynTree::toEigen(dotErrorAttitude)
        = skewAngularVelocity * iDynTree::toEigen(m_state * m_reference.inverse())
          - iDynTree::toEigen(m_state * m_reference.inverse()) * skewDesiredAngularVelocity;

    dotError = iDynTree::unskew(iDynTree::toEigen(skewSymmetric(dotErrorAttitude)));

    // evaluate the control law
    iDynTree::toEigen(m_controllerOutput)
        = iDynTree::toEigen(m_feedforward) - m_c0 * dotError
          - m_c1 * (iDynTree::toEigen(m_stateDerivative) - iDynTree::toEigen(m_referenceDerivative))
          - m_c2 * error;

    m_controllerOutputEvaluated = true;
}

template <> void LinearPD<double>::evaluateControl()
{
    if (m_controllerOutputEvaluated)
        return;

    m_controllerOutput = m_feedforward + m_kp * (m_reference - m_state)
                         + m_kd * (m_referenceDerivative - m_stateDerivative);

    m_controllerOutputEvaluated = true;
}

template <> void LinearPD<iDynTree::VectorDynSize>::evaluateControl()
{
    if (this->m_controllerOutputEvaluated)
        return;

    this->m_controllerOutput.resize(m_kp.size());

    iDynTree::toEigen(this->m_controllerOutput)
        = iDynTree::toEigen(this->m_feedforward)
          + iDynTree::toEigen(m_kp).asDiagonal()
                * (iDynTree::toEigen(this->m_reference) - iDynTree::toEigen(this->m_state))
          + iDynTree::toEigen(m_kd).asDiagonal()
                * (iDynTree::toEigen(this->m_referenceDerivative)
                   - iDynTree::toEigen(this->m_stateDerivative));

    this->m_controllerOutputEvaluated = true;
}
