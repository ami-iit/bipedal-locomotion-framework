/**
 * @file PDController.tpp
 * @authors Giulio Romualdi
 * @copyright 2019 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <iDynTree/Core/EigenHelpers.h>
#include <type_traits>

namespace BipedalLocomotionControllers
{
namespace OptimalControlUtilities
{
template <class T, class U, class W>
void PDController<T, U, W>::setDesiredTrajectory(const T& feedforward,
                                                 const U& referenceDerivative,
                                                 const W& reference)
{
    m_feedforward = feedforward;
    m_referenceDerivative = referenceDerivative;
    m_reference = reference;

    m_controllerOutputEvaluated = false;
}

template <class T, class U, class W>
void PDController<T, U, W>::setFeedback(const U& stateDerivative, const W& state)
{
    m_stateDerivative = stateDerivative;
    m_state = state;

    m_controllerOutputEvaluated = false;
}

template <class T, class U, class W>
const T& PDController<T, U, W>::getControllerOutput()
{
    evaluateControl();
    return m_controllerOutput;
}

template <class T>
void LinearPD<T>::setGains(const T& kp, const T& kd)
{
    m_kp = kp;
    m_kd = kd;

    this->m_controllerOutputEvaluated = false;
}

template <class T>
void LinearPD<T>::evaluateControl()
{
    if (this->m_controllerOutputEvaluated)
        return;

    iDynTree::toEigen(this->m_controllerOutput)
        = iDynTree::toEigen(this->m_feedforward)
        + iDynTree::toEigen(m_kp).asDiagonal() * (iDynTree::toEigen(this->m_reference) - iDynTree::toEigen(this->m_state))
        + iDynTree::toEigen(m_kd).asDiagonal() * (iDynTree::toEigen(this->m_referenceDerivative) - iDynTree::toEigen(this->m_stateDerivative));

    this->m_controllerOutputEvaluated = true;
}

} // namespace OptimalControlUtilities
} // namespace BipedalLocomotionControllers
