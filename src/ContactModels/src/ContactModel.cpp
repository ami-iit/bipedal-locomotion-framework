/**
 * @file ContactModel.cpp
 * @authors Giulio Romualdi
 * @copyright 2020 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the GNU Lesser General Public License v2.1 or any later version.
 */

#include <BipedalLocomotionControllers/ContactModels/ContactModel.h>

using namespace BipedalLocomotionControllers::ContactModels;

const iDynTree::Wrench& ContactModel::getContactWrench()
{
    if (!m_isContactWrenchComputed)
    {
        computeContactWrench();
        m_isContactWrenchComputed = true;
    }

    return m_contactWrench;
}

const iDynTree::Vector6& ContactModel::getAutonomousDynamics()
{
    if (!m_isAutonomusDynamicsComputed)
    {
        computeAutonomousDynamics();
        m_isAutonomusDynamicsComputed = true;
    }

    return m_autonomousDynamics;
}

const iDynTree::Matrix6x6& ContactModel::getControlMatrix()
{
    if (!m_isControlMatrixComputed)
    {
        computeControlMatrix();
        m_isControlMatrixComputed = true;
    }

    return m_controlMatrix;
}
